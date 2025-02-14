from AAPI import *
from PyANGKernel import GKSystem
import numpy as np
import time
import math

##############################################################################
# GLOBAL VARIABLES (same as your code, except we remove step_counter)
##############################################################################
time_step = 15
amber_time = 2
all_red_time = 2

green_signal  = 1  # Green signal code in Aimsun API
red_signal    = 0  # Red signal code in Aimsun API
amber_signal  = 2  # Amber signal code in Aimsun API

junction_id = 1978

max_cycle_time = 8   # In terms of "phase picks" (your original meaning)
cycle_counter = 0

# For storing phase orders
previous_phases_list = []
remaining_phases_list = []

# NEW: State machine variables
signal_state = "GREEN"   # can be: "GREEN", "AMBER", or "ALLRED"
time_in_state = 0.0      # how many simulation seconds spent in the current state
current_phase = None     # which phase is currently green
next_phase = None        # which phase we'll switch to after amber/all-red

# Your original dictionaries
section_upstream_dict = {
    552: [658],
    789: [1116],
    786: [1064],
    572: [988],
    471: [3741],
    523: [3747],
    516: [547],
    499: [539],
    1405: [1483],
    1422: [1461],
    1409: [1408],
    1415: [1401],
    419: [437],
    979: [879],
    772: [3750],
    1084: [626],
    1140: [2294],
    1107: [1120],
    1102: [766]
}

section_downstream_dict = {
    778: [769],
    774: [557],
    780: [3750],
    776: [766],
    511: [509],
    502: [1644],
    2288: [517],
    2291: [513],
    1432: [1402],
    1428: [1446],
    1430: [1443],
    1436: [1449],
    2297: [1529],
    990: [2536],
    431: [2300],
    2303: [1143],
    1091: [614],
    1064: [786],
    1092: [1137]
}

# Get the active model if needed
model = GKSystem.getSystem().getActiveModel()

##############################################################################
# YOUR EXISTING HELPER FUNCTIONS (unchanged)
##############################################################################
def calculate_phase_pressure(junction_id, phase, time1, signal_group_veh_diff):
    num_phase_sg = ECIGetNbSignalGroupsPhaseofJunction(junction_id, phase, time1)
    weight_list = []
    for sg_index in range(num_phase_sg):
        signal_group_id = ECIGetSignalGroupPhaseofJunction(junction_id, phase, sg_index, time1)
        weight_list.append(signal_group_veh_diff[signal_group_id])
    pressure = sum(weight_list)
    return pressure

def get_signal_group_id(junction_id, critical_phase, time1):
    num_phase_sg = ECIGetNbSignalGroupsPhaseofJunction(junction_id, critical_phase, time1)
    signal_group_pool = []
    for sg_index in range(num_phase_sg):
        signal_group_id = ECIGetSignalGroupPhaseofJunction(junction_id, critical_phase, sg_index, time1)
        signal_group_pool.append(signal_group_id)
    return signal_group_pool

##############################################################################
# AIMSUN REQUIRED FUNCTIONS
##############################################################################
def AAPILoad():
    return 0

def AAPIInit():
    return 0

def AAPISimulationReady():
    return 0

def AAPIManage(time1, timeSta, timeTrans, acycle):
    return 0

##############################################################################
# MAIN LOGIC: STATE MACHINE INSIDE AAPIPostManage
##############################################################################
def AAPIPostManage(time1, timeSta, timeTrans, acycle):
    """
    Instead of forcing amber/all-red every 15 seconds, we:
      1) Stay in GREEN at least 'time_step' seconds.
      2) Then run the max-pressure logic to see if we switch phases.
      3) If same phase -> skip amber/all-red, just extend green.
         If different phase -> amber -> all-red -> new green.
    """
    global signal_state
    global time_in_state
    global current_phase
    global next_phase
    global cycle_counter
    global previous_phases_list
    global remaining_phases_list

    # Advance time in the current state
    time_in_state += acycle  # acycle = simulation step in seconds

    # ----------------------------------------------------
    # 1) If we are in GREEN
    # ----------------------------------------------------
    if signal_state == "GREEN":
        # If no phase is active yet, pick an initial phase immediately
        if current_phase is None:
            AKIPrintString("No current_phase set yet; picking an initial phase.")
            current_phase = pick_critical_phase(time1)  # We'll define inline below
            AKIPrintString(f"Initial phase chosen: {current_phase}")

            # Turn that phase's signal groups to GREEN, rest RED
            ECIDisableEvents(junction_id)
            total_sg = ECIGetNumberSignalGroups(junction_id)
            critical_sg_list = get_signal_group_id(junction_id, current_phase, time1)
            for sg_id in range(1, total_sg + 1):
                if sg_id in critical_sg_list:
                    ECIChangeSignalGroupState(junction_id, sg_id, green_signal, timeSta, time1, acycle)
                else:
                    ECIChangeSignalGroupState(junction_id, sg_id, red_signal, timeSta, time1, acycle)

            time_in_state = 0
            return 0

        # If we've been GREEN at least time_step seconds, consider switching
        if time_in_state >= time_step:
            AKIPrintString(f"Time in GREEN >= {time_step}s, checking next phase.")
            critical_phase_candidate = pick_critical_phase(time1)

            if critical_phase_candidate == current_phase:
                # No actual change: stay in GREEN
                AKIPrintString(f"Same phase {current_phase} chosen; extend GREEN.")
                time_in_state = 0
            else:
                # We plan to switch
                next_phase = critical_phase_candidate
                AKIPrintString(f"Phase {current_phase} -> {next_phase}: going to AMBER.")
                
                # Turn current_phase groups to AMBER, others RED
                ECIDisableEvents(junction_id)
                total_sg = ECIGetNumberSignalGroups(junction_id)
                curr_sg_list = get_signal_group_id(junction_id, current_phase, time1)
                for sg_id in range(1, total_sg + 1):
                    if sg_id in curr_sg_list:
                        ECIChangeSignalGroupState(junction_id, sg_id, amber_signal, timeSta, time1, acycle)
                    else:
                        ECIChangeSignalGroupState(junction_id, sg_id, red_signal, timeSta, time1, acycle)
                
                signal_state = "AMBER"
                time_in_state = 0

    # ----------------------------------------------------
    # 2) If we are in AMBER
    # ----------------------------------------------------
    elif signal_state == "AMBER":
        # After 'amber_time' seconds, go ALL-RED
        if time_in_state >= amber_time:
            AKIPrintString("AMBER time done, switching to ALL-RED.")
            ECIDisableEvents(junction_id)
            num_sg = ECIGetNumberSignalGroups(junction_id)
            for sg in range(1, num_sg + 1):
                ECIChangeSignalGroupState(junction_id, sg, red_signal, timeSta, time1, acycle)
            
            signal_state = "ALLRED"
            time_in_state = 0

    # ----------------------------------------------------
    # 3) If we are in ALLRED
    # ----------------------------------------------------
    elif signal_state == "ALLRED":
        # After 'all_red_time' seconds, finalize the new phase
        if time_in_state >= all_red_time:
            AKIPrintString("ALL-RED done, moving to new phase GREEN.")
            current_phase = next_phase  # finalize

            ECIDisableEvents(junction_id)
            num_sg = ECIGetNumberSignalGroups(junction_id)
            new_sg_list = get_signal_group_id(junction_id, current_phase, time1)
            for sg in range(1, num_sg + 1):
                if sg in new_sg_list:
                    ECIChangeSignalGroupState(junction_id, sg, green_signal, timeSta, time1, acycle)
                else:
                    ECIChangeSignalGroupState(junction_id, sg, red_signal, timeSta, time1, acycle)

            signal_state = "GREEN"
            time_in_state = 0

    return 0

def AAPIFinish():
    return 0

def AAPIUnLoad():
    return 0

def AAPIPreRouteChoiceCalculation(time1, timeSta):
    return 0

###############################################################################
# INLINE: "pick_critical_phase()" LOGIC
# We embed your big max-pressure code as a sub-block here.
###############################################################################
def pick_critical_phase(time1):
    """
    We replicate your big logic for:
      - enumerating phases (non-interphase),
      - counting vehicles for each signal group,
      - picking the next 'critical phase' 
      - using 'cycle_counter', 'previous_phases_list', 'remaining_phases_list'
    """
    global cycle_counter
    global previous_phases_list
    global remaining_phases_list

    # 1) Get all normal (non-interphase) phases
    num_phases = ECIGetNumberPhases(junction_id)
    all_phases = []
    for ph in range(1, num_phases + 1):
        if ECIIsAnInterPhase(junction_id, ph, time1) == 0:
            all_phases.append(ph)

    # 2) We do the big vehicle-counting approach
    num_signal_groups = ECIGetNumberSignalGroups(junction_id)
    signal_group_veh_diff = {}
    signal_group_name = {}
    origin_veh_num = {}
    origin_sec_num_lanes = {}
    dest_sec_num_lanes = {}
    all_signal_groups_id = []

    cycle_counter += 1
    AKIPrintString(f"JUNCTION ID = {junction_id}, cycle_counter = {cycle_counter}")

    for signal_group in range(1, num_signal_groups + 1):
        num_turnings = ECIGetNumberTurningsofSignalGroup(junction_id, signal_group)

        nonChar = boolp()
        sg_name = AKIConvertToAsciiString(ECIGetLogicalNameofSignalGroup(junction_id, signal_group), True, nonChar)

        for turning_index in range(num_turnings):
            fromSection = intp()
            toSection = intp()
            report = ECIGetFromToofTurningofSignalGroup(junction_id, signal_group, turning_index, fromSection, toSection)
            if report == 0:
                ## COUNT NUMBER OF VEHICLES IN UPSTREAM OF UPSTREAM LANES OF THE SIGNAL
                for upstream_split_section_id in section_upstream_dict[fromSection.value()]:
                    num_veh_from_split_upstream_section = AKIVehStateGetNbVehiclesSection(int(upstream_split_section_id), True) #Split upstream section of the section which is connected to signal
                    numof_lanes_for_split_upstream_signal_section = AKIInfNetGetSectionANGInf(upstream_split_section_id).nbCentralLanes + AKIInfNetGetSectionANGInf(upstream_split_section_id).nbSideLanes
                    lane_vehicle_count_from_split_upstream_section = {}
                    number_lane_from_split_upstream_section = 0
                    if num_veh_from_split_upstream_section != 0:
                        for from_split_upstream_section_veh in range(num_veh_from_split_upstream_section):
                            # Get vehicle information
                            vehicle_info_upstream = AKIVehStateGetVehicleInfSection(upstream_split_section_id, from_split_upstream_section_veh)
                            # Extract numberLane
                            number_lane_from_split_upstream_section = vehicle_info_upstream.numberLane
                            # Count the number of vehicles in each lane
                            if number_lane_from_split_upstream_section in lane_vehicle_count_from_split_upstream_section:
                                lane_vehicle_count_from_split_upstream_section[number_lane_from_split_upstream_section] += 1
                            else:
                                lane_vehicle_count_from_split_upstream_section[number_lane_from_split_upstream_section] = 1
                        for lane in range(1, numof_lanes_for_split_upstream_signal_section+1):
                            if lane not in lane_vehicle_count_from_split_upstream_section:
                                lane_vehicle_count_from_split_upstream_section[lane] = 0
                    else:
                        lane_vehicle_count_from_split_upstream_section = {0: 0}
                    AKIPrintString(f"Lane Vehicle Count from Split Upstream Section Dictionary: {lane_vehicle_count_from_split_upstream_section}")

                ## COUNT NUMBER OF VEHICLES IN JUST UPSTREAM LANES OF THE SIGNAL
                num_veh_from_section = AKIVehStateGetNbVehiclesSection(fromSection.value(), True) #Section which is connected to the signal
                lane_vehicle_count_from_section = {}
                number_lane_from_section = 0
                if num_veh_from_section != 0:
                    for from_section_veh in range(num_veh_from_section):
                        # Get vehicle information
                        vehicle_info = AKIVehStateGetVehicleInfSection(fromSection.value(), from_section_veh)
                        # Extract numberLane
                        number_lane_from_section = vehicle_info.numberLane
                        # Count the number of vehicles in each lane
                        if number_lane_from_section in lane_vehicle_count_from_section:
                            lane_vehicle_count_from_section[number_lane_from_section] += 1
                        else:
                            lane_vehicle_count_from_section[number_lane_from_section] = 1
                else:
                    lane_vehicle_count_from_section = {0: 0}
                AKIPrintString(f"Lane Vehicle Count from Signal Upstream Section Dictionary: {lane_vehicle_count_from_section}")

                ## COUNT NUMBER OF VEHICLES IN JUST DOWNSTREAM LANES OF THE SIGNAL
                num_veh_to_section = AKIVehStateGetNbVehiclesSection(toSection.value(), True) #Section which is connected to the signal in downstream
                lane_vehicle_count_to_section = {}
                number_lane_to_section = 0
                if num_veh_to_section != 0:
                    for to_section_veh in range(num_veh_to_section):
                        # Get vehicle information
                        vehicle_info = AKIVehStateGetVehicleInfSection(toSection.value(), to_section_veh)
                        # Extract numberLane
                        number_lane_to_section = vehicle_info.numberLane
                        # Count the number of vehicles in each lane
                        if number_lane_to_section in lane_vehicle_count_to_section:
                            lane_vehicle_count_to_section[number_lane_to_section] += 1
                        else:
                            lane_vehicle_count_to_section[number_lane_to_section] = 1
                else:
                    lane_vehicle_count_to_section = {0: 0}
                AKIPrintString(f"Lane Vehicle Count from Signal Downstream Section Dictionary: {lane_vehicle_count_to_section}")

                ## COUNT NUMBER OF VEHICLES IN DOWNSTREAM OF JUST DOWNSTREAM LANES OF THE SIGNAL
                for downstream_split_section_id in section_downstream_dict[toSection.value()]:
                    num_veh_to_split_downstream_section = AKIVehStateGetNbVehiclesSection(int(downstream_split_section_id), True) #Split downstream section of the section which is connected to signal in downstream
                    numof_lanes_for_split_downstream_signal_section = AKIInfNetGetSectionANGInf(downstream_split_section_id).nbCentralLanes + AKIInfNetGetSectionANGInf(downstream_split_section_id).nbSideLanes
                    lane_vehicle_count_to_split_downstream_section = {}
                    number_lane_to_split_downstream_section = 0
                    if num_veh_to_split_downstream_section != 0:
                        for to_split_downstream_section_veh in range(num_veh_to_split_downstream_section):
                            # Get vehicle information
                            vehicle_info = AKIVehStateGetVehicleInfSection(downstream_split_section_id, to_split_downstream_section_veh)
                            # Extract numberLane
                            number_lane_to_split_downstream_section = vehicle_info.numberLane
                            # Count the number of vehicles in each lane
                            if number_lane_to_split_downstream_section in lane_vehicle_count_to_split_downstream_section:
                                lane_vehicle_count_to_split_downstream_section[number_lane_to_split_downstream_section] += 1
                            else:
                                lane_vehicle_count_to_split_downstream_section[number_lane_to_split_downstream_section] = 1
                        for lane in range(1, numof_lanes_for_split_downstream_signal_section+1):
                            if lane not in lane_vehicle_count_to_split_downstream_section:
                                lane_vehicle_count_to_split_downstream_section[lane] = 0
                    else:
                        lane_vehicle_count_to_split_downstream_section = {0: 0}
                AKIPrintString(f"Lane Vehicle Count from Split Downstream Section Dictionary: {lane_vehicle_count_to_split_downstream_section}")

                first_lane_turn_origin = AKIInfNetGetTurningOriginFromLane(fromSection.value(), toSection.value())
                last_lane_turn_origin = AKIInfNetGetTurningOriginToLane(fromSection.value(), toSection.value())
                first_lane_turn_destination = AKIInfNetGetTurningDestinationFromLane(fromSection.value(), toSection.value())
                last_lane_turn_destination = AKIInfNetGetTurningDestinationToLane(fromSection.value(), toSection.value())                        

                lane_nums_turn_origin_sec = []
                lane_nums_turn_dest_sec = []

                if first_lane_turn_origin == last_lane_turn_origin:
                    lane_nums_turn_origin_sec.append(first_lane_turn_origin)
                else:
                    lane_nums_turn_origin_sec = list(range(first_lane_turn_origin, last_lane_turn_origin+1))
                ##AKIPrintString(f'Lane numbers for this turn in origin: {lane_nums_turn_origin_sec}')

                if first_lane_turn_destination == last_lane_turn_destination:
                    lane_nums_turn_dest_sec.append(first_lane_turn_destination)
                else:
                    lane_nums_turn_dest_sec = list(range(first_lane_turn_destination, last_lane_turn_destination+1))
                ##AKIPrintString(f'Lane numbers for this turn in destination: {lane_nums_turn_dest_sec}')

                # Sum the number of vehicles for each signal group in origin and destination sections
                if lane_vehicle_count_from_section.get(0) == 0 and lane_vehicle_count_from_split_upstream_section.get(0) == 0:
                    sum_vehicles_origin = 0
                else:
                    sum_veh_origin_signal_section = sum(lane_vehicle_count_from_section.get(lane, 0) for lane in lane_nums_turn_origin_sec)
                    sum_veh_origin_split_upstream_section = sum(lane_vehicle_count_from_split_upstream_section.get(lane, 0) for lane in lane_nums_turn_origin_sec)
                    sum_vehicles_origin = sum_veh_origin_signal_section + sum_veh_origin_split_upstream_section

                if lane_vehicle_count_to_section.get(0) == 0 and lane_vehicle_count_to_split_downstream_section.get(0) == 0:
                    sum_vehicles_destination = 0
                else:
                    sum_veh_dest_signal_section = sum(lane_vehicle_count_to_section.values()) #sum(lane_vehicle_count_to_section.get(lane, 0) for lane in lane_nums_turn_dest_sec)
                    sum_veh_dest_split_downstream_section = sum(lane_vehicle_count_to_split_downstream_section.values()) #sum(lane_vehicle_count_to_split_downstream_section.get(lane, 0) for lane in lane_nums_turn_dest_sec)
                    sum_vehicles_destination = sum_veh_dest_signal_section + sum_veh_dest_split_downstream_section

                # max pressure control
                veh_num_diff = (sum_vehicles_origin - sum_vehicles_destination) #* len(lane_nums_turn_origin_sec)

                signal_group_veh_diff[signal_group] = veh_num_diff
                signal_group_name[signal_group] = sg_name
                origin_veh_num[signal_group] = sum_vehicles_origin
                origin_sec_num_lanes[signal_group] = len(lane_nums_turn_origin_sec)
                dest_sec_num_lanes[signal_group] = len(lane_nums_turn_dest_sec)
                all_signal_groups_id.append(signal_group)
                AKIPrintString(f"Signal Group {signal_group} = {sg_name} with vehicle difference = {veh_num_diff}")

    # 3) Now pick the "critical phase" per your existing logic
    remaining_cycle_steps = max_cycle_time - cycle_counter
    AKIPrintString(f"Remaining Cycle Steps: {remaining_cycle_steps}")

    if not previous_phases_list and not remaining_phases_list:
        # first time
        critical_phase = all_phases[0]
        previous_phases_list.append(critical_phase)
        remaining_phases_list = all_phases[1:]
        return critical_phase

    elif (1 <= cycle_counter < max_cycle_time and 
          remaining_cycle_steps == len(remaining_phases_list) and
          len(remaining_phases_list) > 1):
        critical_phase = remaining_phases_list[0]
        previous_phases_list.append(critical_phase)
        remaining_phases_list = remaining_phases_list[1:]
        return critical_phase

    elif (1 < cycle_counter < max_cycle_time and 
          remaining_cycle_steps > len(remaining_phases_list) and
          len(remaining_phases_list) > 1):
        # compare pressure
        pressure_current_phase = calculate_phase_pressure(
            junction_id, previous_phases_list[-1], time1, signal_group_veh_diff
        )
        pressure_next_phase = calculate_phase_pressure(
            junction_id, remaining_phases_list[0], time1, signal_group_veh_diff
        )
        if pressure_current_phase > pressure_next_phase:
            # remain on the same
            critical_phase = previous_phases_list[-1]
            return critical_phase
        else:
            critical_phase = remaining_phases_list[0]
            previous_phases_list.append(critical_phase)
            remaining_phases_list = remaining_phases_list[1:]
            return critical_phase

    else:
        # cycle_counter == max_cycle_time or maybe only 1 left
        if remaining_phases_list:
            critical_phase = remaining_phases_list[0]
        else:
            # fallback
            critical_phase = all_phases[0]
        
        previous_phases_list.clear()
        remaining_phases_list.clear()
        cycle_counter = 0
        return critical_phase