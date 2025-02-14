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

junction_id = 505

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




# from AAPI import *
# from PyANGKernel import GKSystem
# import numpy as np
# import time
# import math

# ###############################################################################
# # GLOBAL VARIABLES
# ###############################################################################
# junction_id = 505

# # --- Signal states in Aimsun ---
# GREEN_SIGNAL  = 1
# AMBER_SIGNAL  = 2
# RED_SIGNAL    = 0

# # --- Timing parameters ---
# MIN_GREEN_TIME = 15.0   # seconds of green before checking to switch
# AMBER_TIME     = 2.0
# ALL_RED_TIME   = 2.0

# # --- State machine states ---
# STATE_GREEN = "GREEN"
# STATE_AMBER = "AMBER"
# STATE_ALLRED = "ALLRED"

# # --- Global state machine variables ---
# current_phase = None       # which phase is currently green
# next_phase = None          # which phase we plan to switch to
# signal_state = STATE_GREEN # can be GREEN, AMBER, or ALLRED
# time_in_state = 0.0        # how many *simulation seconds* in current signal_state

# # If you still want a cyclical approach, keep these:
# cycle_counter = 0
# max_cycle_time = 12   # counts how many times you pick a new phase
# previous_phases_list = []
# remaining_phases_list = []

# # Dictionaries from your original code
# section_upstream_dict = {
#     552: [658],
#     789: [1116],
#     786: [1064],
#     572: [988],
#     471: [2285],
#     523: [574],
#     516: [547],
#     499: [539],
#     1405: [1483],
#     1422: [1461],
#     1409: [1408],
#     1415: [1401],
#     419: [437],
#     979: [879],
#     772: [3750],
#     1084: [626],
#     1140: [2294],
#     1107: [1120],
#     1102: [766]
# }

# section_downstream_dict = {
#     778: [769],
#     774: [557],
#     780: [772],
#     776: [766],
#     511: [509],
#     502: [1644],
#     2288: [517],
#     2291: [513],
#     1432: [1402],
#     1428: [1446],
#     1430: [1443],
#     1436: [1449],
#     2297: [1529],
#     990: [988],
#     431: [2300],
#     2303: [1143],
#     1091: [614],
#     1064: [786],
#     1092: [1137]
# }

# model = GKSystem.getSystem().getActiveModel()

# ###############################################################################
# # HELPER FUNCTIONS
# ###############################################################################
# def calculate_phase_pressure(junction_id, phase, time1, signal_group_veh_diff):
#     """Calculate the sum of (originVehicles - destinationVehicles) for the 
#     signal groups in a given phase, for max-pressure control logic."""
#     num_phase_sg = ECIGetNbSignalGroupsPhaseofJunction(junction_id, phase, time1)
#     weight_list = []
#     for sg_index in range(num_phase_sg):
#         sg_id = ECIGetSignalGroupPhaseofJunction(junction_id, phase, sg_index, time1)
#         weight_list.append(signal_group_veh_diff[sg_id])
#     AKIPrintString(f"Pressure for Phase {phase}: {sum(weight_list)}")
#     AKIPrintString(f"Pressure Breakdown: {weight_list}")
#     return sum(weight_list)

# def get_signal_group_id(junction_id, phase, time1):
#     num_phase_sg = ECIGetNbSignalGroupsPhaseofJunction(junction_id, phase, time1)
#     sg_list = []
#     for sg_index in range(num_phase_sg):
#         sg_id = ECIGetSignalGroupPhaseofJunction(junction_id, phase, sg_index, time1)
#         sg_list.append(sg_id)
#     return sg_list

# def set_phase_green(junction_id, phase, time1, timeSta, acycle):
#     """Sets the specified phase's signal groups to GREEN; everything else RED."""
#     # 1) Get all signals for the phase
#     critical_sg_list = get_signal_group_id(junction_id, phase, time1)
#     # 2) Get all signal groups
#     all_sg_ids = range(1, ECIGetNumberSignalGroups(junction_id) + 1)
    
#     ECIDisableEvents(junction_id)
    
#     # 3) Turn the critical phase signals GREEN, everything else RED
#     for sg_id in all_sg_ids:
#         if sg_id in critical_sg_list:
#             ECIChangeSignalGroupState(junction_id, sg_id, GREEN_SIGNAL, timeSta, time1, acycle)
#         else:
#             ECIChangeSignalGroupState(junction_id, sg_id, RED_SIGNAL, timeSta, time1, acycle)
#     AKIPrintString(f'Cycle Counter: {cycle_counter}')
#     AKIPrintString(f"Critical Phase: {phase}")
#     AKIPrintString(f"Previous Phase List: {previous_phases_list}")
#     AKIPrintString(f"Remaining Phase List: {remaining_phases_list}")

# def set_phase_amber(junction_id, phase, time1, timeSta, acycle):
#     """Turns the signal groups of the given phase to AMBER; everything else RED."""
#     critical_sg_list = get_signal_group_id(junction_id, phase, time1)
#     all_sg_ids = range(1, ECIGetNumberSignalGroups(junction_id) + 1)
#     AKIPrintString('AMBER SIGNAL')
    
#     ECIDisableEvents(junction_id)
    
#     for sg_id in all_sg_ids:
#         if sg_id in critical_sg_list:
#             ECIChangeSignalGroupState(junction_id, sg_id, AMBER_SIGNAL, timeSta, time1, acycle)
#         else:
#             ECIChangeSignalGroupState(junction_id, sg_id, RED_SIGNAL, timeSta, time1, acycle)

# def set_all_red(junction_id, time1, timeSta, acycle):
#     """Turn all signal groups in the junction to RED."""
#     AKIPrintString('ALL RED SIGNAL')
#     num_signal_groups = ECIGetNumberSignalGroups(junction_id)
#     ECIDisableEvents(junction_id)
#     for sg in range(1, num_signal_groups + 1):
#         ECIChangeSignalGroupState(junction_id, sg, RED_SIGNAL, timeSta, time1, acycle)

# def pick_next_phase_by_max_pressure(time1):
#     """
#     This function encapsulates the big 'max-pressure' logic:
#     - Count vehicles for each signal group
#     - Determine the 'critical phase'
#     - Return that phase
#     """
#     global cycle_counter, previous_phases_list, remaining_phases_list
    
#     # ----------------------------------------------------------
#     # Gather a list of "regular" phases (not inter-phases)
#     # ----------------------------------------------------------
#     num_phases = ECIGetNumberPhases(junction_id)
#     all_phases = []
#     for phase in range(1, num_phases + 1):
#         # If it's not an interphase
#         if ECIIsAnInterPhase(junction_id, phase, time1) == 0:
#             all_phases.append(phase)

#     cycle_counter += 1
    
#     # ----------------------------------------------------------
#     # For each signal group, compute (#Vehicles upstream) - (#Vehicles downstream)
#     # ----------------------------------------------------------
#     num_signal_groups = ECIGetNumberSignalGroups(junction_id)
#     signal_group_veh_diff = {}
    
#     for sg_id in range(1, num_signal_groups + 1):
#         # Count vehicles for that signal group
#         num_turnings = ECIGetNumberTurningsofSignalGroup(junction_id, sg_id)
        
#         total_diff = 0
#         for turning_index in range(num_turnings):
#             fromSection = intp()
#             toSection = intp()
#             report = ECIGetFromToofTurningofSignalGroup(junction_id, sg_id, turning_index, fromSection, toSection)
#             if report == 0:
#                 # Upstream vehicle counting
#                 sum_vehicles_origin = 0
#                 lane_vehicle_count_from_section = {}
                
#                 # Just the immediate "fromSection"
#                 num_veh_from_section = AKIVehStateGetNbVehiclesSection(fromSection.value(), True)
#                 if num_veh_from_section > 0:
#                     for iVeh in range(num_veh_from_section):
#                         vinfo = AKIVehStateGetVehicleInfSection(fromSection.value(), iVeh)
#                         lane_vehicle_count_from_section[vinfo.numberLane] = \
#                             lane_vehicle_count_from_section.get(vinfo.numberLane, 0) + 1
                
#                 # Also the "split" upstream sections
#                 sum_upstream = 0
#                 if fromSection.value() in section_upstream_dict:
#                     for upstream_split_id in section_upstream_dict[fromSection.value()]:
#                         nveh_split = AKIVehStateGetNbVehiclesSection(upstream_split_id, True)
#                         lane_count_split = {}
#                         if nveh_split > 0:
#                             for iVeh2 in range(nveh_split):
#                                 vinfo2 = AKIVehStateGetVehicleInfSection(upstream_split_id, iVeh2)
#                                 lane_count_split[vinfo2.numberLane] = \
#                                     lane_count_split.get(vinfo2.numberLane, 0) + 1
#                         sum_upstream += sum(lane_count_split.values())
                
#                 sum_immediate_upstream = sum(lane_vehicle_count_from_section.values())
#                 sum_vehicles_origin = sum_immediate_upstream + sum_upstream
                
#                 # Downstream vehicle counting
#                 sum_vehicles_destination = 0
#                 lane_vehicle_count_to_section = {}
                
#                 # immediate "toSection"
#                 num_veh_to_section = AKIVehStateGetNbVehiclesSection(toSection.value(), True)
#                 if num_veh_to_section > 0:
#                     for iVeh in range(num_veh_to_section):
#                         vinfo = AKIVehStateGetVehicleInfSection(toSection.value(), iVeh)
#                         lane_vehicle_count_to_section[vinfo.numberLane] = \
#                             lane_vehicle_count_to_section.get(vinfo.numberLane, 0) + 1

#                 # also the "split" downstream sections
#                 sum_downstream = 0
#                 if toSection.value() in section_downstream_dict:
#                     for downstream_split_id in section_downstream_dict[toSection.value()]:
#                         nveh_split = AKIVehStateGetNbVehiclesSection(downstream_split_id, True)
#                         lane_count_split2 = {}
#                         if nveh_split > 0:
#                             for iVeh3 in range(nveh_split):
#                                 vinfo3 = AKIVehStateGetVehicleInfSection(downstream_split_id, iVeh3)
#                                 lane_count_split2[vinfo3.numberLane] = \
#                                     lane_count_split2.get(vinfo3.numberLane, 0) + 1
#                         sum_downstream += sum(lane_count_split2.values())
                
#                 sum_immediate_downstream = sum(lane_vehicle_count_to_section.values())
#                 sum_vehicles_destination = sum_immediate_downstream + sum_downstream

#                 first_lane_turn_origin = AKIInfNetGetTurningOriginFromLane(fromSection.value(), toSection.value())
#                 last_lane_turn_origin = AKIInfNetGetTurningOriginToLane(fromSection.value(), toSection.value())                       

#                 lane_nums_turn_origin_sec = []

#                 if first_lane_turn_origin == last_lane_turn_origin:
#                     lane_nums_turn_origin_sec.append(first_lane_turn_origin)
#                 else:
#                     lane_nums_turn_origin_sec = list(range(first_lane_turn_origin, last_lane_turn_origin+1))
#                 #AKIPrintString(f'Lane numbers for this turn in origin: {lane_nums_turn_origin_sec}')
                
#                 diff = (sum_vehicles_origin - sum_vehicles_destination) #* len(lane_nums_turn_origin_sec)
#                 total_diff += diff
        
#         signal_group_veh_diff[sg_id] = total_diff

#     # ----------------------------------------------------------
#     # Now decide the next "critical phase" by comparing pressures
#     # The logic below re-implements your stepwise approach:
#     # ----------------------------------------------------------
    
#     # If you haven't stored any phase, pick the first from all_phases:
#     if not previous_phases_list and not remaining_phases_list:
#         critical_phase = all_phases[0]
#         previous_phases_list.append(critical_phase)
#         remaining_phases_list = all_phases[1:]
#         return critical_phase
    
#     # If you're partway through
#     remaining_cycle_steps = max_cycle_time - cycle_counter
    
#     if 1 <= cycle_counter < max_cycle_time and remaining_cycle_steps == len(remaining_phases_list) and len(remaining_phases_list) > 1:
#         # Force next phase from the front of the remaining list
#         critical_phase = remaining_phases_list[0]
#         previous_phases_list.append(critical_phase)
#         remaining_phases_list = remaining_phases_list[1:]
#         return critical_phase
    
#     elif 1 < cycle_counter < max_cycle_time and remaining_cycle_steps > len(remaining_phases_list) and len(remaining_phases_list) > 1:
#         # Compare current vs next in line
#         pressure_current = calculate_phase_pressure(junction_id, previous_phases_list[-1], time1, signal_group_veh_diff)
#         pressure_next    = calculate_phase_pressure(junction_id, remaining_phases_list[0], time1, signal_group_veh_diff)
#         if pressure_current > pressure_next:
#             # Stay with the same phase
#             critical_phase = previous_phases_list[-1]
#             return critical_phase
#         else:
#             critical_phase = remaining_phases_list[0]
#             previous_phases_list.append(critical_phase)
#             remaining_phases_list = remaining_phases_list[1:]
#             return critical_phase
    
#     else:
#         # cycle_counter == max_cycle_time or only 1 left
#         if remaining_phases_list:
#             critical_phase = remaining_phases_list[0]
#         else:
#             # fallback
#             critical_phase = all_phases[0]
        
#         previous_phases_list.clear()
#         remaining_phases_list.clear()
#         cycle_counter = 0
#         return critical_phase

# ###############################################################################
# # AIMSUN REQUIRED FUNCTIONS
# ###############################################################################
# def AAPILoad():
#     return 0

# def AAPIInit():
#     return 0

# def AAPISimulationReady():
#     return 0

# def AAPIManage(time1, timeSta, timeTrans, acycle):
#     return 0

# def AAPIPostManage(time1, timeSta, timeTrans, acycle):
#     """
#     This is where the 'state machine' logic happens on each simulation step.
#     We track how long we have been in the current state, then decide how to
#     transition.
#     """
#     global signal_state, time_in_state
#     global current_phase, next_phase
    
#     # "acycle" is the simulation step size in seconds (e.g. 1.0 sec or 0.5 sec)
#     time_in_state += acycle

#     # ----------------------------------------------------
#     # 1) If we are in GREEN state
#     # ----------------------------------------------------
#     if signal_state == STATE_GREEN:
#         # If we have no current_phase yet, pick an initial one
#         if current_phase is None:
#             current_phase = pick_next_phase_by_max_pressure(time1)
#             # Immediately set that phase to GREEN
#             set_phase_green(junction_id, current_phase, time1, timeSta, acycle)
#             time_in_state = 0
#             return 0
        
#         # If we've been in green for at least MIN_GREEN_TIME, 
#         # check if we want to switch
#         if time_in_state >= MIN_GREEN_TIME:
#             # Run your max-pressure logic: pick next candidate phase
#             candidate_phase = pick_next_phase_by_max_pressure(time1)
            
#             if candidate_phase == current_phase:
#                 # No phase change → we remain in green, 
#                 # just reset the timer so we can keep extending green 
#                 time_in_state = 0
#             else:
#                 # We are switching phases → go to AMBER
#                 next_phase = candidate_phase
#                 set_phase_amber(junction_id, current_phase, time1, timeSta, acycle)
#                 signal_state = STATE_AMBER
#                 time_in_state = 0

#     # ----------------------------------------------------
#     # 2) If we are in AMBER state
#     # ----------------------------------------------------
#     elif signal_state == STATE_AMBER:
#         # Once we've been in amber long enough, go to ALL-RED
#         if time_in_state >= AMBER_TIME:
#             set_all_red(junction_id, time1, timeSta, acycle)
#             signal_state = STATE_ALLRED
#             time_in_state = 0

#     # ----------------------------------------------------
#     # 3) If we are in ALLRED state
#     # ----------------------------------------------------
#     elif signal_state == STATE_ALLRED:
#         # Once we've been in all-red long enough, finalize the new phase
#         if time_in_state >= ALL_RED_TIME:
#             # Switch to next_phase's GREEN
#             current_phase = next_phase
#             set_phase_green(junction_id, current_phase, time1, timeSta, acycle)
#             signal_state = STATE_GREEN
#             time_in_state = 0

#     return 0

# def AAPIFinish():
#     return 0

# def AAPIUnLoad():
#     return 0

# def AAPIPreRouteChoiceCalculation(time1, timeSta):
#     return 0



# from AAPI import *
# from PyANGKernel import GKSystem
# from itertools import combinations
# import numpy as np
# import time
# import math

# step_counter = 0  # Step counter for signal control decision
# time_step = 15
# fixed_time_step = 15
# amber_time = 2
# all_red_time = 2
# green_signal = 1 # Green signal code in Aimsun API
# red_signal = 0 # Red signal code in Aimsun API
# amber_signal = 2 # Amber signal code in Aimsun API
# junction_id = 505
# max_cycle_time = 12   #In terms of time steps
# cycle_counter = 0
# min_green = 20
# signal_group_pool = []
# previous_phases_list = []
# remaining_phases_list = []
# PHASE_CHANGED = True
# section_upstream_dict = {
#     552: [658],   #509, 647,
#     789: [1116],
#     786: [1064],
#     572: [988],     #UP TO THIS CENTER INTERSECTION
#     471: [2285],
#     523: [574],
#     516: [547],
#     499: [539],     #UP TO THIS SOUTHSIDE INTERSECTION
#     1405: [1483],
#     1422: [1461],
#     1409: [1408],
#     1415: [1401],   #UP TO THIS NORTHSIDE INTERSECTION
#     419: [437],
#     979: [879],
#     772: [3750],     #UP TO THIS EASTSIDE INTERSECTION
#     1084: [626],
#     1140: [2294],
#     1107: [1120],
#     1102: [766]     #uP TO THIS WESTSIDE INTERSECTION
# }
# section_downstream_dict = {
#     778: [769],
#     774: [557],
#     780: [772],
#     776: [766],     #UP TO THIS CENTER INTERSECTION
#     511: [509],
#     502: [1644],
#     2288: [517],
#     2291: [513],    #UP TO THIS SOUTHSIDE INTERSECTION
#     1432: [1402],
#     1428: [1446],
#     1430: [1443],
#     1436: [1449],   #UP TO NORTHSIDE INTERSECTION
#     2297: [1529],
#     990: [988],
#     431: [2300],    #UP TO EASTSIDE INTERSECTION
#     2303: [1143],
#     1091: [614],
#     1064: [786],
#     1092: [1137]    #UP TO WESTSIDE INTERSECTION
# }
# # Calling active model using scripting and later on it will be used to get the lane length
# model = GKSystem.getSystem().getActiveModel()

# def calculate_phase_pressure(junction_id, phase, time1, signal_group_veh_diff):
#     num_phase_sg = ECIGetNbSignalGroupsPhaseofJunction(junction_id, phase, time1)
#     weight_list = []
#     for sg_index in range(num_phase_sg):
#         signal_group_id = ECIGetSignalGroupPhaseofJunction(junction_id, phase, sg_index, time1)
#         AKIPrintString(f"SIGNAL GROUP POOL ID= {signal_group_id}")
#         weight_list.append(signal_group_veh_diff[signal_group_id])
#         AKIPrintString(f"WEIGHT FOR SIGNAL GROUP {signal_group_id} = {weight_list}")
#     pressure = sum(weight_list)
#     return pressure

# def get_signal_group_id(junction_id, critical_phase, time1):
#     num_phase_sg = ECIGetNbSignalGroupsPhaseofJunction(junction_id, critical_phase, time1)
#     signal_group_pool = []
#     for sg_index in range(num_phase_sg):
#         signal_group_id = ECIGetSignalGroupPhaseofJunction(junction_id, critical_phase, sg_index, time1)
#         signal_group_pool.append(signal_group_id)
#     return signal_group_pool

# def AAPILoad():
#     #AKIPrintString("AAPILoad")
#     return 0

# def AAPIInit():
#     #AKIPrintString("AAPIInit")
#     return 0

# def AAPISimulationReady():
#     #AKIPrintString("AAPISimulationReady")
#     return 0

# def AAPIManage(time1, timeSta, timeTrans, acycle):
#     #AKIPrintString("AAPIManage")
#     return 0

# def AAPIPostManage(time1, timeSta, timeTrans, acycle):
#     ECIDisableEvents(junction_id)
#     start = time.time()
#     global step_counter
#     global time_step
#     global signal_group_pool
#     global cycle_counter
#     global previous_phases_list
#     global remaining_phases_list
#     global PHASE_CHANGED

#     # Increment the step counter
#     step_counter += 1
#     AKIPrintString(f"Step Counter: {step_counter}")

#     num_phases = ECIGetNumberPhases(junction_id)
#     all_phases = []
#     for phase in range(1, num_phases + 1):
#         if ECIIsAnInterPhase(junction_id, phase, time1) == 0:
#             all_phases.append(phase)

#     if PHASE_CHANGED and step_counter % ((time_step + amber_time)/ acycle) == 0 and step_counter != 0: #AMBER TIME
#         AKIPrintString("AMBER TIME")
#         num_signal_groups = ECIGetNumberSignalGroups(junction_id)
#         green_groups = []

#         for i in range(1, num_signal_groups + 1):
#             state_sg = ECIGetCurrentStateofSignalGroup(junction_id, i)
#             if state_sg == 1:
#                 green_groups.append(i)

#             #ECIDisableEvents(junction_id)
#             #ECIIsEventsEnabled(junction_id)
            
#             for group in green_groups:
#                 ECIChangeSignalGroupState(junction_id, group, amber_signal, timeSta, time1, acycle)

#     elif PHASE_CHANGED and step_counter % ((time_step + amber_time + all_red_time)/ acycle) == 0 and time1!= 0 and step_counter != 0: #ALL RED TIME
#         AKIPrintString("ALL RED TIME")
#         #ECIDisableEvents(junction_id)
#         num_signal_groups = ECIGetNumberSignalGroups(junction_id)
#         for signal_group in range (1, num_signal_groups + 1):
#             ECIIsEventsEnabled(junction_id)
#             ECIChangeSignalGroupState(junction_id, signal_group, red_signal, timeSta, time1, acycle)
#         step_counter = -1
    
#     # Check if the time interval for signal control decision is reached (say every 15 seconds)
#     elif time1 == 1 or step_counter % (time_step / acycle) == 0:
#         AKIPrintString("SIGNAL CONTROL DECISION")
#         cycle_counter += 1
#         AKIPrintString(f"JUNCTION ID = {junction_id}")
#         # Iterate over each junction to get its ID
#         num_signal_groups = ECIGetNumberSignalGroups(junction_id)
#         signal_group_veh_diff = {}
#         signal_group_name = {}
#         origin_veh_num = {}
#         origin_sec_num_lanes = {}
#         dest_sec_num_lanes = {}
#         all_signal_groups_id = []

#         # Iterate over each signal group to get its turning movements
#         for signal_group in range(1, num_signal_groups + 1):
#             AKIPrintString(f"Signal group number = {signal_group}")
#             # Read the number of turnings for the signal group
#             num_turnings = ECIGetNumberTurningsofSignalGroup(junction_id, signal_group)

#             nonChar = boolp()
#             sg_name = AKIConvertToAsciiString(ECIGetLogicalNameofSignalGroup(junction_id, signal_group), True, nonChar)

#             for turning_index in range(num_turnings):
#                 fromSection = intp()
#                 toSection = intp()
#                 report = ECIGetFromToofTurningofSignalGroup(junction_id, signal_group, turning_index, fromSection, toSection)
#                 if report == 0:
#                     ## COUNT NUMBER OF VEHICLES IN UPSTREAM OF UPSTREAM LANES OF THE SIGNAL
#                     for upstream_split_section_id in section_upstream_dict[fromSection.value()]:
#                         num_veh_from_split_upstream_section = AKIVehStateGetNbVehiclesSection(int(upstream_split_section_id), True) #Split upstream section of the section which is connected to signal
#                         numof_lanes_for_split_upstream_signal_section = AKIInfNetGetSectionANGInf(upstream_split_section_id).nbCentralLanes + AKIInfNetGetSectionANGInf(upstream_split_section_id).nbSideLanes
#                         lane_vehicle_count_from_split_upstream_section = {}
#                         number_lane_from_split_upstream_section = 0
#                         if num_veh_from_split_upstream_section != 0:
#                             for from_split_upstream_section_veh in range(num_veh_from_split_upstream_section):
#                                 # Get vehicle information
#                                 vehicle_info_upstream = AKIVehStateGetVehicleInfSection(upstream_split_section_id, from_split_upstream_section_veh)
#                                 # Extract numberLane
#                                 number_lane_from_split_upstream_section = vehicle_info_upstream.numberLane
#                                 # Count the number of vehicles in each lane
#                                 if number_lane_from_split_upstream_section in lane_vehicle_count_from_split_upstream_section:
#                                     lane_vehicle_count_from_split_upstream_section[number_lane_from_split_upstream_section] += 1
#                                 else:
#                                     lane_vehicle_count_from_split_upstream_section[number_lane_from_split_upstream_section] = 1
#                             for lane in range(1, numof_lanes_for_split_upstream_signal_section+1):
#                                 if lane not in lane_vehicle_count_from_split_upstream_section:
#                                     lane_vehicle_count_from_split_upstream_section[lane] = 0
#                         else:
#                             lane_vehicle_count_from_split_upstream_section = {0: 0}
#                         AKIPrintString(f"Lane Vehicle Count from Split Upstream Section Dictionary: {lane_vehicle_count_from_split_upstream_section}")

#                     ## COUNT NUMBER OF VEHICLES IN JUST UPSTREAM LANES OF THE SIGNAL
#                     num_veh_from_section = AKIVehStateGetNbVehiclesSection(fromSection.value(), True) #Section which is connected to the signal
#                     lane_vehicle_count_from_section = {}
#                     number_lane_from_section = 0
#                     if num_veh_from_section != 0:
#                         for from_section_veh in range(num_veh_from_section):
#                             # Get vehicle information
#                             vehicle_info = AKIVehStateGetVehicleInfSection(fromSection.value(), from_section_veh)
#                             # Extract numberLane
#                             number_lane_from_section = vehicle_info.numberLane
#                             # Count the number of vehicles in each lane
#                             if number_lane_from_section in lane_vehicle_count_from_section:
#                                 lane_vehicle_count_from_section[number_lane_from_section] += 1
#                             else:
#                                 lane_vehicle_count_from_section[number_lane_from_section] = 1
#                     else:
#                         lane_vehicle_count_from_section = {0: 0}
#                     AKIPrintString(f"Lane Vehicle Count from Signal Upstream Section Dictionary: {lane_vehicle_count_from_section}")

#                     ## COUNT NUMBER OF VEHICLES IN JUST DOWNSTREAM LANES OF THE SIGNAL
#                     num_veh_to_section = AKIVehStateGetNbVehiclesSection(toSection.value(), True) #Section which is connected to the signal in downstream
#                     lane_vehicle_count_to_section = {}
#                     number_lane_to_section = 0
#                     if num_veh_to_section != 0:
#                         for to_section_veh in range(num_veh_to_section):
#                             # Get vehicle information
#                             vehicle_info = AKIVehStateGetVehicleInfSection(toSection.value(), to_section_veh)
#                             # Extract numberLane
#                             number_lane_to_section = vehicle_info.numberLane
#                             # Count the number of vehicles in each lane
#                             if number_lane_to_section in lane_vehicle_count_to_section:
#                                 lane_vehicle_count_to_section[number_lane_to_section] += 1
#                             else:
#                                 lane_vehicle_count_to_section[number_lane_to_section] = 1
#                     else:
#                         lane_vehicle_count_to_section = {0: 0}
#                     AKIPrintString(f"Lane Vehicle Count from Signal Downstream Section Dictionary: {lane_vehicle_count_to_section}")

#                     ## COUNT NUMBER OF VEHICLES IN DOWNSTREAM OF JUST DOWNSTREAM LANES OF THE SIGNAL
#                     for downstream_split_section_id in section_downstream_dict[toSection.value()]:
#                         num_veh_to_split_downstream_section = AKIVehStateGetNbVehiclesSection(int(downstream_split_section_id), True) #Split downstream section of the section which is connected to signal in downstream
#                         numof_lanes_for_split_downstream_signal_section = AKIInfNetGetSectionANGInf(downstream_split_section_id).nbCentralLanes + AKIInfNetGetSectionANGInf(downstream_split_section_id).nbSideLanes
#                         lane_vehicle_count_to_split_downstream_section = {}
#                         number_lane_to_split_downstream_section = 0
#                         if num_veh_to_split_downstream_section != 0:
#                             for to_split_downstream_section_veh in range(num_veh_to_split_downstream_section):
#                                 # Get vehicle information
#                                 vehicle_info = AKIVehStateGetVehicleInfSection(downstream_split_section_id, to_split_downstream_section_veh)
#                                 # Extract numberLane
#                                 number_lane_to_split_downstream_section = vehicle_info.numberLane
#                                 # Count the number of vehicles in each lane
#                                 if number_lane_to_split_downstream_section in lane_vehicle_count_to_split_downstream_section:
#                                     lane_vehicle_count_to_split_downstream_section[number_lane_to_split_downstream_section] += 1
#                                 else:
#                                     lane_vehicle_count_to_split_downstream_section[number_lane_to_split_downstream_section] = 1
#                             for lane in range(1, numof_lanes_for_split_downstream_signal_section+1):
#                                 if lane not in lane_vehicle_count_to_split_downstream_section:
#                                     lane_vehicle_count_to_split_downstream_section[lane] = 0
#                         else:
#                             lane_vehicle_count_to_split_downstream_section = {0: 0}
#                     AKIPrintString(f"Lane Vehicle Count from Split Downstream Section Dictionary: {lane_vehicle_count_to_split_downstream_section}")

#                     first_lane_turn_origin = AKIInfNetGetTurningOriginFromLane(fromSection.value(), toSection.value())
#                     last_lane_turn_origin = AKIInfNetGetTurningOriginToLane(fromSection.value(), toSection.value())
#                     first_lane_turn_destination = AKIInfNetGetTurningDestinationFromLane(fromSection.value(), toSection.value())
#                     last_lane_turn_destination = AKIInfNetGetTurningDestinationToLane(fromSection.value(), toSection.value())                        

#                     lane_nums_turn_origin_sec = []
#                     lane_nums_turn_dest_sec = []

#                     if first_lane_turn_origin == last_lane_turn_origin:
#                         lane_nums_turn_origin_sec.append(first_lane_turn_origin)
#                     else:
#                         lane_nums_turn_origin_sec = list(range(first_lane_turn_origin, last_lane_turn_origin+1))
#                     #AKIPrintString(f'Lane numbers for this turn in origin: {lane_nums_turn_origin_sec}')

#                     if first_lane_turn_destination == last_lane_turn_destination:
#                         lane_nums_turn_dest_sec.append(first_lane_turn_destination)
#                     else:
#                         lane_nums_turn_dest_sec = list(range(first_lane_turn_destination, last_lane_turn_destination+1))
#                     #AKIPrintString(f'Lane numbers for this turn in destination: {lane_nums_turn_dest_sec}')

#                     # Sum the number of vehicles for each signal group in origin and destination sections
#                     if lane_vehicle_count_from_section.get(0) == 0 and lane_vehicle_count_from_split_upstream_section.get(0) == 0:
#                         sum_vehicles_origin = 0
#                     else:
#                         sum_veh_origin_signal_section = sum(lane_vehicle_count_from_section.get(lane, 0) for lane in lane_nums_turn_origin_sec)
#                         sum_veh_origin_split_upstream_section = sum(lane_vehicle_count_from_split_upstream_section.get(lane, 0) for lane in lane_nums_turn_origin_sec)
#                         sum_vehicles_origin = sum_veh_origin_signal_section + sum_veh_origin_split_upstream_section

#                     if lane_vehicle_count_to_section.get(0) == 0 and lane_vehicle_count_to_split_downstream_section.get(0) == 0:
#                         sum_vehicles_destination = 0
#                     else:
#                         sum_veh_dest_signal_section = sum(lane_vehicle_count_to_section.get(lane, 0) for lane in lane_nums_turn_dest_sec)
#                         sum_veh_dest_split_downstream_section = sum(lane_vehicle_count_to_split_downstream_section.get(lane, 0) for lane in lane_nums_turn_dest_sec)
#                         sum_vehicles_destination = sum_veh_dest_signal_section + sum_veh_dest_split_downstream_section

#                     AKIPrintString(f'Sum of vehicles for signal group {signal_group} in origin: {sum_vehicles_origin}')
#                     AKIPrintString(f'Sum of vehicles for signal group {signal_group} in destination: {sum_vehicles_destination}')

#                     # max pressure control
#                     veh_num_diff = (sum_vehicles_origin - sum_vehicles_destination) ##* len(lane_nums_turn_origin_sec)

#                     signal_group_veh_diff[signal_group] = veh_num_diff
#                     signal_group_name[signal_group] = sg_name
#                     origin_veh_num[signal_group] = sum_vehicles_origin
#                     origin_sec_num_lanes[signal_group] = len(lane_nums_turn_origin_sec)
#                     dest_sec_num_lanes[signal_group] = len(lane_nums_turn_dest_sec)
#                     all_signal_groups_id.append(signal_group)
        
#         AKIPrintString(f"Cycle Counter: {cycle_counter}")
#         remaining_cycle_steps = max_cycle_time - cycle_counter
#         AKIPrintString(f"Remaining Cycle Steps: {remaining_cycle_steps}")

#         if cycle_counter == 1:
#             critical_phase = all_phases[0]
#             previous_phases_list.append(critical_phase)
#             remaining_phases_list = all_phases[1:]
#         elif 1 < cycle_counter < max_cycle_time and remaining_cycle_steps == len(remaining_phases_list) and len(remaining_phases_list) != 1:
#             critical_phase = remaining_phases_list[0]
#             previous_phases_list.append(critical_phase)
#             remaining_phases_list = remaining_phases_list[1:]
#         elif 1 < cycle_counter < max_cycle_time and remaining_cycle_steps > len(remaining_phases_list) and len(remaining_phases_list) != 1:
#             # Use the helper function to calculate pressures
#             pressure_current_phase = calculate_phase_pressure(junction_id, previous_phases_list[-1], time1, signal_group_veh_diff)
#             pressure_next_phase = calculate_phase_pressure(junction_id, remaining_phases_list[0], time1, signal_group_veh_diff)
#             if pressure_current_phase > pressure_next_phase:
#                 critical_phase = previous_phases_list[-1]
#                 previous_phases_list.append(critical_phase)
#                 remaining_phases_list = remaining_phases_list #No change in remaining phases as the previous phase is selected as critical
#             else:
#                 critical_phase = remaining_phases_list[0]
#                 previous_phases_list.append(critical_phase)
#                 remaining_phases_list = remaining_phases_list[1:] #Remove the selected phase from the remaining phases
#         elif cycle_counter == max_cycle_time or len(remaining_phases_list) == 1:
#             critical_phase = remaining_phases_list[0]
#             previous_phases_list.clear()
#             remaining_phases_list.clear()
#             cycle_counter = 0

#         AKIPrintString(f"Critical Phase: {critical_phase}")
#         AKIPrintString(f"Previous Phase List: {previous_phases_list}")
#         AKIPrintString(f"Remaining Phase List: {remaining_phases_list}")

#         if len(previous_phases_list) > 1 and previous_phases_list[-1] == previous_phases_list[-2]:
#             PHASE_CHANGED = False
#             AKIPrintString(f"Phase not changed")
#         elif len(previous_phases_list) ==1:
#             PHASE_CHANGED = True
#             AKIPrintString(f"Starting phase")
#         else:
#             PHASE_CHANGED = True
#             AKIPrintString(f"Phase changed")

#         if PHASE_CHANGED:
#             time_step = fixed_time_step
#             # Use the helper function to get the signal group IDs for the critical phase
#             critical_sg_list = get_signal_group_id(junction_id, critical_phase, time1)
#             red_sg = [x for x in all_signal_groups_id if x not in critical_sg_list]

#             #Change the signal state for current phase's signal groups to Green
#             for current_phase_sg in (critical_sg_list):
#                 ECIChangeSignalGroupState(junction_id, current_phase_sg, green_signal, timeSta, time1, acycle)
                
#             #Change the signal state for all other signal groups to Red
#             for red in red_sg:
#                 ECIChangeSignalGroupState(junction_id, red, red_signal, timeSta, time1, acycle)

#             step_counter = time_step
#             time_step = 2 * fixed_time_step
#         else:
#             step_counter = 0
#             time_step = fixed_time_step
#         AKIPrintString( "____________________________________________________________________________________________________________________________________________________" )
#     return 0

# def AAPIFinish():
# 	#AKIPrintString( "AAPIFinish" )
# 	return 0

# def AAPIUnLoad():
# 	#AKIPrintString( "AAPIUnLoad" )
# 	return 0
	
# def AAPIPreRouteChoiceCalculation(time1, timeSta):
# 	#AKIPrintString( "AAPIPreRouteChoiceCalculation" )
# 	return 0