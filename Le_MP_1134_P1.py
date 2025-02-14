from AAPI import *
from PyANGKernel import GKSystem
from itertools import combinations
import numpy as np
import time
import math

step_counter = 0  # Step counter for signal control decision
time_step = 5
amber_time = 2
all_red_time = 2
green_signal = 1 # Green signal code in Aimsun API
red_signal = 0 # Red signal code in Aimsun API
amber_signal = 2 # Amber signal code in Aimsun API
junction_id = 1134
eta = 2.5
cycle_time = 100  #This is the total green time and does not include the all red and amber
min_green = 15
phase_pool = []
signal_group_pool = []
green_duration_phase = []
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
# Calling active model using scripting and later on it will be used to get the lane length
model = GKSystem.getSystem().getActiveModel()

def AAPILoad():
    ##AKIPrintString("AAPILoad")
    return 0

def AAPIInit():
    ##AKIPrintString("AAPIInit")
    return 0

def AAPISimulationReady():
    ##AKIPrintString("AAPISimulationReady")
    return 0

def AAPIManage(time1, timeSta, timeTrans, acycle):
    ##AKIPrintString("AAPIManage")
    return 0

def AAPIPostManage(time1, timeSta, timeTrans, acycle):
    start = time.time()
    global step_counter
    global time_step
    global phase_pool
    global signal_group_pool
    global green_duration_phase

    # Increment the step counter
    step_counter += 1

    if step_counter % (time_step/ acycle) == 0:
        num_signal_groups = ECIGetNumberSignalGroups(junction_id)
        green_groups = []

        for i in range(1, num_signal_groups + 1):
            state_sg = ECIGetCurrentStateofSignalGroup(junction_id, i)
            if state_sg == 1:
                green_groups.append(i)

            ECIDisableEvents(junction_id)
            ECIIsEventsEnabled(junction_id)
            
            for group in green_groups:
                ECIChangeSignalGroupState(junction_id, group, amber_signal, timeSta, time1, acycle)

    elif step_counter % ((time_step + amber_time)/ acycle) == 0 and time1!= 0:
        ECIDisableEvents(junction_id)
        num_signal_groups = ECIGetNumberSignalGroups(junction_id)
        for signal_group in range (1, num_signal_groups + 1):
            ECIIsEventsEnabled(junction_id)
            ECIChangeSignalGroupState(junction_id, signal_group, red_signal, timeSta, time1, acycle)
    
    # Check if the time interval for signal control decision is reached (say every 15 seconds)
    elif time1 == 0 or step_counter % ((time_step + amber_time + all_red_time) / acycle) == 0:
        if all(not sublist for sublist in phase_pool):
            #AKIPrintString(f"JUNCTION ID = {junction_id}")
            # Iterate over each junction to get its ID
            num_signal_groups = ECIGetNumberSignalGroups(junction_id)
            signal_group_veh_diff = {}
            signal_group_name = {}
            origin_veh_num = {}
            origin_sec_num_lanes = {}
            dest_sec_num_lanes = {}
            pressure_for_phase = {}
            all_signal_groups_id = []

            # Iterate over each signal group to get its turning movements
            for signal_group in range(1, num_signal_groups + 1):
                #AKIPrintString(f"Signal group number = {signal_group}")
                # Read the number of turnings for the signal group
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
                            #AKIPrintString(f"Lane Vehicle Count from Split Upstream Section Dictionary: {lane_vehicle_count_from_split_upstream_section}")

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
                        #AKIPrintString(f"Lane Vehicle Count from Signal Upstream Section Dictionary: {lane_vehicle_count_from_section}")

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
                        #AKIPrintString(f"Lane Vehicle Count from Signal Downstream Section Dictionary: {lane_vehicle_count_to_section}")

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
                        #AKIPrintString(f"Lane Vehicle Count from Split Downstream Section Dictionary: {lane_vehicle_count_to_split_downstream_section}")

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

                        #AKIPrintString(f'Sum of vehicles for signal group {signal_group} in origin: {sum_vehicles_origin}')
                        #AKIPrintString(f'Sum of vehicles for signal group {signal_group} in destination: {sum_vehicles_destination}')

                        # max pressure control
                        veh_num_diff = (sum_vehicles_origin - sum_vehicles_destination) #* len(lane_nums_turn_origin_sec)

                        signal_group_veh_diff[signal_group] = veh_num_diff
                        signal_group_name[signal_group] = sg_name
                        origin_veh_num[signal_group] = sum_vehicles_origin
                        origin_sec_num_lanes[signal_group] = len(lane_nums_turn_origin_sec)
                        dest_sec_num_lanes[signal_group] = len(lane_nums_turn_dest_sec)
                        all_signal_groups_id.append(signal_group)
            
            pressure_phase_list = []
            phase_pool = []

            num_phases = ECIGetNumberPhases(junction_id)
            for phase in range(1, num_phases + 1):
                signal_group_pool = []
                num_phase_signal_groups = ECIGetNbSignalGroupsPhaseofJunction(junction_id, phase, time1)
                if ECIIsAnInterPhase(junction_id, phase, time1) == 0:
                    weight_list = []
                    num_lane_phase = []
                    for sg_index in range(num_phase_signal_groups):
                        signal_group_id = ECIGetSignalGroupPhaseofJunction(junction_id, phase, sg_index, time1)
                        #AKIPrintString(f"SIGNAL GROUP POOL ID= {signal_group_id}")
                        weight_list.append(signal_group_veh_diff[signal_group_id])
                        #AKIPrintString(f"WEIGHT FOR SIGNAL GROUP {signal_group_id} = {weight_list}")
                        num_lane_phase.append(origin_sec_num_lanes[signal_group_id])
                        signal_group_pool.append(signal_group_id)
                        #AKIPrintString(f"SIGNAL GROUP POOL = {signal_group_pool}")
                    #total_num_lanes = sum(num_lane_phase)
                    pressure_phase = sum(weight_list) #* total_num_lanes * ((1800/3600) * time_step / 1000)
                    pressure_for_phase[phase] = pressure_phase
                    #AKIPrintString(f"Pressure for {phase} = {pressure_phase}")
                    pressure_phase_list.append(pressure_phase)
                    phase_pool.append(signal_group_pool)
                    #AKIPrintString(f"PHASE POOL = {phase_pool}")

            pressure_exp_value = [np.exp(np.clip(eta * phase_pressure, -700, 700)) for phase_pressure in pressure_phase_list]
            sum_exp_value = sum(pressure_exp_value)
            #AKIPrintString(f"SUM PRESSURE = {sum_exp_value}")

            green_duration_phase = []
            min_green_list = []
            total_min_green = min_green * len(phase_pool)
            remaining_green = (cycle_time - total_min_green)

            for phase_exp in pressure_exp_value:
                green_calc = round(phase_exp/ sum_exp_value * remaining_green)
                green_duration = green_calc
                # green_duration = max(min_green, green_calc)
                # if green_duration == min_green:
                #     min_green_list.append(min_green)
                green_duration_phase.append(green_duration)

            # total_min_green = sum(min_green_list)
            # excess_time = (cycle_time - total_min_green)
            # greens_over_min_green = [x for x in green_duration_phase if x > min_green]
            # sum_greens_over_min_green = sum(greens_over_min_green)

            # updated_greens = [(x/ sum_greens_over_min_green) * excess_time for x in greens_over_min_green] 
            
            # index = 0
            # for i in range(len(green_duration_phase)):
            #     if green_duration_phase[i] > 20:
            #         green_duration_phase[i] = updated_greens[index]
            #         index += 1

            green_duration_phase = [x+min_green for x in green_duration_phase]

            if sum(green_duration_phase) > cycle_time:
                highest_element = max(green_duration_phase)
                highest_element_index = green_duration_phase.index(highest_element)
                # Calculate the amount to reduce from the highest element
                reduction_amount = sum(green_duration_phase) - cycle_time
                # Adjust the highest element
                green_duration_phase[highest_element_index] -= reduction_amount

            #AKIPrintString(f'Green durations are: {green_duration_phase}')

            ECIDisableEvents(junction_id)
            ECIIsEventsEnabled(junction_id)

            critical_sg_list = next((sublist for sublist in phase_pool if sublist), None)
            red_sg = [sg for sg in all_signal_groups_id if sg not in critical_sg_list]

            #Change the signal state for current phase's signal groups to Green
            for current_phase_sg in (critical_sg_list):
                ECIChangeSignalGroupState(junction_id, current_phase_sg, green_signal, timeSta, time1, acycle)
                
            #Change the signal state for all other signal groups to Red
            for red in red_sg:
                ECIChangeSignalGroupState(junction_id, red, red_signal, timeSta, time1, acycle)

            time_step = green_duration_phase[0]
            phase_pool = phase_pool[1:]
            green_duration_phase = green_duration_phase[1:]
        else:
            #AKIPrintString(f"JUNCTION ID = {junction_id}")
            #AKIPrintString(f"PHASE POOL UPDATED = {phase_pool}")
            #AKIPrintString(f"GREEN DURATION LIST UPDATED= {green_duration_phase}")
            all_signal_groups_id = []
            num_signal_groups = ECIGetNumberSignalGroups(junction_id)
            for signal_group in range (1, num_signal_groups + 1):
                all_signal_groups_id.append(signal_group)
            ECIDisableEvents(junction_id)
            ECIIsEventsEnabled(junction_id)

            critical_sg_list = next((sublist for sublist in phase_pool if sublist), None)
            red_sg = [sg for sg in all_signal_groups_id if sg not in critical_sg_list]

            #Change the signal state for current phase's signal groups to Green
            for current_phase_sg in (critical_sg_list):
                ECIChangeSignalGroupState(junction_id, current_phase_sg, green_signal, timeSta, time1, acycle)
                
            #Change the signal state for all other signal groups to Red
            for red in red_sg:
                ECIChangeSignalGroupState(junction_id, red, red_signal, timeSta, time1, acycle)

            time_step = green_duration_phase[0]
            phase_pool = phase_pool[1:]
            green_duration_phase = green_duration_phase[1:]
        step_counter = 0
        #AKIPrintString( "____________________________________________________________________________________________________________________________________________________" )
    return 0

def AAPIFinish():
	##AKIPrintString( "AAPIFinish" )
	return 0

def AAPIUnLoad():
	##AKIPrintString( "AAPIUnLoad" )
	return 0
	
def AAPIPreRouteChoiceCalculation(time1, timeSta):
	##AKIPrintString( "AAPIPreRouteChoiceCalculation" )
	return 0