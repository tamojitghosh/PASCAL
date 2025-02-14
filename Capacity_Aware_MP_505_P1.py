from AAPI import *
from PyANGKernel import GKSystem
from itertools import combinations
import numpy as np
import time

step_counter = 0  # Step counter for signal control decision
time_step = 20
amber_time = 2
all_red_time = 2
green_signal = 1 # Green signal code in Aimsun API
red_signal = 0 # Red signal code in Aimsun API
amber_signal = 2 # Amber signal code in Aimsun API
junction_id = 505
m = 4
c_infinity = 500
space_headway_jam = 6.5
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
                    ###AKIPrintString(f'Lane numbers for this turn in origin: {lane_nums_turn_origin_sec}')

                    if first_lane_turn_destination == last_lane_turn_destination:
                        lane_nums_turn_dest_sec.append(first_lane_turn_destination)
                    else:
                        lane_nums_turn_dest_sec = list(range(first_lane_turn_destination, last_lane_turn_destination+1))
                    ###AKIPrintString(f'Lane numbers for this turn in destination: {lane_nums_turn_dest_sec}')

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

                    ##AKIPrintString(f'Sum of vehicles for signal group {signal_group} in origin: {sum_vehicles_origin}')
                    ##AKIPrintString(f'Sum of vehicles for signal group {signal_group} in destination: {sum_vehicles_destination}')

                    # First assign section information to a variable using section id
                    from_section = model.getCatalog().find(fromSection.value())
                    to_section = model.getCatalog().find(toSection.value())
                    split_upstream_section = model.getCatalog().find(upstream_split_section_id)
                    split_downstream_section = model.getCatalog().find(downstream_split_section_id)

                    # Now get the number of lanes in each section for each turn
                    lanes_from_section = from_section.getLanes()
                    lanes_to_section = to_section.getLanes()
                    lanes_from_split_upstream = split_upstream_section.getLanes()
                    lanes_from_split_downstream = split_downstream_section.getLanes()

                    from_section_lane_lengths = []
                    to_section_lane_lengths =[]
                    split_upstream_section_lane_lengths = []
                    split_downstream_section_lane_lengths = []

                    ## Now get the lane lengths for the section. First get the sidelane lengths then add central lane length from section info
                    for lane in lanes_from_section:
                        from_section_lane_lengths.append(lane.getSideLaneLength2D())
                    from_section_lane_lengths.reverse()
                    section_info = AKIInfNetGetSectionANGInf(fromSection.value())
                    central_lane_length = section_info.length
                    from_section_lane_lengths = [central_lane_length if x==0 else x for x in from_section_lane_lengths]
                    ##AKIPrintString(f"FROM SECTION LANE LENGTHS = {from_section_lane_lengths}")
                    
                    ## Now get the lane lengths for the split upstream lane
                    for lane in lanes_from_split_upstream:
                        split_upstream_section_lane_lengths.append(lane.getSideLaneLength2D())
                    split_upstream_section_lane_lengths.reverse()
                    upstream_split_section_info = AKIInfNetGetSectionANGInf(upstream_split_section_id)
                    split_section_central_lane_length = upstream_split_section_info.length
                    split_upstream_section_lane_lengths = [split_section_central_lane_length if x==0 else x for x in split_upstream_section_lane_lengths]
                    while len(split_upstream_section_lane_lengths) < len(from_section_lane_lengths):
                        split_upstream_section_lane_lengths.append(0)
                    ##AKIPrintString(f"SPLIT UPSTREAM LANE LENGTHS = {split_upstream_section_lane_lengths}")
                    sum_origin_section_lengths = [from_section_lane_lengths[i] + split_upstream_section_lane_lengths[i] for i in range(len(from_section_lane_lengths))]

                    ## Get the lane lengths for downstream section
                    for lane in lanes_to_section:
                        to_section_lane_lengths.append(lane.getSideLaneLength2D())
                    to_section_lane_lengths.reverse()
                    to_section_info = AKIInfNetGetSectionANGInf(toSection.value())
                    to_section_central_lane_length = to_section_info.length
                    to_section_lane_lengths = [to_section_central_lane_length if x==0 else x for x in to_section_lane_lengths]
                    ##AKIPrintString(f"TO SECTION LANE LENGTHS = {to_section_lane_lengths}")

                    ## Get the lane lengths for split downstream section
                    for lane in lanes_from_split_downstream:
                        split_downstream_section_lane_lengths.append(lane.getSideLaneLength2D())
                    split_downstream_section_lane_lengths.reverse()
                    downstream_split_section_info = AKIInfNetGetSectionANGInf(downstream_split_section_id)
                    downstream_split_section_central_lane_length = downstream_split_section_info.length
                    split_downstream_section_lane_lengths = [downstream_split_section_central_lane_length if x==0 else x for x in split_downstream_section_lane_lengths]
                    while len(split_downstream_section_lane_lengths) < len(to_section_lane_lengths):
                        split_downstream_section_lane_lengths.append(0)
                    ##AKIPrintString(f"SPLIT DOWNSTREAM LANE LENGTHS = {split_downstream_section_lane_lengths}")
                    sum_dest_section_lengths = [to_section_lane_lengths[i] + split_downstream_section_lane_lengths[i] for i in range(len(to_section_lane_lengths))]

                    turn_lane_length_origin = sum(from_section_lane_lengths) + sum(split_upstream_section_lane_lengths) #[sum_origin_section_lengths[k - 1] for k in lane_nums_turn_origin_sec]
                    ##AKIPrintString(f"TURN LANE LENGTHS ORIGIN= {turn_lane_length_origin}")
                    origin_lane_cap = turn_lane_length_origin/space_headway_jam

                    turn_lane_length_dest = sum(to_section_lane_lengths) + sum(split_downstream_section_lane_lengths) #[sum_dest_section_lengths[k - 1] for k in lane_nums_turn_dest_sec] ## For downstream entire section length is considered as we do not know where the vehicles will go
                    ##AKIPrintString(f"TURN LANE LENGTHS DESTINATION= {turn_lane_length_dest}")
                    dest_lane_cap = turn_lane_length_dest/ space_headway_jam
                
                    # Capacity aware signal control
                    veh_num_diff = (min(1, ((sum_vehicles_origin/ c_infinity) + (2 - (origin_lane_cap/ c_infinity)) * pow((sum_vehicles_origin/ origin_lane_cap), m))/ (1 + pow((sum_vehicles_origin/ origin_lane_cap), (m-1)))) - min(1, ((sum_vehicles_destination/ c_infinity) + (2 - (dest_lane_cap/ c_infinity)) * pow((sum_vehicles_destination/ dest_lane_cap), m))/ (1 + pow((sum_vehicles_destination/ dest_lane_cap), (m-1)))))

                    #AKIPrintString(f'Weight for signal group {signal_group} = {veh_num_diff}')

                    signal_group_veh_diff[signal_group] = veh_num_diff
                    signal_group_name[signal_group] = sg_name
                    origin_veh_num[signal_group] = sum_vehicles_origin
                    origin_sec_num_lanes[signal_group] = len(lane_nums_turn_origin_sec)
                    dest_sec_num_lanes[signal_group] = len(lane_nums_turn_dest_sec)
                    all_signal_groups_id.append(signal_group)
        
        num_phases = ECIGetNumberPhases(junction_id)
        for phase in range(1, num_phases + 1):
            num_phase_signal_groups = ECIGetNbSignalGroupsPhaseofJunction(junction_id, phase, time1)
            if ECIIsAnInterPhase(junction_id, phase, time1) == 0:
                weight_list = []
                num_lane_phase = []
                for sg_index in range(num_phase_signal_groups):
                    signal_group_id = ECIGetSignalGroupPhaseofJunction(junction_id, phase, sg_index, time1)
                    weight_list.append(signal_group_veh_diff[signal_group_id])
                    num_lane_phase.append(origin_sec_num_lanes[signal_group_id])
                #total_num_lanes = sum(num_lane_phase)
                pressure_phase = sum(weight_list) #* total_num_lanes * ((1800/3600) * time_step)
                pressure_for_phase[phase] = pressure_phase
                #AKIPrintString(f"Pressure for {phase} = {pressure_phase}")

        # Find the phase with the maximum pressure
        critical_phase = max(pressure_for_phase, key=pressure_for_phase.get)
        #AKIPrintString(f'Critical phase is: {critical_phase}')

        critical_phase_sg = ECIGetNbSignalGroupsPhaseofJunction(junction_id, critical_phase, time1)

        ECIDisableEvents(junction_id)
        ECIIsEventsEnabled(junction_id)

        critical_sg_list = []
        for critical_sg in range(critical_phase_sg):
            critical_sg_id = ECIGetSignalGroupPhaseofJunction(junction_id, critical_phase, critical_sg, time1)
            ECIChangeSignalGroupState(junction_id, critical_sg_id, green_signal, timeSta, time1, acycle)
            critical_sg_list.append(critical_sg_id)
        #AKIPrintString(f"CRITICAL SIGNAL GROUPS LIST = {critical_sg_list}")
        red_sg = [sg for sg in all_signal_groups_id if sg not in critical_sg_list]
        #AKIPrintString(f"RED SIGNAL GROUPS LIST = {red_sg}")
            
        #Change the signal state for all other signal groups to Red
        for red in red_sg:
            ECIChangeSignalGroupState(junction_id, red, red_signal, timeSta, time1, acycle)
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