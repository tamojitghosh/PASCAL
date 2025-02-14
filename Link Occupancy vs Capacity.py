from AAPI import *
from PyANGKernel import GKSystem
import numpy as np
import time
import pandas as pd

step_counter = 0  # Step counter for signal control decision
time_step = 60
space_headway_jam = 6.5
j_ids = [1427, 505, 985, 1134, 1978]
junction_names = ["ns", "ss", "es", "ws", "ct"]
queue_data_list = []

# For each junction we have a list of SG IDs in the order [NB, SB, EB, WB] or [NB, EB, WB]
sg_id_nested = [
    [1,  2,  6,  5],    # for junc_id=1427 => nb=1, sb=2, eb=6, wb=5
    [1, 10,  7,  5],    # for junc_id=505  => nb=1, sb=10, eb=7, wb=5
    [1,  3,  4],        # for junc_id=985  => nb=1, eb=3, wb=4
    [1,  8,  4,  6],    # for junc_id=1134 => nb=1, sb=8,  eb=4, wb=6
    [1,  3,  5,  7]     # for junc_id=1978 => nb=1, sb=3,  eb=5, wb=7
]

# Helper to map the i-th SG ID to a direction string
def map_signal_groups_to_dir(sg_list):
    if len(sg_list) == 4:
        directions = ["nb", "sb", "eb", "wb"]
    elif len(sg_list) == 3:
        directions = ["nb", "eb", "wb"]
    else:
        # Adjust if your network has other cases
        directions = []
    return dict(zip(sg_list, directions))

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
    #AKIPrintString("AAPILoad")
    return 0

def AAPIInit():
    #AKIPrintString("AAPIInit")
    return 0

def AAPISimulationReady():
    #AKIPrintString("AAPISimulationReady")
    return 0

def AAPIManage(time1, timeSta, timeTrans, acycle):
    #AKIPrintString("AAPIManage")
    return 0

def AAPIPostManage(time1, timeSta, timeTrans, acycle):
    start = time.time()
    global step_counter
    global time_step

    # Increment the step counter
    step_counter += 1

    if timeSta in range(21600, 32402, 60):
        queue_cap_us = {}
        queue_cap_ds = {}

        for idx, j_id in enumerate(j_ids):
            # SG IDs we expect for this junction
            sgs_for_j = sg_id_nested[idx]
            
            # Map each SG ID to a direction string (nb, sb, eb, wb)
            direction_map = map_signal_groups_to_dir(sgs_for_j)

            # For each SG ID in the user-defined list
            for sg_id in sgs_for_j:
                num_turnings = ECIGetNumberTurningsofSignalGroup(j_id, sg_id)

                for turning_index in range(num_turnings):
                    fromSection = intp()
                    toSection = intp()
                    report = ECIGetFromToofTurningofSignalGroup(j_id, sg_id, turning_index, fromSection, toSection)
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
                        #AKIPrintString(f'Lane numbers for this turn in origin: {lane_nums_turn_origin_sec}')

                        if first_lane_turn_destination == last_lane_turn_destination:
                            lane_nums_turn_dest_sec.append(first_lane_turn_destination)
                        else:
                            lane_nums_turn_dest_sec = list(range(first_lane_turn_destination, last_lane_turn_destination+1))
                        #AKIPrintString(f'Lane numbers for this turn in destination: {lane_nums_turn_dest_sec}')

                        # Sum the number of vehicles for each signal group in origin and destination sections
                        if lane_vehicle_count_from_section.get(0) == 0 and lane_vehicle_count_from_split_upstream_section.get(0) == 0:
                            sum_vehicles_origin = 0
                        else:
                            sum_veh_origin_signal_section = sum(lane_vehicle_count_from_section.values()) #sum(lane_vehicle_count_from_section.get(lane, 0) for lane in lane_nums_turn_origin_sec)
                            sum_veh_origin_split_upstream_section = sum(lane_vehicle_count_from_split_upstream_section.values()) #sum(lane_vehicle_count_from_split_upstream_section.get(lane, 0) for lane in lane_nums_turn_origin_sec)
                            sum_vehicles_origin = sum_veh_origin_signal_section + sum_veh_origin_split_upstream_section

                        if lane_vehicle_count_to_section.get(0) == 0 and lane_vehicle_count_to_split_downstream_section.get(0) == 0:
                            sum_vehicles_destination = 0
                        else:
                            sum_veh_dest_signal_section = sum(lane_vehicle_count_to_section.values()) #sum(lane_vehicle_count_to_section.get(lane, 0) for lane in lane_nums_turn_dest_sec)
                            sum_veh_dest_split_downstream_section = sum(lane_vehicle_count_to_split_downstream_section.values()) #sum(lane_vehicle_count_to_split_downstream_section.get(lane, 0) for lane in lane_nums_turn_dest_sec)
                            sum_vehicles_destination = sum_veh_dest_signal_section + sum_veh_dest_split_downstream_section

                        AKIPrintString(f'Sum of vehicles for signal group {sg_id} in origin: {sum_vehicles_origin}')
                        AKIPrintString(f'Sum of vehicles for signal group {sg_id} in destination: {sum_vehicles_destination}')

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
                        AKIPrintString(f"FROM SECTION LANE LENGTHS = {from_section_lane_lengths}")
                        
                        ## Now get the lane lengths for the split upstream lane
                        for lane in lanes_from_split_upstream:
                            split_upstream_section_lane_lengths.append(lane.getSideLaneLength2D())
                        split_upstream_section_lane_lengths.reverse()
                        upstream_split_section_info = AKIInfNetGetSectionANGInf(upstream_split_section_id)
                        split_section_central_lane_length = upstream_split_section_info.length
                        split_upstream_section_lane_lengths = [split_section_central_lane_length if x==0 else x for x in split_upstream_section_lane_lengths]
                        while len(split_upstream_section_lane_lengths) < len(from_section_lane_lengths):
                            split_upstream_section_lane_lengths.append(0)
                        AKIPrintString(f"SPLIT UPSTREAM LANE LENGTHS = {split_upstream_section_lane_lengths}")

                        ## Get the lane lengths for downstream section
                        for lane in lanes_to_section:
                            to_section_lane_lengths.append(lane.getSideLaneLength2D())
                        to_section_lane_lengths.reverse()
                        to_section_info = AKIInfNetGetSectionANGInf(toSection.value())
                        to_section_central_lane_length = to_section_info.length
                        to_section_lane_lengths = [to_section_central_lane_length if x==0 else x for x in to_section_lane_lengths]
                        AKIPrintString(f"TO SECTION LANE LENGTHS = {to_section_lane_lengths}")

                        ## Get the lane lengths for split downstream section
                        for lane in lanes_from_split_downstream:
                            split_downstream_section_lane_lengths.append(lane.getSideLaneLength2D())
                        split_downstream_section_lane_lengths.reverse()
                        downstream_split_section_info = AKIInfNetGetSectionANGInf(downstream_split_section_id)
                        downstream_split_section_central_lane_length = downstream_split_section_info.length
                        split_downstream_section_lane_lengths = [downstream_split_section_central_lane_length if x==0 else x for x in split_downstream_section_lane_lengths]
                        while len(split_downstream_section_lane_lengths) < len(to_section_lane_lengths):
                            split_downstream_section_lane_lengths.append(0)
                        AKIPrintString(f"SPLIT DOWNSTREAM LANE LENGTHS = {split_downstream_section_lane_lengths}")

                        turn_lane_length_origin = sum(from_section_lane_lengths) + sum(split_upstream_section_lane_lengths) #[sum_origin_section_lengths[k - 1] for k in lane_nums_turn_origin_sec]
                        AKIPrintString(f"TURN LANE LENGTHS ORIGIN= {turn_lane_length_origin}")
                        origin_lane_cap = turn_lane_length_origin/space_headway_jam

                        turn_lane_length_dest = sum(to_section_lane_lengths) + sum(split_downstream_section_lane_lengths) #[sum_dest_section_lengths[k - 1] for k in lane_nums_turn_dest_sec] ## For downstream entire section length is considered as we do not know where the vehicles will go
                        AKIPrintString(f"TURN LANE LENGTHS DESTINATION= {turn_lane_length_dest}")
                        dest_lane_cap = turn_lane_length_dest/ space_headway_jam
                    
                        upstream_lane_occ_cap = sum_vehicles_origin/origin_lane_cap
                        downstream_lane_occ_cap = sum_vehicles_destination/dest_lane_cap

                        queue_cap_us[(j_id, sg_id)] = upstream_lane_occ_cap
                        queue_cap_ds[(j_id, sg_id)] = downstream_lane_occ_cap
        
        row_dict = {"time_int": timeSta}

        # For each junction, for each direction that *could* exist
        for idx, j_id in enumerate(j_ids):
            j_name = junction_names[idx]
            sgs_for_j = sg_id_nested[idx]
            dir_map   = map_signal_groups_to_dir(sgs_for_j)

            # dir_map is { sg_id: "nb" (or sb/eb/wb), ...}
            for sg_id, dir_str in dir_map.items():
                # Construct the column names, e.g. "ns_nb_us", "ns_nb_ds"
                col_us = f"{j_name}_{dir_str}_us"
                col_ds = f"{j_name}_{dir_str}_ds"

                # Retrieve values from the dictionary (default=0 if missing)
                up_val = queue_cap_us.get((j_id, sg_id), 0.0)
                ds_val = queue_cap_ds.get((j_id, sg_id), 0.0)

                row_dict[col_us] = up_val
                row_dict[col_ds] = ds_val
        
        queue_data_list.append(row_dict)

        # Reset step_counter so we only do this once per interval
        step_counter = 0

    return 0

def AAPIFinish():
    AKIPrintString("AAPIFinish")

    all_columns = set()
    for r in queue_data_list:
        all_columns.update(r.keys())
    all_columns = list(all_columns)

    # We want "time_int" as the first column, then everything else
    # sorted or in the order you prefer:
    if "time_int" in all_columns:
        all_columns.remove("time_int")
        all_columns = ["time_int"] + sorted(all_columns)

    df = pd.DataFrame(queue_data_list, columns=all_columns)

    output_file = r"C:\Users\N10934138\OneDrive - Queensland University of Technology\TG\Research\Thesis\Codes\Signal control\Paper 1 Codes\queue_to_capacity_data.csv"
    df.to_csv(output_file, index=False)
    return 0

def AAPIUnLoad():
	AKIPrintString( "AAPIUnLoad" )
	return 0
	
def AAPIPreRouteChoiceCalculation(time1, timeSta):
	AKIPrintString( "AAPIPreRouteChoiceCalculation" )
	return 0





















# from AAPI import *
# from PyANGKernel import GKSystem
# import numpy as np
# import time
# import pandas as pd

# step_counter = 0  # Step counter for signal control decision
# time_step = 300
# j_id = 1978
# space_headway_jam = 6.5
# queue_data_list = []

# section_upstream_dict = {
#     552: [658],
#     789: [1116],
#     786: [1064],
#     572: [988],
#     471: [3741],
#     523: [3747],
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
#     780: [3750],
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
#     990: [2536],
#     431: [2300],
#     2303: [1143],
#     1091: [614],
#     1064: [786],
#     1092: [1137]
# }
# # Calling active model using scripting and later on it will be used to get the lane length
# model = GKSystem.getSystem().getActiveModel()

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
#     start = time.time()
#     global step_counter
#     global time_step

#     # Increment the step counter
#     step_counter += 1
    
#     # Check if the time interval for signal control decision is reached (say every 15 seconds)
#     if time1 == 0 or step_counter % ((time_step) / acycle) == 0:
#         #AKIPrintString(f"JUNCTION ID = {j_id}")
#         # Iterate over each junction to get its ID
#         num_signal_groups = ECIGetNumberSignalGroups(j_id)
#         queue_cap_us = {}
#         queue_cap_ds = {}

#         # Iterate over each signal group to get its turning movements
#         for signal_group in range(1, num_signal_groups + 1):
#             AKIPrintString(f"Signal group number = {signal_group}")
#             # Read the number of turnings for the signal group
#             num_turnings = ECIGetNumberTurningsofSignalGroup(j_id, signal_group)

#             nonChar = boolp()
#             sg_name = AKIConvertToAsciiString(ECIGetLogicalNameofSignalGroup(j_id, signal_group), True, nonChar)

#             for turning_index in range(num_turnings):
#                 fromSection = intp()
#                 toSection = intp()
#                 report = ECIGetFromToofTurningofSignalGroup(j_id, signal_group, turning_index, fromSection, toSection)
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
#                         sum_veh_origin_signal_section = sum(lane_vehicle_count_from_section.values()) #sum(lane_vehicle_count_from_section.get(lane, 0) for lane in lane_nums_turn_origin_sec)
#                         sum_veh_origin_split_upstream_section = sum(lane_vehicle_count_from_split_upstream_section.values()) #sum(lane_vehicle_count_from_split_upstream_section.get(lane, 0) for lane in lane_nums_turn_origin_sec)
#                         sum_vehicles_origin = sum_veh_origin_signal_section + sum_veh_origin_split_upstream_section

#                     if lane_vehicle_count_to_section.get(0) == 0 and lane_vehicle_count_to_split_downstream_section.get(0) == 0:
#                         sum_vehicles_destination = 0
#                     else:
#                         sum_veh_dest_signal_section = sum(lane_vehicle_count_to_section.values()) #sum(lane_vehicle_count_to_section.get(lane, 0) for lane in lane_nums_turn_dest_sec)
#                         sum_veh_dest_split_downstream_section = sum(lane_vehicle_count_to_split_downstream_section.values()) #sum(lane_vehicle_count_to_split_downstream_section.get(lane, 0) for lane in lane_nums_turn_dest_sec)
#                         sum_vehicles_destination = sum_veh_dest_signal_section + sum_veh_dest_split_downstream_section

#                     AKIPrintString(f'Sum of vehicles for signal group {signal_group} in origin: {sum_vehicles_origin}')
#                     AKIPrintString(f'Sum of vehicles for signal group {signal_group} in destination: {sum_vehicles_destination}')

#                     # First assign section information to a variable using section id
#                     from_section = model.getCatalog().find(fromSection.value())
#                     to_section = model.getCatalog().find(toSection.value())
#                     split_upstream_section = model.getCatalog().find(upstream_split_section_id)
#                     split_downstream_section = model.getCatalog().find(downstream_split_section_id)

#                     # Now get the number of lanes in each section for each turn
#                     lanes_from_section = from_section.getLanes()
#                     lanes_to_section = to_section.getLanes()
#                     lanes_from_split_upstream = split_upstream_section.getLanes()
#                     lanes_from_split_downstream = split_downstream_section.getLanes()

#                     from_section_lane_lengths = []
#                     to_section_lane_lengths =[]
#                     split_upstream_section_lane_lengths = []
#                     split_downstream_section_lane_lengths = []

#                     ## Now get the lane lengths for the section. First get the sidelane lengths then add central lane length from section info
#                     for lane in lanes_from_section:
#                         from_section_lane_lengths.append(lane.getSideLaneLength2D())
#                     from_section_lane_lengths.reverse()
#                     section_info = AKIInfNetGetSectionANGInf(fromSection.value())
#                     central_lane_length = section_info.length
#                     from_section_lane_lengths = [central_lane_length if x==0 else x for x in from_section_lane_lengths]
#                     AKIPrintString(f"FROM SECTION LANE LENGTHS = {from_section_lane_lengths}")
                    
#                     ## Now get the lane lengths for the split upstream lane
#                     for lane in lanes_from_split_upstream:
#                         split_upstream_section_lane_lengths.append(lane.getSideLaneLength2D())
#                     split_upstream_section_lane_lengths.reverse()
#                     upstream_split_section_info = AKIInfNetGetSectionANGInf(upstream_split_section_id)
#                     split_section_central_lane_length = upstream_split_section_info.length
#                     split_upstream_section_lane_lengths = [split_section_central_lane_length if x==0 else x for x in split_upstream_section_lane_lengths]
#                     while len(split_upstream_section_lane_lengths) < len(from_section_lane_lengths):
#                         split_upstream_section_lane_lengths.append(0)
#                     AKIPrintString(f"SPLIT UPSTREAM LANE LENGTHS = {split_upstream_section_lane_lengths}")

#                     ## Get the lane lengths for downstream section
#                     for lane in lanes_to_section:
#                         to_section_lane_lengths.append(lane.getSideLaneLength2D())
#                     to_section_lane_lengths.reverse()
#                     to_section_info = AKIInfNetGetSectionANGInf(toSection.value())
#                     to_section_central_lane_length = to_section_info.length
#                     to_section_lane_lengths = [to_section_central_lane_length if x==0 else x for x in to_section_lane_lengths]
#                     AKIPrintString(f"TO SECTION LANE LENGTHS = {to_section_lane_lengths}")

#                     ## Get the lane lengths for split downstream section
#                     for lane in lanes_from_split_downstream:
#                         split_downstream_section_lane_lengths.append(lane.getSideLaneLength2D())
#                     split_downstream_section_lane_lengths.reverse()
#                     downstream_split_section_info = AKIInfNetGetSectionANGInf(downstream_split_section_id)
#                     downstream_split_section_central_lane_length = downstream_split_section_info.length
#                     split_downstream_section_lane_lengths = [downstream_split_section_central_lane_length if x==0 else x for x in split_downstream_section_lane_lengths]
#                     while len(split_downstream_section_lane_lengths) < len(to_section_lane_lengths):
#                         split_downstream_section_lane_lengths.append(0)
#                     AKIPrintString(f"SPLIT DOWNSTREAM LANE LENGTHS = {split_downstream_section_lane_lengths}")

#                     turn_lane_length_origin = sum(from_section_lane_lengths) + sum(split_upstream_section_lane_lengths) #[sum_origin_section_lengths[k - 1] for k in lane_nums_turn_origin_sec]
#                     AKIPrintString(f"TURN LANE LENGTHS ORIGIN= {turn_lane_length_origin}")
#                     origin_lane_cap = turn_lane_length_origin/space_headway_jam

#                     turn_lane_length_dest = sum(to_section_lane_lengths) + sum(split_downstream_section_lane_lengths) #[sum_dest_section_lengths[k - 1] for k in lane_nums_turn_dest_sec] ## For downstream entire section length is considered as we do not know where the vehicles will go
#                     AKIPrintString(f"TURN LANE LENGTHS DESTINATION= {turn_lane_length_dest}")
#                     dest_lane_cap = turn_lane_length_dest/ space_headway_jam
                
#                     upstream_lane_occ_cap = sum_vehicles_origin#/origin_lane_cap
#                     downstream_lane_occ_cap = sum_vehicles_destination#/dest_lane_cap

#                     queue_cap_us[signal_group] = upstream_lane_occ_cap
#                     queue_cap_ds[signal_group] = downstream_lane_occ_cap
#                     AKIPrintString(f"Queue capacity upstream: {queue_cap_us}")
#                     AKIPrintString(f"Queue capacity downstream: {queue_cap_ds}")
#             step_counter = 0
    
#         row_dict = {
#             "time_int": time1,
#             "nb_us": queue_cap_us.get(1, 0.0),
#             "sb_us": queue_cap_us.get(3, 0.0),
#             "eb_us": queue_cap_us.get(5, 0.0),
#             "wb_us": queue_cap_us.get(7, 0.0),
#             "nb_ds": queue_cap_ds.get(1, 0.0),
#             "sb_ds": queue_cap_ds.get(3, 0.0),
#             "eb_ds": queue_cap_ds.get(5, 0.0),
#             "wb_ds": queue_cap_ds.get(7, 0.0),
#         }
#         queue_data_list.append(row_dict)
#         AKIPrintString(f"Queue Data List: {queue_data_list}")
#         AKIPrintString( "____________________________________________________________________________________________________________________________________________________" )
#     return 0

# def AAPIFinish():
#     AKIPrintString("AAPIFinish")
#     df = pd.DataFrame(queue_data_list, columns=[
#         "time_int", "nb_us", "sb_us", "eb_us", "wb_us",
#         "nb_ds", "sb_ds", "eb_ds", "wb_ds"
#     ])

#     output_file = r"C:\Users\N10934138\OneDrive - Queensland University of Technology\TG\Research\Thesis\Codes\Signal control\Paper 1 Codes\queue_to_capacity_data.csv"
#     df.to_csv(output_file, index=False)

#     return 0

# def AAPIUnLoad():
# 	AKIPrintString( "AAPIUnLoad" )
# 	return 0
	
# def AAPIPreRouteChoiceCalculation(time1, timeSta):
# 	AKIPrintString( "AAPIPreRouteChoiceCalculation" )
# 	return 0