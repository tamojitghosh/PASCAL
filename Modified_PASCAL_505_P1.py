from AAPI import *
from PyANGKernel import GKSystem
from itertools import combinations
import numpy as np
import time
import math

##############################################################################
# GLOBAL VARIABLES FOR STATE MACHINE
##############################################################################
time_step = 1     # minimum green time before re-selecting
amber_time = 2
all_red_time = 2
space_headway_jam = 6.5 #in meters to find out how many vehicles can be accommodated in one lane
green_signal  = 1
red_signal    = 0
amber_signal  = 2

junction_id = 505
# We no longer rely on "step_counter" to do amber/red. 
# We do keep it for logs or minor checks if you wish.
step_counter = 0  

# ------------------  STATE MACHINE variables ------------------
signal_state = "GREEN"        # can be "GREEN", "AMBER", "ALLRED"
time_in_state = 0.0           # how many simulation seconds in the current state

# For the movement-based logic:
current_green_set = set()     # which signal groups are green right now
next_green_set = set()        # which signal groups we plan to turn green
movements_to_turn_off = set() # which signal groups are leaving green

##############################################################################
# OTHER GLOBALS from your original code
##############################################################################
selection_pool = []
sat_flow = 2
max_green = 60
min_green = 15

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

# Conflict matrix, plus your existing "compatible_combinations_dict" logic
conflict_matrix = {
    'NB Th': ['NB LT', 'NB RT', 'SB Th', 'SB LT'],
    'NB LT': ['NB Th', 'NB RT', 'SB Th', 'SB LT', 'EB Th', 'EB RT', 'WB RT'],
    'NB RT': ['NB Th', 'NB LT', 'SB RT'],
    'SB Th': ['NB Th', 'NB LT', 'SB LT', 'SB RT'],
    'SB LT': ['NB Th', 'NB LT', 'SB Th', 'SB RT', 'EB RT', 'WB Th', 'WB RT'],
    'SB RT': ['NB RT', 'SB Th', 'SB LT'],
    'EB Th': ['NB LT', 'EB RT', 'WB Th'],
    'EB RT': ['NB LT', 'SB LT', 'EB Th', 'WB RT'],
    'WB Th': ['SB LT', 'EB Th', 'WB RT'],
    'WB RT': ['NB LT', 'SB LT', 'EB RT', 'WB Th']
}

compatible_combinations_dict = {}  # We build below

def check_compatibility(combination, movement_list):
    for movement1, movement2 in combinations(combination, 2):
        if movement1 in movement_list[movement2] or movement2 in movement_list[movement1]:
            continue
        else:
            return False
    return True

def discard_subsets(combinations_list):
    sorted_combinations = sorted(combinations_list, key=len, reverse=True)
    final_combinations = []
    for combination in sorted_combinations:
        if all(not set(combination).issubset(set(c)) for c in final_combinations):
            final_combinations.append(combination)
    return final_combinations

for key, elements in conflict_matrix.items():
    compatible_combinations = []
    for r in range(1, len(elements) + 1):
        for combo in combinations(elements, r):
            if check_compatibility(combo, conflict_matrix):
                compatible_combinations.append(list(combo))
    compatible_combinations = discard_subsets(compatible_combinations)
    compatible_combinations_dict[key] = compatible_combinations

def find_highest_sum_combination(critical_signal_group, compatible_combinations_dict, criteria_values_signal_group_name):
    combinations_list = compatible_combinations_dict.get(critical_signal_group, [])
    highest_sum = float('-inf')
    best_combination = None
    for combo in combinations_list:
        combo_sum = sum(criteria_values_signal_group_name.get(key, 0) for key in combo)
        if combo_sum > highest_sum:
            highest_sum = combo_sum
            best_combination = combo
    return best_combination

# Get the active model if needed
model = GKSystem.getSystem().getActiveModel()

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
# THE MAIN STATE MACHINE (replaces your old step_counter-based amber/red)
##############################################################################
def AAPIPostManage(time1, timeSta, timeTrans, acycle):
    """
    - We remain in 'GREEN' until we exceed 'time_step' seconds in state.
    - Then we pick a new set of movements (via 'pick_new_green_set').
    - Movements leaving green => AMBER -> ALLRED.
    - Movements continuing => stay green, no amber or red for them.
    - Movements newly added => wait until old ones finish all-red, then go green.
    """
    global signal_state
    global time_in_state
    global step_counter

    global current_green_set
    global next_green_set
    global movements_to_turn_off

    # Increase time in the current signal state
    time_in_state += acycle
    step_counter  += 1  # optional for debugging

    # ----------------------------------------------------------------
    # 1) If we are in GREEN state
    # ----------------------------------------------------------------
    if signal_state == "GREEN":
        # If we've been green for at least 'time_step' seconds, 
        # see if we want to pick new movements
        if time_in_state >= time_step:
            next_green_set = pick_new_green_set(time1, timeSta, acycle)
            #AKIPrintString(f"Next green set: {next_green_set}")

            current_green_set.clear()
            num_signal_groups = ECIGetNumberSignalGroups(junction_id)
            for i in range(1, num_signal_groups + 1):
                state_sg = ECIGetCurrentStateofSignalGroup(junction_id, i)
                if state_sg == 1:
                    current_green_set.add(i)
            #AKIPrintString(f"Current green set: {current_green_set}")

            # Find which movements are continuing vs turning off
            continuing = current_green_set.intersection(next_green_set)
            #AKIPrintString(f"Continuing movements: {continuing}")
            movements_to_turn_off = current_green_set - continuing
            #AKIPrintString(f"Turning off movements: {movements_to_turn_off}")

            if len(movements_to_turn_off) == 0:
                # If there's no difference, we keep the same set => just reset timer
                time_in_state = 0
            else:
                # We do have some to turn off => set them to AMBER
                turn_movements_to_amber(movements_to_turn_off, timeSta, time1, acycle)
                signal_state = "AMBER"
                time_in_state = 0

    # ----------------------------------------------------------------
    # 2) If we are in AMBER state
    # ----------------------------------------------------------------
    elif signal_state == "AMBER":
        # After 'amber_time' seconds, the movements_to_turn_off go fully RED
        if time_in_state >= amber_time:
            turn_movements_to_red(movements_to_turn_off, timeSta, time1, acycle)
            signal_state = "ALLRED"
            time_in_state = 0

    # ----------------------------------------------------------------
    # 3) If we are in ALLRED state
    # ----------------------------------------------------------------
    elif signal_state == "ALLRED":
        # After 'all_red_time' seconds, finalize new green set
        if time_in_state >= all_red_time:
            finalize_new_green_set(next_green_set, timeSta, time1, acycle)
            # old movements_to_turn_off are now red, continuing remain green
            # current_green_set.clear()
            # current_green_set.update(next_green_set)  # the next set is now official
            movements_to_turn_off.clear()
            signal_state = "GREEN"
            time_in_state = 0

    return 0

def AAPIFinish():
    return 0

def AAPIUnLoad():
    return 0

def AAPIPreRouteChoiceCalculation(time1, timeSta):
    return 0

##############################################################################
# NEW: "pick_new_green_set()" => EXACTLY your old logic that picks:
#  - If selection_pool is empty => we do the big block of counting, 
#    find critical signal group, best compliment, etc.
#  - Else => we do the other block.
#  - We do *not* immediately set them green in here; we just *decide*
#    which should be green. We return that set of signal group IDs.
##############################################################################
def pick_new_green_set(time1, timeSta, acycle):
    """
    This function is basically your old code from the big "if not selection_pool"
    and "else" blocks, except we remove the forced amber/all-red steps. 
    We only decide a new set of movements to be green, and return them as a Python set.
    """
    global selection_pool
    global time_step

    # We'll create a local set that will store the new movements we want green:
    new_green = set()

    #AKIPrintString("========== PICKING A NEW GREEN SET ==========")

    if not selection_pool:
        ##AKIPrintString(f"JUNCTION ID = {junction_id}")
        signal_group_veh_diff = {}
        signal_group_name = {}
        origin_veh_num = {}
        dest_spare_cap = {}
        origin_sec_num_lanes = {}
        dest_sec_num_lanes = {}

        num_signal_groups = ECIGetNumberSignalGroups(junction_id)
        for signal_group in range(1, num_signal_groups + 1):
            ##AKIPrintString(f"Signal group number = {signal_group}")
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
                        ##AKIPrintString(f"Lane Vehicle Count from Split Upstream Section Dictionary: {lane_vehicle_count_from_split_upstream_section}")

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
                    ##AKIPrintString(f"Lane Vehicle Count from Signal Upstream Section Dictionary: {lane_vehicle_count_from_section}")

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
                    ##AKIPrintString(f"Lane Vehicle Count from Signal Downstream Section Dictionary: {lane_vehicle_count_to_section}")

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
                    ##AKIPrintString(f"Lane Vehicle Count from Split Downstream Section Dictionary: {lane_vehicle_count_to_split_downstream_section}")

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

                    turn_lane_length_origin = [sum_origin_section_lengths[k - 1] for k in lane_nums_turn_origin_sec]
                    ##AKIPrintString(f"TURN LANE LENGTHS ORIGIN= {turn_lane_length_origin}")
                    origin_lane_cap = sum(turn_lane_length_origin)/space_headway_jam

                    turn_lane_length_dest = sum(to_section_lane_lengths) + sum(split_downstream_section_lane_lengths) #sum_dest_section_lengths #[sum_dest_section_lengths[k - 1] for k in lane_nums_turn_dest_sec] ## For downstream entire section length is considered as we do not know where the vehicles will go
                    ##AKIPrintString(f"TURN LANE LENGTHS DESTINATION= {turn_lane_length_dest}")
                    dest_lane_cap = turn_lane_length_dest/ space_headway_jam

                    #PASCAL Pressure Calculation
                    veh_num_diff = (sum_vehicles_origin/ origin_lane_cap) * (1 - (sum_vehicles_destination/ dest_lane_cap)) #* len(lane_nums_turn_origin_sec)

                    signal_group_veh_diff[signal_group] = veh_num_diff
                    signal_group_name[signal_group] = sg_name
                    origin_veh_num[signal_group] = sum_vehicles_origin
                    dest_spare_cap[signal_group] = (dest_lane_cap - sum_vehicles_destination)
                    origin_sec_num_lanes[signal_group] = len(lane_nums_turn_origin_sec)
                    dest_sec_num_lanes[signal_group] = len(lane_nums_turn_dest_sec)

        # After finishing the big counting:
        criteria_values_signal_group_name = {signal_group_name.get(key, key): value
                                             for key, value in signal_group_veh_diff.items()}
        # Find the movement with max difference => critical
        critical_signal_group = max(criteria_values_signal_group_name,
                                    key=criteria_values_signal_group_name.get)
        best_compli_combi = find_highest_sum_combination(
            critical_signal_group, compatible_combinations_dict, 
            criteria_values_signal_group_name
        )
        # The "red signal groups" etc. just for logging
        all_signal_groups = list(criteria_values_signal_group_name.keys())
        red_signal_groups = [sg for sg in all_signal_groups
                             if sg != critical_signal_group and sg not in best_compli_combi]
        #AKIPrintString(f'Critical signal group is: {critical_signal_group}')
        #AKIPrintString(f'Final complimentary signal groups: {best_compli_combi}')
        #AKIPrintString(f'Red signal groups: {red_signal_groups}')

        # Convert critical + best_compli into their numeric IDs
        critical_signal_group_num = [k for k, v in signal_group_name.items()
                                     if v == critical_signal_group]
        best_compli_combi_num = [k for k, v in signal_group_name.items()
                                 if v in best_compli_combi]
        red_signal_groups_num = [k for k, v in signal_group_name.items() 
                                 if v in red_signal_groups]

        # The new green set is the union of critical_signal_group_num + best_compli_combi_num
        new_green.update(critical_signal_group_num)
        new_green.update(best_compli_combi_num)

        # current_green_set.clear()
        # num_signal_groups = ECIGetNumberSignalGroups(junction_id)
        # for i in range(1, num_signal_groups + 1):
        #     state_sg = ECIGetCurrentStateofSignalGroup(junction_id, i)
        #     if state_sg == 1:
        #         current_green_set.add(i)

        # Also, we figure out the updated time_step (like your original code).
        # This also remains the same logic, but we do not forcibly do amber/red.
        # ...
        new_ts = round(min((origin_veh_num[critical_signal_group_num[0]] * sat_flow 
                            / origin_sec_num_lanes[critical_signal_group_num[0]]), 
                           (dest_spare_cap[critical_signal_group_num[0]] * sat_flow 
                            / dest_sec_num_lanes[critical_signal_group_num[0]]), 
                           max_green))
        if new_ts < min_green:
            new_ts = min_green
        time_step = new_ts
        #AKIPrintString(f'NEW TIME STEP = {time_step}')

        # We also set selection_pool
        selection_pool = red_signal_groups_num
        #AKIPrintString(f'UPDATED SELECTION POOL = {selection_pool}')

    else:
        signal_group_veh_diff = {}
        signal_group_name = {}
        origin_veh_num = {}
        dest_spare_cap = {}
        origin_sec_num_lanes = {}
        dest_sec_num_lanes = {}

        num_signal_groups = ECIGetNumberSignalGroups(junction_id)
        for signal_group in range(1, num_signal_groups + 1):
            ##AKIPrintString(f"Signal group number = {signal_group}")
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
                        ##AKIPrintString(f"Lane Vehicle Count from Split Upstream Section Dictionary: {lane_vehicle_count_from_split_upstream_section}")

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
                    ##AKIPrintString(f"Lane Vehicle Count from Signal Upstream Section Dictionary: {lane_vehicle_count_from_section}")

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
                    ##AKIPrintString(f"Lane Vehicle Count from Signal Downstream Section Dictionary: {lane_vehicle_count_to_section}")

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
                    ##AKIPrintString(f"Lane Vehicle Count from Split Downstream Section Dictionary: {lane_vehicle_count_to_split_downstream_section}")

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

                    turn_lane_length_origin = [sum_origin_section_lengths[k - 1] for k in lane_nums_turn_origin_sec]
                    ##AKIPrintString(f"TURN LANE LENGTHS ORIGIN= {turn_lane_length_origin}")
                    origin_lane_cap = sum(turn_lane_length_origin)/space_headway_jam

                    turn_lane_length_dest = sum(to_section_lane_lengths) + sum(split_downstream_section_lane_lengths) #sum_dest_section_lengths #[sum_dest_section_lengths[k - 1] for k in lane_nums_turn_dest_sec] ## For downstream entire section length is considered as we do not know where the vehicles will go
                    ##AKIPrintString(f"TURN LANE LENGTHS DESTINATION= {turn_lane_length_dest}")
                    dest_lane_cap = turn_lane_length_dest/ space_headway_jam

                    #PASCAL Pressure Calculation
                    veh_num_diff = (sum_vehicles_origin/ origin_lane_cap) * (1 - (sum_vehicles_destination/ dest_lane_cap)) #* len(lane_nums_turn_origin_sec)

                    signal_group_veh_diff[signal_group] = veh_num_diff
                    signal_group_name[signal_group] = sg_name
                    origin_veh_num[signal_group] = sum_vehicles_origin
                    dest_spare_cap[signal_group] = (dest_lane_cap - sum_vehicles_destination)
                    origin_sec_num_lanes[signal_group] = len(lane_nums_turn_origin_sec)
                    dest_sec_num_lanes[signal_group] = len(lane_nums_turn_dest_sec)
        
        criteria_values_signal_group_name = {signal_group_name.get(key, key): value 
                                             for key, value in signal_group_veh_diff.items()}
        selection_pool_array = np.array(selection_pool)
        common_sg = np.intersect1d(list(signal_group_veh_diff.keys()), selection_pool_array)
        max_key = common_sg[np.argmax([signal_group_veh_diff[key] for key in common_sg])]
        critical_signal_group = signal_group_name[max_key]
        best_compli_combi = find_highest_sum_combination(critical_signal_group,
                                                         compatible_combinations_dict,
                                                         criteria_values_signal_group_name)
        all_signal_groups = list(criteria_values_signal_group_name.keys())
        red_signal_groups = [sg for sg in all_signal_groups 
                             if sg != critical_signal_group and sg not in best_compli_combi]

        #AKIPrintString(f'Critical signal group is: {critical_signal_group}')
        #AKIPrintString(f'Final complimentary signal groups: {best_compli_combi}')
        #AKIPrintString(f'Red signal groups: {red_signal_groups}')

        critical_signal_group_num = [k for k,v in signal_group_name.items()
                                     if v == critical_signal_group]
        best_compli_combi_num = [k for k,v in signal_group_name.items()
                                 if v in best_compli_combi]
        red_signal_groups_num = [k for k,v in signal_group_name.items()
                                 if v in red_signal_groups]

        # Union => new green
        new_green.update(critical_signal_group_num)
        new_green.update(best_compli_combi_num)

        # current_green_set.clear()
        # num_signal_groups = ECIGetNumberSignalGroups(junction_id)
        # for i in range(1, num_signal_groups + 1):
        #     state_sg = ECIGetCurrentStateofSignalGroup(junction_id, i)
        #     if state_sg == 1:
        #         current_green_set.add(i)

        # time_step logic
        new_ts = round(min((origin_veh_num[critical_signal_group_num[0]] * sat_flow 
                            / origin_sec_num_lanes[critical_signal_group_num[0]]), 
                           (dest_spare_cap[critical_signal_group_num[0]] * sat_flow 
                            / dest_sec_num_lanes[critical_signal_group_num[0]]), 
                           max_green))
        if new_ts < min_green:
            new_ts = min_green
        time_step = new_ts
        #AKIPrintString(f'NEW TIME STEP = {time_step}')

        # Update selection_pool
        # your original code:
        #   selection_pool = [mov for mov in selection_pool
        #                     if mov not in critical_signal_group_num
        #                     and mov not in best_compli_combi_num]
        # ...
        new_pool = []
        for mov in selection_pool:
            if mov not in critical_signal_group_num and mov not in best_compli_combi_num:
                new_pool.append(mov)
        selection_pool = new_pool
        #AKIPrintString(f"NEW SELECTION POOL = {selection_pool}")

    # return the final set
    return new_green

##############################################################################
# HELPER FUNCTIONS FOR THE STATE MACHINE
# (Amber, Red, Finalizing new greens)
##############################################################################
def turn_movements_to_amber(to_turn_off, timeSta, time1, acycle):
    """Sets only those movements to amber; continuing movements remain green."""
    #AKIPrintString(f"TURNING OFF => AMBER for {to_turn_off}")
    ECIDisableEvents(junction_id)
    for sg_id in to_turn_off:
        ECIChangeSignalGroupState(junction_id, sg_id, amber_signal, timeSta, time1, acycle)

def turn_movements_to_red(to_turn_off, timeSta, time1, acycle):
    """Sets only the old movements fully red; continuing remain green."""
    #AKIPrintString(f"TURNING OFF => RED for {to_turn_off}")
    ECIDisableEvents(junction_id)
    for sg_id in to_turn_off:
        ECIChangeSignalGroupState(junction_id, sg_id, red_signal, timeSta, time1, acycle)

def finalize_new_green_set(next_green, timeSta, time1, acycle):
    """Sets the *new* movements green; continuing were never turned off."""
    global current_green_set

    # current_green_set.clear()
    # num_signal_groups = ECIGetNumberSignalGroups(junction_id)
    # for i in range(1, num_signal_groups + 1):
    #     state_sg = ECIGetCurrentStateofSignalGroup(junction_id, i)
    #     if state_sg == 1:
    #         current_green_set.add(i)

    new_movements = next_green - current_green_set  # only truly new get turned green now
    #AKIPrintString(f"TURNING NEW MOVEMENTS => GREEN for {new_movements}")
    ECIDisableEvents(junction_id)
    for sg_id in new_movements:
        ECIChangeSignalGroupState(junction_id, sg_id, green_signal, timeSta, time1, acycle)
    # continuing ones remain green
