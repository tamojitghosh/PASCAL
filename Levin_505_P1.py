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
section_upstream_dict = {
    552: [658],   #509, 647,
    789: [1116],
    786: [1064],
    572: [988],     #UP TO THIS CENTER INTERSECTION
    471: [2285],
    523: [574],
    516: [547],
    499: [539],     #UP TO THIS SOUTHSIDE INTERSECTION
    1405: [1483],
    1422: [1461],
    1409: [1408],
    1415: [1401],   #UP TO THIS NORTHSIDE INTERSECTION
    419: [437],
    979: [879],
    772: [3750],     #UP TO THIS EASTSIDE INTERSECTION
    1084: [626],
    1140: [2294],
    1107: [1120],
    1102: [766]     #uP TO THIS WESTSIDE INTERSECTION
}
section_downstream_dict = {
    778: [769],
    774: [557],
    780: [772],
    776: [766],     #UP TO THIS CENTER INTERSECTION
    511: [509],
    502: [1644],
    2288: [517],
    2291: [513],    #UP TO THIS SOUTHSIDE INTERSECTION
    1432: [1402],
    1428: [1446],
    1430: [1443],
    1436: [1449],   #UP TO NORTHSIDE INTERSECTION
    2297: [1529],
    990: [988],
    431: [2300],    #UP TO EASTSIDE INTERSECTION
    2303: [1143],
    1091: [614],
    1064: [786],
    1092: [1137]    #UP TO WESTSIDE INTERSECTION
}
# Calling active model using scripting and later on it will be used to get the lane length
model = GKSystem.getSystem().getActiveModel()

def AAPILoad():
    return 0

def AAPIInit():
    return 0

def AAPISimulationReady():
    return 0

def AAPIManage(time1, timeSta, timeTrans, acycle):
    return 0

def AAPIPostManage(time1, timeSta, timeTrans, acycle):
    start = time.time()
    global step_counter
    global time_step

    step_counter += 1
    if step_counter % (time_step / acycle) == 0:
        handle_amber_signal(time1, timeSta, acycle)
    elif step_counter % ((time_step + amber_time) / acycle) == 0 and time1 != 0:
        handle_red_signal(time1, timeSta, acycle)
    elif time1 == 0 or step_counter % ((time_step + amber_time + all_red_time) / acycle) == 0:
        handle_signal_control(time1, timeSta, acycle)
        step_counter = 0
    return 0

def handle_amber_signal(time1, timeSta, acycle):
    num_signal_groups = ECIGetNumberSignalGroups(junction_id)
    green_groups = [i for i in range(1, num_signal_groups + 1) if ECIGetCurrentStateofSignalGroup(junction_id, i) == green_signal]

    ECIDisableEvents(junction_id)
    for group in green_groups:
        ECIChangeSignalGroupState(junction_id, group, amber_signal, timeSta, time1, acycle)

def handle_red_signal(time1, timeSta, acycle):
    ECIDisableEvents(junction_id)
    num_signal_groups = ECIGetNumberSignalGroups(junction_id)
    for signal_group in range(1, num_signal_groups + 1):
        ECIChangeSignalGroupState(junction_id, signal_group, red_signal, timeSta, time1, acycle)

def handle_signal_control(time1, timeSta, acycle):
    num_signal_groups = ECIGetNumberSignalGroups(junction_id)
    signal_group_veh_diff, signal_group_name, origin_veh_num, origin_sec_num_lanes, dest_sec_num_lanes, all_signal_groups_id = calculate_vehicle_counts(num_signal_groups)

    num_phases = ECIGetNumberPhases(junction_id)
    pressure_for_phase = calculate_phase_pressure(num_phases, signal_group_veh_diff, origin_sec_num_lanes, time1)

    critical_phase = max(pressure_for_phase, key=pressure_for_phase.get)
    critical_sg_list = change_signal_states(critical_phase, time1, timeSta, acycle, all_signal_groups_id)
    change_remaining_signals_to_red(critical_sg_list, all_signal_groups_id, timeSta, time1, acycle)

def calculate_vehicle_counts(num_signal_groups):
    signal_group_veh_diff = {}
    signal_group_name = {}
    origin_veh_num = {}
    origin_sec_num_lanes = {}
    dest_sec_num_lanes = {}
    all_signal_groups_id = []

    for signal_group in range(1, num_signal_groups + 1):
        num_turnings = ECIGetNumberTurningsofSignalGroup(junction_id, signal_group)
        nonChar = boolp()
        sg_name = AKIConvertToAsciiString(ECIGetLogicalNameofSignalGroup(junction_id, signal_group), True, nonChar)

        sum_vehicles_origin, sum_vehicles_destination, lane_nums_turn_origin_sec, lane_nums_turn_dest_sec = calculate_turning_vehicles(num_turnings, signal_group)

        veh_num_diff = (sum_vehicles_origin - sum_vehicles_destination) * len(lane_nums_turn_origin_sec)
        signal_group_veh_diff[signal_group] = veh_num_diff
        signal_group_name[signal_group] = sg_name
        origin_veh_num[signal_group] = sum_vehicles_origin
        origin_sec_num_lanes[signal_group] = len(lane_nums_turn_origin_sec)
        dest_sec_num_lanes[signal_group] = len(lane_nums_turn_dest_sec)
        all_signal_groups_id.append(signal_group)

    return signal_group_veh_diff, signal_group_name, origin_veh_num, origin_sec_num_lanes, dest_sec_num_lanes, all_signal_groups_id

def calculate_turning_vehicles(num_turnings, signal_group):
    sum_vehicles_origin = 0
    sum_vehicles_destination = 0
    lane_nums_turn_origin_sec = []
    lane_nums_turn_dest_sec = []

    for turning_index in range(num_turnings):
        fromSection = intp()
        toSection = intp()
        report = ECIGetFromToofTurningofSignalGroup(junction_id, signal_group, turning_index, fromSection, toSection)
        if report == 0:
            lane_vehicle_count_from_split_upstream_section = get_lane_vehicle_count(section_upstream_dict[fromSection.value()])
            lane_vehicle_count_from_section = get_lane_vehicle_count([fromSection.value()])
            lane_vehicle_count_to_section = get_lane_vehicle_count([toSection.value()])
            lane_vehicle_count_to_split_downstream_section = get_lane_vehicle_count(section_downstream_dict[toSection.value()])

            first_lane_turn_origin = AKIInfNetGetTurningOriginFromLane(fromSection.value(), toSection.value())
            last_lane_turn_origin = AKIInfNetGetTurningOriginToLane(fromSection.value(), toSection.value())
            first_lane_turn_destination = AKIInfNetGetTurningDestinationFromLane(fromSection.value(), toSection.value())
            last_lane_turn_destination = AKIInfNetGetTurningDestinationToLane(fromSection.value(), toSection.value())

            lane_nums_turn_origin_sec = list(range(first_lane_turn_origin, last_lane_turn_origin + 1))
            lane_nums_turn_dest_sec = list(range(first_lane_turn_destination, last_lane_turn_destination + 1))

            sum_vehicles_origin = sum(lane_vehicle_count_from_section.get(lane, 0) for lane in lane_nums_turn_origin_sec) + \
                                  sum(lane_vehicle_count_from_split_upstream_section.get(lane, 0) for lane in lane_nums_turn_origin_sec)
            sum_vehicles_destination = sum(lane_vehicle_count_to_section.values()) + \
                                       sum(lane_vehicle_count_to_split_downstream_section.values())

    return sum_vehicles_origin, sum_vehicles_destination, lane_nums_turn_origin_sec, lane_nums_turn_dest_sec

def get_lane_vehicle_count(section_ids):
    lane_vehicle_count = {}
    for section_id in section_ids:
        num_veh = AKIVehStateGetNbVehiclesSection(int(section_id), True)
        if num_veh != 0:
            for veh in range(num_veh):
                vehicle_info = AKIVehStateGetVehicleInfSection(section_id, veh)
                lane = vehicle_info.numberLane
                lane_vehicle_count[lane] = lane_vehicle_count.get(lane, 0) + 1
        else:
            lane_vehicle_count[0] = 0
    return lane_vehicle_count

def calculate_phase_pressure(num_phases, signal_group_veh_diff, origin_sec_num_lanes, time1):
    pressure_for_phase = {}
    for phase in range(1, num_phases + 1):
        if ECIIsAnInterPhase(junction_id, phase, time1) == 0:
            num_phase_signal_groups = ECIGetNbSignalGroupsPhaseofJunction(junction_id, phase, time1)
            weight_list = [signal_group_veh_diff[ECIGetSignalGroupPhaseofJunction(junction_id, phase, sg_index, time1)] for sg_index in range(num_phase_signal_groups)]
            pressure_for_phase[phase] = sum(weight_list)
    return pressure_for_phase

def change_signal_states(critical_phase, time1, timeSta, acycle, all_signal_groups_id):
    critical_phase_sg = ECIGetNbSignalGroupsPhaseofJunction(junction_id, critical_phase, time1)
    ECIDisableEvents(junction_id)
    critical_sg_list = [ECIGetSignalGroupPhaseofJunction(junction_id, critical_phase, sg_index, time1) for sg_index in range(critical_phase_sg)]
    for critical_sg_id in critical_sg_list:
        ECIChangeSignalGroupState(junction_id, critical_sg_id, green_signal, timeSta, time1, acycle)
    return critical_sg_list

def change_remaining_signals_to_red(critical_sg_list, all_signal_groups_id, timeSta, time1, acycle):
    red_sg = [sg for sg in all_signal_groups_id if sg not in critical_sg_list]
    for red in red_sg:
        ECIChangeSignalGroupState(junction_id, red, red_signal, timeSta, time1, acycle)

def AAPIFinish():
    return 0

def AAPIUnLoad():
    return 0

def AAPIPreRouteChoiceCalculation(time1, timeSta):
    return 0