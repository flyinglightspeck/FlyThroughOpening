import matplotlib as mpl

from FLS import FLS
from FlightPattern import FlightPattern
from Controller import Controller
from util import *
from config import Config

# matplotlib.use('TkAgg')
# mpl.use('macosx')


mpl.use('macosx')
params = {
        'gamma_Acc': 0.25, 'd_v_0': 1.0, 'l_Acc': 2.5,
        'gamma_z': 0.25, 'd_z_0': 1.25, 'a_z': 1.0, 'L_z_2': 1.75,
        'gamma_Ali': 0.4, 'd_Ali_0': 1.0, 'l_Ali': 2.5, 'alpha_Ali': 1.0,
        'gamma_Att': 0.25, 'd_Att_0': 1.25, 'l_Att': 2.75, 'alpha_Att': 1.0,
        'gamma_perp': 0.5, 'gamma_parallel': 0.5, 'sigma_z': 1,
        'gamma_w': 1.2, 'e_w1': 1.25, 'e_w2': 0.0, 'l_w': 2.5,
        'center': np.array([0, 0, 100]), 'radius': 1, 'dist_to_opening': 0.2, 'normal_vector': np.array([0, 0, 1]),
        'gamma_Dest_h': 2, 'gamma_Dest_v': 0.4, 'l_Dest': 3, 'alfa_Dest_d': 2, 'alfa_Dest_v': 1,
        'v_Dest': 0.7, 'slot_num': 5,
        'time_step': 1/30, 'fls_size': 0.15/2,
        'path_policy': 0,
        'space': 1.5, 'init_altitude': 0.1
    }

# np.random.seed(42)


if __name__ == "__main__":
    normal_vector = Config.normal_vector / np.linalg.norm(Config.normal_vector)
    flight_pattern = FlightPattern(Config.center, Config.radius, Config.dist_to_opening, Config.v_Dest,
                                   Config.slot_num, Config.time_step, normal_vector)

    assign_IDs = [0, 1, 2, 3, 4]
    total_data = []
    # data_dist = []
    # for dist_to_FP in [1*Config.radius, 3*Config.radius, 5*Config.radius, 7*Config.radius, 10*Config.radius, 100*Config.radius, 1000*Config.radius]:


    dist_to_FP = 5 * Config.radius

    points_on_line = [[i*Config.radius, 1, Config.center[2] - dist_to_FP] for i in range(10)]
    for i in range(len(points_on_line)):

        data_dist = []
        for assign_ID in assign_IDs:

            slot_assignment = [assign_ID]
            # Initialize swarm with flss
            flss = [FLS(0, points_on_line[i], [np.random.uniform(0, 2 * np.pi), np.random.uniform(0, 2 * np.pi)], 0,
                        Config.time_step, [1.5, 0], [1, 1], slot_ID=slot_assignment[0])]


            delta = 0.1
            ctrller = Controller(flss, flight_pattern, Config.time_step, delta)

            data = []
            for policy in [0, 1]:
                ctrller.predict_slots(flss[0], float('inf'), policy=policy)
                time = flss[0].destination.expiration * Config.time_step
                distance = np.linalg.norm(flss[0].position - flss[0].destination.coordinate)

                if flss[0].destination.expected_arrive_time is not None:
                    delta = flss[0].destination.expected_arrive_time - np.linalg.norm(flss[0].position - flss[0].destination.coordinate) / (Config.v_Dest / 2)
                    if delta < 0:
                        delta = 0
                else:
                    delta = 0
                print(time, distance)
                travel_time = time - delta
                avg_speed = distance/travel_time

                # data.extend([delta, travel_time, avg_speed])
                data.append([time, distance])
            data_dist.append(data)

        total_data.append(data_dist)
        # csv_file = 'output_assign.csv'

        # with open(csv_file, 'a', newline='') as file:
        #     writer = csv.writer(file)
        #     # # If the file doesn't exist, write the header
        #     writer.writerow([f"{dist_to_FP/Config.radius:.0f}R time FTT", f"{dist_to_FP/Config.radius:.0f}R distance FTT",
        #                      f"{dist_to_FP/Config.radius:.0f}R time SD", f"{dist_to_FP/Config.radius:.0f}R distance SD"])
        #
        #
        #     # writer.writerow([f"{dist_to_FP/Config.radius:.0f}R Delta FTT", f"{dist_to_FP/Config.radius:.0f}R TT FTT", f"{dist_to_FP/Config.radius:.0f}R speed FTT",
        #     #                  f"{dist_to_FP/Config.radius:.0f}R Delta SD", f"{dist_to_FP/Config.radius:.0f}R TT SD", f"{dist_to_FP/Config.radius:.0f}R speed SD",])
        #
        #     # Append the data
        #     writer.writerows(data_dist)
        #
        # print(f"Data appended to {csv_file}")

# # 5th point FTT time, different assignment
#     draw_data = total_data[:5]
#
#     y_lists = []
#     for i in assign_IDs:
#         y_lists.append([data[i][0][0] for data in draw_data])
#
#     x = [1*Config.radius, 3*Config.radius, 5*Config.radius, 7*Config.radius, 10*Config.radius]
#     line_names = [f"Assigned to Slot {i+1:.0f}" for i in assign_IDs]
#     x_label = 'Distance From the Line To the Flight Pattern (Meter)'
#     y_label = 'Time To Rendezvous (Second)'
#     draw_line_chart(y_lists, x, line_names, x_label, y_label, save_name='../results/5th_point_rendezvousetime_by_linedist_differentAssignment_FTT.png')

# # 5th point FTT dist, different assignment
#     draw_data = total_data[:5]
#
#     y_lists = []
#     for i in assign_IDs:
#         y_lists.append([data[i][0][1] for data in draw_data])
#
#     x = [1*Config.radius, 3*Config.radius, 5*Config.radius, 7*Config.radius, 10*Config.radius]
#     line_names = [f"Assigned to Slot {i+1:.0f}" for i in assign_IDs]
#     x_label = 'Distance From the Line To the Flight Pattern (Meter)'
#     y_label = 'Distance Traveled To Rendezvous (Meter)'
#     draw_line_chart(y_lists, x, line_names, x_label, y_label, save_name='../results/5th_point_traveldist_by_linedist_differentAssignment_FTT.png')

# 5th point SD time, different assignment
    draw_data = total_data[:5]

    y_lists = []
    for i in assign_IDs:
        y_lists.append([data[i][1][0] for data in draw_data])

    x = [1 * Config.radius, 3 * Config.radius, 5 * Config.radius, 7 * Config.radius, 10 * Config.radius]
    line_names = [f"Assigned to Slot {i + 1:.0f}" for i in assign_IDs]
    x_label = 'Distance From the Line To the Flight Pattern (Meter)'
    y_label = 'Time To Rendezvous (Second)'
    draw_line_chart(y_lists, x, line_names, x_label, y_label, save_name='../results/5th_point_rendezvousetime_by_linedist_differentAssignment_SD.png')

# # 5th point SD dist, different assignment
#     draw_data = total_data[:5]
#
#     y_lists = []
#     for i in assign_IDs:
#         y_lists.append([data[i][1][1] for data in draw_data])
#
#     x = [1*Config.radius, 3*Config.radius, 5*Config.radius, 7*Config.radius, 10*Config.radius]
#     line_names = [f"Assigned to Slot {i+1:.0f}" for i in assign_IDs]
#     x_label = 'Distance From the Line To the Flight Pattern (Meter)'
#     y_label = 'Distance Traveled To Rendezvous (Meter)'
#     draw_line_chart(y_lists, x, line_names, x_label, y_label, save_name='../results/5th_point_traveldist_by_linedist_differentAssignment_SD.png')


# # 5th point time for different slot assigned
#     draw_data = total_data[3]
#
#     y_FTT = [data[0][0] for data in draw_data]
#
#     y_SD = [data[1][0] for data in draw_data]
#
#     y_lists = [y_FTT, y_SD]
#
#     x = [i for i in assign_IDs]
#     line_names = ["FTT", 'SD
#     x_label = 'Distance From the Line To the Flight Pattern (Meter)'
#     y_label = 'Time To Rendezvous (Second)'
#     draw_line_chart(y_lists, x, line_names, x_label, y_label, save_name='../results/5th_point_rendezvousetime_by_assignedSlot_5R.png')
#
# # 5th point dist for different slot assigned
#     draw_data = total_data[3]
#
#     y_FTT = [data[0][1] for data in draw_data]
#
#     y_SD = [data[1][1] for data in draw_data]
#
#     y_lists = [y_FTT, y_SD]
#
#     x = [i for i in assign_IDs]
#     line_names = ["FTT", 'SD
#     x_label = 'Distance From the Line To the Flight Pattern (Meter)'
#     y_label = 'Distance Traveled To Rendezvous (Meter)'
#     draw_line_chart(y_lists, x, line_names, x_label, y_label, save_name='../results/5th_point_traveldist_by_assignedSlot_5R.png')


# 5E line , SD time, different assignment
    draw_data = total_data

    y_lists = []
    for i in assign_IDs:
        y_lists.append([data[i][1][0] for data in draw_data])

    x = [i for i in range(1, 11)]
    line_names = [f"Slot {i + 1:.0f}" for i in assign_IDs]
    x_label = 'Starting Point of the FLS'
    y_label = 'Time To Rendezvous (Second)'
    draw_line_chart(y_lists, x, line_names, x_label, y_label, save_name='../results/5R_rendezvousetime_by_linedist_differentAssignment_SD.png')
