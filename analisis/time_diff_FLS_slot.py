import math
import matplotlib as mpl

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

from FLS import FLS
from FlightPattern import FlightPattern
from Controller import Controller
from State import StateTypes
from util import *

from config import Config

# matplotlib.use('TkAgg')
# mpl.use('macosx')

# params = {
#     'gamma_Acc': 0.25, 'd_v_0': 1.0, 'l_Acc': 2.5,
#     'gamma_z': 0.25, 'd_z_0': 1.25, 'a_z': 1.0, 'L_z_2': 1.75,
#     'gamma_Ali': 0.4, 'd_Ali_0': 1.0, 'l_Ali': 2.5, 'alpha_Ali': 1.0,
#     'gamma_Att': 0.25, 'd_Att_0': 1.25, 'l_Att': 2.75, 'alpha_Att': 1.0,
#     'gamma_perp': 0.5, 'gamma_parallel': 0.5, 'sigma_z': 1,
#     'gamma_w': 1.2, 'e_w1': 1.25, 'e_w2': 0.0, 'l_w': 2.5,
#     'center': np.array([0, 0, 30]), 'radius': 5, 'opening': np.array([3, 0, 35]),
#     'gamma_Dest_h': 2, 'gamma_Dest_v': 0.4, 'l_Dest': 3, 'alfa_Dest_d': 2, 'alfa_Dest_v': 1,
#     'v_Dest': 0.7,
#     'time_step': 1 / 20, 'fls_size': 0.25
# }

mpl.use('macosx')
# params = {
#         'gamma_Acc': 0.25, 'd_v_0': 1.0, 'l_Acc': 2.5,
#         'gamma_z': 0.25, 'd_z_0': 1.25, 'a_z': 1.0, 'L_z_2': 1.75,
#         'gamma_Ali': 0.4, 'd_Ali_0': 1.0, 'l_Ali': 2.5, 'alpha_Ali': 1.0,
#         'gamma_Att': 0.25, 'd_Att_0': 1.25, 'l_Att': 2.75, 'alpha_Att': 1.0,
#         'gamma_perp': 0.5, 'gamma_parallel': 0.5, 'sigma_z': 1,
#         'gamma_w': 1.2, 'e_w1': 1.25, 'e_w2': 0.0, 'l_w': 2.5,
#         'center': np.array([0, 0, 0.8]), 'radius': 1, 'dist_to_opening': 0.2, 'normal_vector': np.array([0, 0, 1]),
#         'gamma_Dest_h': 2, 'gamma_Dest_v': 0.4, 'l_Dest': 3, 'alfa_Dest_d': 2, 'alfa_Dest_v': 1,
#         'v_Dest': 0.7, 'slot_num': 5,
#         'time_step': 1/30, 'fls_size': 0.15/2,
#         'path_policy': 0,
#         'space': 1.5, 'init_altitude': 0.1
#     }

# np.random.seed(42)


if __name__ == "__main__":
    c_steps = [1, 10, 100]
    # c_step = float('inf')

    fls_num = 1
    delta = 0.1
    init_formations = np.array([[1, 1, 0]])

    for init_coords, init_name in zip(init_formations, ['Far Initial Point', 'Close Initial Point']):

        flight_pattern = FlightPattern(Config.center, Config.radius, np.array([0,0,0]), Config.v_Dest, 10,
                                       Config.time_step)

        fls = FLS(0, init_coords, [np.random.uniform(0, 2 * np.pi), np.random.uniform(0, 2 * np.pi)], 1.5,
                  Config.time_step, [1.5, 0.4], [1, 1], slot_ID=0)

        dist_to_center_xy = np.linalg.norm(fls.position[:2] - Config.center[:2])
        dist_to_center_z = abs(fls.position[2] - Config.center[2])

        # dist_lower_bound = np.sqrt(min(abs(dist_to_center_xy - Config.radius), dist_to_center_xy) ** 2
        #                            + dist_to_center_z ** 2)

        dist_lower_bound = np.sqrt((dist_to_center_xy - Config.radius) ** 2 + dist_to_center_z ** 2)
        dist_upper_bound = np.sqrt((dist_to_center_xy + Config.radius) ** 2 + dist_to_center_z ** 2)

        _, _, time_lower_bound = fls.linear_movement_OPT(dist_lower_bound, Config.v_Dest, float('inf'))
        _, _, time_upper_bound = fls.linear_movement_OPT(dist_upper_bound, Config.v_Dest, float('inf'))

        step_lower_bound = floor(time_lower_bound/Config.time_step)

        step_upper_bound = ceil(time_upper_bound/Config.time_step)

        end_flag = False
        y_data = []
        for step in range(int(step_lower_bound), int(step_upper_bound)+1):

            destination = flight_pattern.get_slot_coord(fls.slot_ID, steps=step + 1)
            dist = np.linalg.norm(destination - fls.position)
            _, _, t = fls.linear_movement_OPT(dist, Config.v_Dest, float('inf'))

            time_diff = t - step * Config.time_step
            y_data.append(time_diff)


        # draw_line_chart([y_data], list(range(int(step_lower_bound), int(step_upper_bound * 2 - step_lower_bound)+1)), " ",
        #                 f"Time Difference", "Time (Second)", 'Arriving Time Difference (Second)')

        num_points = int(step_upper_bound)+1 - (int(step_lower_bound))
        draw_line_chart([y_data], np.linspace(time_lower_bound, time_upper_bound, num_points), " ",
                             "Time (Second)", 'Arriving Time Difference (Second)', '../results/timediff_by_t_old.png')

        # plt.show()

        lo = floor(time_lower_bound / fls.time_step)
        hi = ceil(time_upper_bound / fls.time_step)
        search_counter = 0
        while lo < hi:
            mid = (lo + hi) // 2
            search_counter += 1
            destination = flight_pattern.get_slot_coord(0, steps=mid + 1)
            dist = np.linalg.norm(destination - fls.position)
            _, _, t = fls.linear_movement_OPT(dist, flight_pattern.slot_speed, float('inf'))

            time_diff = t - mid * Config.time_step

            if time_diff > 0:
                lo = mid + 1
            else:
                hi = mid
        print(search_counter)
        print(flight_pattern.get_slot_coord(fls.slot_ID, steps=lo+1), lo)
        print(np.linalg.norm(flight_pattern.get_slot_coord(fls.slot_ID, steps=lo+1) - fls.position))