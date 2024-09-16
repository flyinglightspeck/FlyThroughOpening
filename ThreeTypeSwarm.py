import math
import matplotlib as mpl

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

from FLS import FLS
from Controller import Controller
from State import StateTypes
from util import *
from config import Config

# matplotlib.use('TkAgg')
mpl.use('macosx')

# params = {
#     'gamma_Acc': 0.25, 'd_v_0': 3.0, 'l_Acc': 2.5,
#     'gamma_z': 0.25, 'd_z_0': 1.25, 'a_z': 1.0, 'L_z_2': 1.75,
#     'gamma_Ali': 0.4, 'd_Ali_0': 1.0, 'l_Ali': 2.5, 'alpha_Ali': 1.0,
#     'gamma_Att': 0.25, 'd_Att_0': 1.25, 'l_Att': 2.75, 'alpha_Att': 1.0,
#     'gamma_perp': 0.5, 'gamma_parallel': 0.5, 'sigma_z': 1,
#     'gamma_w': 1.2, 'e_w1': 1.25, 'e_w2': 0.0, 'l_w': 2.5,
#     'center': np.array([0, 0]), 'radius': 20
# }

time_step = 1 / 20
fls_num = 2
min_distance = 0.5
fls_size = 0.25

delta = 0.1

np.random.seed(42)

# init_formation = generate_points(-4, 4, -4, 4, 8, 12, fls_num, min_distance)
#
# flss = [FLS(i, init_formation[i], [np.random.uniform(0, 2 * np.pi), np.random.uniform(0, 2 * np.pi)], 0.5,
#             time_step, [0.7, 0.4], [1, 1], slot_ID=-1) for i in range(fls_num)]
dist_between = 10
init_formation = np.array([[dist_between/2, 0, 10], [-dist_between/2, 0, 10]])

headings = np.array([[np.pi, 0], [0, 0]])

flss = [FLS(i, init_formation[i], headings[i], 0.5,
            time_step, [0.7, 0.4], [1, 1], slot_ID=-1) for i in range(fls_num)]


ctrller = Controller(flss, None, time_step, delta)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


def update_plot(axis):
    ctrller.update_FLS_swarm(Config)
    # positions = swarm.get_positions()
    paths = ctrller.get_paths()

    axis.cla()

    draw_cylinder(axis, Config.center, Config.radius, 20)
    for i, path in enumerate(paths):
        path = np.array(path)
        axis.plot(path[:, 0], path[:, 1], path[:, 2], linewidth=2, alpha=0.8)  # Draw the path

        if i == 0:
            axis.scatter(path[-1, 0], path[-1, 1], path[-1, 2], s=20, c='r')
        else:
            axis.scatter(path[-1, 0], path[-1, 1], path[-1, 2], s=20, c='b')  # Draw FLSs

    axis.set_xlim(-Config.radius, Config.radius)
    axis.set_ylim(-Config.radius, Config.radius)
    axis.set_zlim(0, 12)
    axis.set_xlabel('X')
    axis.set_ylabel('Y')
    axis.set_zlabel('Z')

    print(f"Dist Between: {np.linalg.norm(flss[0].position - flss[1].position): .2f}")

    print(f"Speed FLS 1: {flss[0].velocity: .2f}, Speed FLS 2: {flss[1].velocity: .2f}")
    # metrics = swarm.get_behavior()
    # print(f"D: {metrics[0]:.2f}, P: {metrics[1]:.2f}, M: {metrics[2]:.2f}, Collisions: {colliding_counter}")

time_lable = fig.text(0.1, 0.9, f"Time: {0:.2f}", fontsize=12, color='k')

for frame in range(math.ceil(200 / time_step)):
    update_plot(ax)
    time_lable.set_text(f"Time: {frame * time_step:.2f}")

    # plt.show()
    plt.draw()
    plt.pause(time_step)
