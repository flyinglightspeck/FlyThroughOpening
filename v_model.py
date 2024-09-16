# 8.807218206389686
# ID 215
# expected arrive time 6.233334
# expiration 187

import importlib
import platform
import sys
import matplotlib as mpl
import multiprocessing as mp

from CollisionTracker import CollisionTracker
from Destination import Destination
from FLS import FLS
from FlightPattern import FlightPattern
from Controller import Controller
from util import *
from simu_with_shapes import *
from config import Config


def fls_go_linear(fls, end_speed):
    destination = fls.destination.coordinate
    moving_vec = destination - fls.position
    dist = np.linalg.norm(moving_vec)

    if dist != 0:
        heading = moving_vec / dist

        factor_heading_error = [np.random.uniform(min([0, 0]), max([0, 0])) for _ in
                                range(2)]
        heading = angles_to_vector(vector_to_angles(heading) + factor_heading_error)
    else:
        heading = np.array([0, 0, 0])

    end_speed, dist_traveled = fls.make_move(dist, end_speed, 0)

    fls.update_state_linear(dist_traveled, end_speed, heading)

    if np.linalg.norm(destination - fls.position) < 1e-6:
        return True

    return False


def run_LAX_check(shape_name, config, c_step, speed_error, heading_error, delta, fresh_rate=0,
                  wirte_traj=0, max_step=float('inf'), log_name=None):
    print(f"Simulation Start: Sanity Check")

    fls = FLS(0, [0, 0, 0], [np.random.uniform(0, 2 * np.pi), np.random.uniform(0, 2 * np.pi)], 0,
              config.time_step, [1.5, 0], [1, 1], slot_ID=0)
    fls.destination = Destination([0, 0, 8.807218206389686], 187, 6.233334)
    step = 0

    while 1:
        if step > 100000:
            break



        step += 1
        arrived_flag = fls_go_linear(fls, config.v_Dest)
        if arrived_flag:
            break

    return


if __name__ == "__main__":
    # init_formations = np.array([[5, 0, 10]])
    deltas = [0.1]
    c_steps = [float('inf')]
    errors = np.array([[0, 0]])

    fresh_rate = 0
    wirte_traj = 0
    delta = 0.1

    N = ''

    module_name = f"config{N}"
    config_module = importlib.import_module(module_name)

    # Access the Config class
    config = config_module.Config

    simulation_name = f"{config.shape}_Q{config.Q}_S{config.speed_range[0]}_N_{config.slot_num}"

    np.random.seed(4)
    run_LAX_check(config.shape, config, float('inf'), 0, 0, delta,
                                         fresh_rate=fresh_rate, wirte_traj=wirte_traj,
                                         log_name=simulation_name)
