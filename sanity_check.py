import importlib
import platform
import sys
import matplotlib as mpl
import multiprocessing as mp

from CollisionTracker import CollisionTracker
from FLS import FLS
from FlightPattern import FlightPattern
from Controller import Controller
from util import *
from simu_with_shapes import *
from config import Config

def run_LAX_check(shape_name, config, c_step, speed_error, heading_error, delta, fresh_rate=0,
                       wirte_traj=0, max_step=float('inf'), log_name=None):
    print(f"Simulation Start: Sanity Check")

    normal_vector = config.normal_vector / np.linalg.norm(config.normal_vector)

    flight_pattern = FlightPattern(config.center, config.radius, config.dist_to_opening, config.v_Dest,
                                   config.slot_num, config.time_step, normal_vector)

    # init_formation = init_formation[:10]

    # Initialize swarm with flss

    assign_IDs = [0, 1, 2, 3, 4]


    dist_to_FP = 5 * config.radius

    points_on_line = [[i*config.radius, 1, config.center[2] - dist_to_FP] for i in range(10)]

    flss = [FLS(0, points_on_line[0], [np.random.uniform(0, 2 * np.pi), np.random.uniform(0, 2 * np.pi)], 0,
                config.time_step, [1.5, 0], [1, 1], slot_ID=0)]

    ctrller = Controller(flss, flight_pattern, config.time_step, delta)

    # ctrller.predict_slots(config.v_Dest)

    if fresh_rate > 0:
        fig = plt.figure()
        fig.suptitle(f'Slot Speed: {config.v_Dest:.1f}, Radius: {config.radius:.1f}')
        manager = plt.get_current_fig_manager()
        manager.full_screen_toggle()
        ax = fig.add_subplot(111, projection='3d')

        # collision_lable = fig.text(0.1, 0.8, f"Collisions: {0:.0f}", fontsize=16, color='k')

    step = 0
    end_flag = False

    sync_num = 0
    total_collisions = []

    collision_tracker = CollisionTracker()

    while sync_num < len(flss):
        if step > 100000:
            break

        sync_count = 0

        for fls in flss:
            if fls.state == StateTypes.SYNC:
                sync_count += 1
        sync_num = max([sync_count, sync_num])

        step += 1

        if step >= max_step:
            step = float('inf')
            break

        if fresh_rate > 0 and (step % fresh_rate) == 0:
            end_flag, collisions, positions, new_deltas = update_control(config, ctrller, collision_tracker, speed_error,
                                                                         heading_error, c_step, True, ax)
            plt.draw()
            plt.pause(config.time_step)
        else:
            end_flag, collisions, positions, _ = update_control(config, ctrller, collision_tracker, speed_error, heading_error, c_step)

    print("Simulation Ended")
    sim_info = f"Simulation: Sanity Check, Time: {step * config.time_step}, Should Be: 11.2667"
    print(sim_info)

    if fresh_rate > 0:
        plt.show()
        plt.close('all')

    return step, total_collisions

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
    opt_step, collisions = run_LAX_check(config.shape, config, float('inf'), 0, 0, delta,
                                              fresh_rate=fresh_rate, wirte_traj=wirte_traj,
                                              log_name=simulation_name)