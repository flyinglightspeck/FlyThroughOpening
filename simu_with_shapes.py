import importlib
import math
import platform
import sys
import matplotlib as mpl
import multiprocessing as mp

import numpy as np

from CollisionTracker import CollisionTracker
from FLS import FLS
from FlightPattern import FlightPattern
from Controller import Controller
from util import *

# if platform.system() == "Darwin":  # macOS
#     mpl.use('macosx')
# elif platform.system() == "Windows" or platform.system() == "Linux":  # Windows and Linux
#     mpl.use('TkAgg')


def update_control(config, ctrller, collision_tracker, speed_error, heading_error, c_step, show_animation=False, axis=None, end_state=StateTypes.QUIT):
    # ctrller.update_FLS_swarm(params)

    end_flag, deltas = ctrller.update_FLSs_linear_fp(config, speed_error, heading_error, c_step, redeploy_flag=False)
    # positions = swarm.get_positions()

    # print(f"Dist To Destination: {np.linalg.norm(slots[0] - positions[0]):.2f}")

    positions = ctrller.get_positions()

    collisions, groups = check_collision_info(ctrller.flss, collision_tracker, 2 * config.fls_size, end_state)

    if show_animation:
        colliding_list = [ID for c in groups for ID in c[2]]
        update_drawing(config, ctrller, axis, colliding_list, zorder=10)

    # metrics = ctrller.get_behavior()
    # print(f"D: {metrics[0]:.2f}, P: {metrics[1]:.2f}, M: {metrics[2]:.2f}, Collisions: {colliding_counter}")
    return end_flag, collisions, positions, deltas


def update_drawing(config, ctrller, axis, colliding_list, zorder=-1):
    paths = ctrller.get_paths()
    slots = ctrller.get_slots()

    axis.cla()
    # draw_cylinder(axis, config.center, config.radius, 20)
    for i, path in enumerate(paths):
        if len(path) == 0:
            continue
        path = np.array(path)
        axis.plot(path[:, 0], path[:, 1], path[:, 2], linewidth=1, alpha=0.8, zorder=zorder)  # Draw the path

        if i in colliding_list:
            axis.scatter(path[-1, 0], path[-1, 1], path[-1, 2], s=40, c='r', zorder=zorder)
        elif ctrller.flss[i].state == StateTypes.SYNC:
            axis.scatter(path[-1, 0], path[-1, 1], path[-1, 2], s=20, c='g', zorder=zorder)
        elif ctrller.flss[i].state == StateTypes.EXIT:
            axis.scatter(path[-1, 0], path[-1, 1], path[-1, 2], s=20, color=(32/255, 56/255, 136/255), zorder=zorder)
        elif ctrller.flss[i].state == StateTypes.QUIT:
            pass
            # axis.scatter(path[-1, 0], path[-1, 1], path[-1, 2], s=20, c='r')
        # elif ctrller.flss[i].state.value == StateTypes.END.value:
        elif ctrller.flss[i].state == StateTypes.DYN:
            axis.scatter(path[-1, 0], path[-1, 1], path[-1, 2], s=20, color=(81/255, 141/255, 219/255), zorder=zorder)
        else:
            axis.scatter(path[-1, 0], path[-1, 1], path[-1, 2], s=20, color=(167/255, 210/255, 228/255), alpha=0.2, zorder=zorder)  # Draw FLSs

        # draw_sphere(ax, path[-1], config.fls_size, 'b', 0.6)
    if len(slots) > 0:
        axis.scatter(slots[0:, 0], slots[0:, 1], slots[0:, 2], color=(225/255, 156/255, 102/255), marker='o', s=40, alpha=0.4, zorder=zorder)
    # for slot_pos in slots:
    #     draw_ring(axis, slot_pos, config.fls_size, 0.03, 'c', alpha=0.6)

    # starting_point = config.center
    # end_point = starting_point + config.normal_vector
    # x_values = [starting_point[0], end_point[0]]
    # y_values = [starting_point[1], end_point[1]]
    # z_values = [starting_point[2], end_point[2]]
    # axis.plot(x_values, y_values, z_values, label='Vector', color='blue')

    axis.set_xlim(-config.space, config.space)
    axis.set_ylim(-config.space, config.space)
    axis.set_zlim(0, (config.center[2] + config.dist_to_opening) * 1.1)
    # axis.set_xlabel('X', fontname='Times New Roman')
    # axis.set_ylabel('Y', fontname='Times New Roman')
    # axis.set_zlabel('Z', fontname='Times New Roman')
    axis.set_aspect('equal', adjustable='box')
    # axis.view_init(elev=30, azim=180, roll=0)

    axis.grid(True)
    axis.xaxis.pane.set_alpha(0.0)
    axis.yaxis.pane.set_alpha(0.0)
    # axis.zaxis.pane.set_alpha(0.0)
    # axis.xaxis.line.set_color((1.0, 1.0, 1.0, 0.0))
    # axis.yaxis.line.set_color((1.0, 1.0, 1.0, 0.0))
    # axis.zaxis.line.set_color((1.0, 1.0, 1.0, 0.0))
    # Hide axes ticks
    axis.set_xticks([])
    axis.set_yticks([])
    axis.set_zticks([])

    # axis.quiver(0.5, 0.0, 0.5, 0.5, 0, 0, color='red', arrow_length_ratio=0.1, label='X')
    # axis.quiver(0.5, 0.0, 0.5, 0, 1.0, 0, color='green', arrow_length_ratio=0.1, label='Y')
    # axis.quiver(0.5, 0.0, 0.5, 0, 0, 0.5, color='blue', arrow_length_ratio=0.1, label='Z')


def run_LAX_with_shape(shape_name, config, c_step, speed_error, heading_error, delta, fresh_rate=0,
                       wirte_traj=0, max_step=float('inf'), log_name=None):
    print(f"Simulation Start: {log_name}")
    shape_file = f'./assets/{shape_name}.xyz'
    init_formation = load_shape(shape_file, shrink_min_dist=config.Q * (2 * config.fls_size),
                                shift_to_center=True, shift_bottom=0)

    init_formation = init_formation[init_formation[:, 2].argsort()[::-1]]

    normal_vector = config.normal_vector / np.linalg.norm(config.normal_vector)

    config.center[2] = np.max(init_formation[:, 2]) + max(np.max(init_formation[:, 2]) * 0.2,
                                                          (config.v_Dest ** 2) / (2 * config.acc_range[0]))

    # config.center[2] = np.max(init_formation[:, 2]) + max(np.max(init_formation[:, 2]) * 0.2,
    #                                                          (config.speed_range[0] ** 2) / (2 * config.acc_range[0]))
    #
    # print("Notic the distance from the flight pattern to the shape is identified by max speed, originally speed of slot")

    flight_pattern = FlightPattern(config.center, config.radius, config.dist_to_opening, config.v_Dest,
                                   config.slot_num, config.time_step, normal_vector)

    exit_vec = flight_pattern.slots[0] - flight_pattern.get_slot_coord(0, -1)
    exit_vec = exit_vec/np.linalg.norm(exit_vec)

    exit_time = math.sqrt(2 * config.dist_to_opening/config.acc_range[0]) * 1.42

    opening_pos = flight_pattern.slots[0] + exit_time * config.v_Dest * exit_vec + np.array(config.normal_vector) * config.dist_to_opening

    # init_formation = init_formation[:10]

    fls_num = len(init_formation)

    slot_assignment = flight_pattern.assign_slot(fls_num)

    # Initialize swarm with flss
    flss = [FLS(i, init_formation[i], [np.random.uniform(0, 2 * np.pi), np.random.uniform(0, 2 * np.pi)], 0,
                config.time_step, config.speed_range, config.acc_range, slot_ID=slot_assignment[i]) for i in
            range(fls_num)]

    consume_step = ceil(config.inter_arrival_time / config.time_step)


    ctrller = Controller(flss, flight_pattern, config.time_step, delta, consume_step=consume_step)

    # ctrller.predict_slots(config.v_Dest)

    if fresh_rate > 0:
        config.space = max(np.max(init_formation[:, 0]), np.max(init_formation[:, 1])) * 1.2
        figure_path = f'{config.figrue_path}{config.shape}_{config.path_policy}'
        if not os.path.exists(figure_path):
            os.makedirs(figure_path)

        fig = plt.figure(figsize=[10, 10])
        # fig.suptitle(f'Slot Speed: {config.v_Dest:.1f}, Radius: {config.radius:.1f}')
        manager = plt.get_current_fig_manager()
        # manager.full_screen_toggle()
        ax = fig.add_subplot(111, projection='3d')

        ax.set_position([0, 0, 1, 1])

        time_lable = fig.text(0.02, 0.18, f"Time: {0:.2f}", fontsize=16, color='k')
        collision_lable = fig.text(0.02, 0.22, f"Collisions: {0:.0f}", fontsize=16, color='k')
        arrive_count_lable = fig.text(0.02, 0.26, f"Exit FLSs: {0:.0f}", fontsize=16, color='k')

    ctrller.step = 0
    end_flag = False

    traj_file_name = f'./results/traj_{config.v_Dest:.1f}_{config.radius:.1f}_R{wirte_traj:.0f}_ALL.csv'
    if wirte_traj > 0:
        positions = ctrller.get_positions()
        write_tractory(traj_file_name, positions,
                       f'{ctrller.step * config.time_step:.2f}')

    exit_num = 0
    total_collisions = []
    num_of_DYN_FLS = np.zeros(100)

    collision_tracker = CollisionTracker()

    while exit_num < len(flss):
        if ctrller.step > 100000:
            break

        quit_count = 0
        DYN_count = 0
        for fls in flss:
            if fls.state == StateTypes.QUIT:
                quit_count += 1
            elif fls.state == StateTypes.DYN:
                DYN_count += 1
        exit_num = max([quit_count, exit_num])

        num_of_DYN_FLS[ctrller.step % 100] = DYN_count

        ctrller.step += 1

        if ctrller.step % 100 == 0:
        #     print(f"ctrller.step: {ctrller.step}, Exit FLS: {exit_num}, Collisions: {collision_tracker.get_collision_count()}")
        #
            log_moving_FLS(num_of_DYN_FLS, f'{config.results_path}{log_name}.csv')
            num_of_DYN_FLS = np.zeros(100)

        if ctrller.step >= max_step:
            ctrller.step = float('inf')
            break

        if fresh_rate > 0 and (ctrller.step % fresh_rate) == 0:
            end_flag, collisions, positions, new_deltas = update_control(config, ctrller, collision_tracker, speed_error,
                                                                         heading_error, c_step, True, ax)

            # print(flss[0].velocity, f"Time: {step * config.time_step:.2f}")

            # draw_opening(ax, opening_pos + np.array([0,0,2]), 0.5, color='dimgrey', alpha=1, zorder=1)
            #
            #
            plt.draw()
            plt.pause(config.time_step)
        else:
            end_flag, collisions, positions, _ = update_control(config, ctrller, collision_tracker, speed_error, heading_error, c_step)

        if len(collisions) > 0:
            total_collisions.append([ctrller.step, [c for c in collisions]])

        if fresh_rate > 0 and (ctrller.step % fresh_rate) == 0:
            collision_lable.set_text(f"Collisions: {collision_tracker.get_collision_count():.0f}")
            time_lable.set_text(f"Time: {ctrller.step * config.time_step:.2f}")
            arrive_count_lable.set_text(f"Exit FLSs: {exit_num:.0f}")

            fig.savefig(f'{figure_path}/{ctrller.step:.0f}.png', dpi=200)

        if wirte_traj and (ctrller.step % wirte_traj) == 0:
            skip_list = []
            # for fls in flss:
            # if fls.state == StateTypes.DYN and fls.velocity == 0:
            #     skip_list.append(fls.ID)

            write_tractory(traj_file_name, positions,
                           f'{ctrller.step * config.time_step:.2f}', skip_list)

    if ctrller.step % 100 > 0:
        print(f"Step: {ctrller.step}, Exit FLS: {exit_num}, Collisions: {collision_tracker.get_collision_count()}")
    #     log_moving_FLS(num_of_DYN_FLS, f'{config.results_path}{log_name}.csv')

    print("Simulation Ended")
    sim_info = f"Simulation: {log_name}, Total Steps: {ctrller.step}, Collisions: {collision_tracker.get_collision_count()}"
    print(sim_info)

    with open(f"{config.results_path}{log_name}.txt", "w") as file:
        file.write(sim_info + "\n")

        file.write(f"Step, Number of Collision\n")
        file.write(f" Collision ID, Is Exist, Involving FLSs, [FLS Info]]\n")

        for row in total_collisions:
            file.write(f"{row[0]} {len(row[1])} \n")
            for collision in row[1]:
                file.write(f" {collision[0]} {collision[1]} {collision[2]}")
                for f_str in collision[3]:
                    file.write(f" {f_str}")
                file.write(f"\n")

    with open(f"{config.results_path}{log_name}_time.csv", mode='w', newline='') as file:
        writer = csv.writer(file)
        # Write the string and list as a new row
        for fls in flss:
            writer.writerow([fls.ID, fls.assign_time, fls.depart_time])

    if fresh_rate > 0:
        plt.show()
        plt.close('all')

    return ctrller.step, total_collisions

def run_with_multiProcess(delta, config, fresh_rate, wirte_traj):
    simulation_name = f"{config.shape}_Q{config.Q}_S{config.speed_range[0]}_N_{config.slot_num}_P{config.path_policy}"

    if not os.path.exists(config.results_path):
        os.makedirs(config.results_path, exist_ok=True)

    file_path = f'{config.results_path}{simulation_name}.csv'
    delete_file(file_path)

    np.random.seed(4)
    opt_step, collisions = run_LAX_with_shape(config.shape, config, float('inf'), 0, 0, delta,
                                              fresh_rate=fresh_rate, wirte_traj=wirte_traj,
                                              log_name=simulation_name)
    # df = pd.read_csv(file_path, header=None)
    # y_lists = [df[0].tolist()]
    # plot_line(y_lists, [i for i in range(len(y_lists[0]))], " ", "Steps", "Moving FLSs",
    #           save_name=f"{config.results_path}{simulation_name}.png")


if __name__ == "__main__":


    # init_formations = np.array([[5, 0, 10]])
    deltas = [0.1]
    c_steps = [float('inf')]
    errors = np.array([[0, 0]])
    delta = 0.1

    if len(sys.argv) > 2:
        server_ID = int(sys.argv[1])
        job_num = int(sys.argv[2])

        # print([N, shape_num, config_set_num])

        sys.path.append(os.path.join(os.getcwd(), 'experiments'))

        p_list = []
        config_index = 0
        for i in range(job_num):
            np.random.seed(4)
            config_index = server_ID * job_num + i
            module_name = f"config{config_index}"
            spec = importlib.util.find_spec(module_name)
            if spec is not None:
                # Module exists, so import it
                config_module = importlib.import_module(module_name)
            else:
                # Module does not exist
                print(f"Module '{module_name}' not found.")
                continue

            config = config_module.Config
            simu_name = f"{config.shape}_Q{config.Q}_S{config.speed_range[0]}_N_{config.slot_num}_P{config.path_policy}"
            print(simu_name)

            if len(sys.argv) > 3:
                print(simu_name)
            else:
                p_list.append(mp.Process(target=run_with_multiProcess,
                                         args=(delta, config, config.fresh_rate, config.wirte_traj),
                                         name=simu_name))

        for p in p_list:
            print(p)
            p.start()
    else:
        N = '_local'

        module_name = f"config{N}"

        spec = importlib.util.find_spec(module_name)
        if spec is not None:
            # Module exists, so import it
            config_module = importlib.import_module(module_name)
        else:
            # Module does not exist
            print(f"Module '{module_name}' not found.")
            exit()

        # Access the Config class
        config = config_module.Config

        simulation_name = f"{config.shape}_Q{config.Q}_S{config.speed_range[0]}_N_{config.slot_num}_test"

        if not os.path.exists(config.results_path):
            os.makedirs(config.results_path, exist_ok=True)

        file_path = f'{config.results_path}{simulation_name}.csv'
        delete_file(file_path)

        np.random.seed(4)
        opt_step, collisions = run_LAX_with_shape(config.shape, config, float('inf'), 0, 0, delta,
                                                  fresh_rate=config.fresh_rate, wirte_traj=config.wirte_traj,
                                                  log_name=simulation_name)
        # df = pd.read_csv(file_path, header=None)
        # y_lists = [df[0].tolist()]
        # plot_line(y_lists, [i for i in range(len(y_lists[0]))], " ", "Steps", "Moving FLSs",
        #           save_name=f"{config.results_path}{simulation_name}.png")
