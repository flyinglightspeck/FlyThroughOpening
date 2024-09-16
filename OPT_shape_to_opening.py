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
from simu_with_shapes import update_drawing


if platform.system() == "Darwin":  # macOS
    mpl.use('macosx')

def update_control_OPT(config, ctrller, collision_tracker, show_animation=False, axis=None):
    current_time = ctrller.step * ctrller.time_step
    end_flag = ctrller.update_FLSs_linear(current_time + ctrller.time_step)

    positions = ctrller.get_positions()

    collisions, groups = check_collision_info(ctrller.flss, collision_tracker, 2 * config.fls_size, StateTypes.QUIT)

    if show_animation:
        colliding_list = [ID for c in groups for ID in c[2]]
        update_drawing(config, ctrller, axis, colliding_list, zorder=10)

    return end_flag, collisions, positions


def run_LAX_check(shape_name, config, c_step, speed_error, heading_error, delta, fresh_rate=0,
                  wirte_traj=0, max_step=float('inf'), log_name=None):
    print(f"Simulation Start:")

    shape_file = f'./assets/{shape_name}.xyz'
    init_formation = load_shape(shape_file, shrink_min_dist=config.Q * (2 * config.fls_size),
                                shift_to_center=True, shift_bottom=0)

    init_formation = init_formation[init_formation[:, 2].argsort()[::-1]]

    normal_vector = config.normal_vector / np.linalg.norm(config.normal_vector)

    config.center[2] = np.max(init_formation[:, 2]) + max(np.max(init_formation[:, 2]) * 0.2,
                                                          (config.v_Dest ** 2) / (2 * config.acc_range[0]))

    flight_pattern = FlightPattern(config.center, config.radius, config.dist_to_opening, config.v_Dest,
                                   config.slot_num, config.time_step, normal_vector)

    exit_vec = flight_pattern.slots[0] - flight_pattern.get_slot_coord(0, -1)
    exit_vec = exit_vec / np.linalg.norm(exit_vec)

    exit_time = math.sqrt(2 * config.dist_to_opening / config.acc_range[0]) * 1.42

    opening_pos = flight_pattern.slots[0] + exit_time * config.v_Dest * exit_vec + np.array(
        config.normal_vector) * config.dist_to_opening

    fls_num = len(init_formation)

    flss = [FLS(i, init_formation[i], [np.random.uniform(0, 2 * np.pi), np.random.uniform(0, 2 * np.pi)], 0,
                config.time_step, config.speed_range, config.acc_range, slot_ID=-1) for i in range(fls_num)]


    Lambda = (25 * 0.7) / (2 * math.pi * 1)
    schedule = opt_schedule_dict(flss, Lambda, opening_pos)
    ctrller = Controller(flss, None, config.time_step, delta, schedule)


    if fresh_rate > 0:
        config.space = max(np.max(init_formation[:, 0]), np.max(init_formation[:, 1])) * 1.2
        figure_path = f'{config.figrue_path}{config.shape}_{config.speed_range[0]}_OPT'
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

    exit_num = 0
    total_collisions = []

    collision_tracker = CollisionTracker()

    while exit_num < len(flss):
        # if step > 100000:
        #     break

        quit_count = 0
        for fls in flss:
            if fls.state == StateTypes.QUIT:
                quit_count += 1
                # fls.state = StateTypes.QUIT

        exit_num = max([quit_count, exit_num])

        if ctrller.step % 100 == 0:
            print(f"Step: {ctrller.step}, Exit FLS: {exit_num}, Collisions: {collision_tracker.get_collision_count()}")

        if ctrller.step >= max_step:
            ctrller.step = float('inf')
            break

        if fresh_rate > 0 and (ctrller.step % fresh_rate) == 0:
            end_flag, collisions, positions = update_control_OPT(config, ctrller, collision_tracker, True, ax)

            ax.scatter(opening_pos[0], opening_pos[1], opening_pos[2], s=30, c='r', alpha=0.3)
            # plt.draw()
            # plt.pause(config.time_step)
        else:
            end_flag, collisions, positions = update_control_OPT(config, ctrller, collision_tracker)

        ctrller.step += 1

        if len(collisions) > 0:
            total_collisions.append([ctrller.step, [c for c in collisions]])

        if fresh_rate > 0 and (ctrller.step % fresh_rate) == 0:
            collision_lable.set_text(f"Collisions: {collision_tracker.get_collision_count():.0f}")
            time_lable.set_text(f"Time: {ctrller.step * config.time_step:.2f}")
            arrive_count_lable.set_text(f"Exit FLSs: {exit_num:.0f}")

            fig.savefig(f'{figure_path}/{ctrller.step:.0f}.png', dpi=200)

    print("Simulation Ended")
    sim_info = f"Simulation: {log_name}, Total Steps: {ctrller.step}, Collisions: {collision_tracker.get_collision_count()}"
    print(sim_info)

    with open(f"{config.results_path}{log_name}_OPT.txt", "w") as file:
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


    with open(f"{config.results_path}{log_name}_OPT_time.csv", mode='w', newline='') as file:
        writer = csv.writer(file)
        # Write the string and list as a new row
        for fls in flss:
            writer.writerow([fls.ID, fls.assign_time, fls.depart_time])
    # if fresh_rate > 0:
    #     plt.show()
    #     plt.close('all')

    return ctrller.step, total_collisions


def run_with_multiProcess(delta, config):
    simulation_name = f"{config.shape}_Q{config.Q}_S{config.speed_range[0]}_N_{config.slot_num}"

    if not os.path.exists(config.results_path):
        os.makedirs(config.results_path, exist_ok=True)

    file_path = f'{config.results_path}{simulation_name}.csv'
    delete_file(file_path)

    np.random.seed(4)
    opt_step, collisions = run_LAX_check(config.shape, config, float('inf'), 0, 0, delta,
                                         fresh_rate=config.fresh_rate, wirte_traj=config.wirte_traj,
                                         log_name=simulation_name)

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
            simu_name = f"{config.shape}_Q{config.Q}_S{config.speed_range[0]}_N_{config.slot_num}_OPT"
            print(simu_name)

            if len(sys.argv) > 3:
                print(simu_name)
            else:
                p_list.append(mp.Process(target=run_with_multiProcess,
                                         args=(delta, config),
                                         name=simu_name))

        for p in p_list:
            print(p)
            p.start()
    else:

        N = '_local'

        module_name = f"config{N}"
        config_module = importlib.import_module(module_name)

        # Access the Config class
        config = config_module.Config

        simulation_name = f"{config.shape}_Q{config.Q}_S{config.speed_range[0]}_N_{0}"
        np.random.seed(4)
        opt_step, collisions = run_LAX_check(config.shape, config, float('inf'), 0, 0, delta,
                                             fresh_rate=config.fresh_rate, wirte_traj=config.wirte_traj,
                                             log_name=simulation_name)
