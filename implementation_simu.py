import matplotlib as mpl

from FLS import FLS
from FlightPattern import FlightPattern
from Controller import Controller
from util import *
from config import Config

# matplotlib.use('TkAgg')
mpl.use('macosx')

def run_LAX(fls_num, config, iteration, c_step, speed_error, heading_error, delta, fresh_rate=True,
            wirte_traj=0, max_step=float('inf')):
    normal_vector = Config.normal_vector / np.linalg.norm(Config.normal_vector)
    flight_pattern = FlightPattern(Config.center, Config.radius, Config.dist_to_openingconfig, Config.v_Dest,
                                   Config.slot_numconfig, Config.time_step, normal_vector)

    # init_formation = generate_points(-Config.spaceconfig, Config.spaceconfig, -Config.spaceconfig, Config.spaceconfig,
    #                                  Config.init_altitudeconfig, Config.init_altitudeconfig, fls_num, 2 * Config.fls_sizeconfig)

    init_formation = np.array([[1, 1, 0.0], [-1, 0, 0.], [0, -1, 0.0], [-1, -1, 0.0], [0, 1, 0.0]])

    # slot_assignment = flight_pattern.assign_slot(fls_num)

    slot_assignment = flight_pattern.assign_slot(fls_num, [0, 3, 1, 2, 4])

    # Initialize swarm with flss
    flss = [FLS(i, init_formation[i], [np.random.uniform(0, 2 * np.pi), np.random.uniform(0, 2 * np.pi)], 0,
                Config.time_step, [1.5, 0], [1, 1], slot_ID=slot_assignment[i]) for i in range(fls_num)]

    ctrller = Controller(flss, flight_pattern, Config.time_step, delta)

    # ctrller.predict_slots(Config.v_Dest)

    def update_control(config, show_animation=False, axis=None):
        # ctrller.update_FLS_swarm(config)
        end_flag, deltas = ctrller.update_FLSs_linear_fp(config, speed_error, heading_error, c_step)
        # positions = swarm.get_positions()

        if show_animation and axis is not None:
            paths = ctrller.get_paths()
            slots = ctrller.get_slots()

            axis.cla()
            # draw_cylinder(axis, Config.center, Config.radius, 20)
            for i, path in enumerate(paths):
                path = np.array(path)
                axis.plot(path[:, 0], path[:, 1], path[:, 2], linewidth=2, alpha=0.8)  # Draw the path

                if ctrller.flss[i].state == StateTypes.SYNC:
                    axis.scatter(path[-1, 0], path[-1, 1], path[-1, 2], s=20, c='g')
                elif ctrller.flss[i].state == StateTypes.EXIT:
                    axis.scatter(path[-1, 0], path[-1, 1], path[-1, 2], s=20, c='c')
                elif ctrller.flss[i].state.value >= StateTypes.END.value:
                    axis.scatter(path[-1, 0], path[-1, 1], path[-1, 2], s=20, c='r')
                else:
                    axis.scatter(path[-1, 0], path[-1, 1], path[-1, 2], s=20, c='b')  # Draw FLSs

                # draw_sphere(ax, path[-1], Config.fls_size, 'b', 0.6)

            axis.scatter(slots[0:, 0], slots[0:, 1], slots[0:, 2], c='c', marker='o')
            # for slot_pos in slots:
            #     draw_ring(axis, slot_pos, Config.fls_size, 0.03, 'c', alpha=0.6)

            # starting_point = Config.center
            # end_point = starting_point + Config.normal_vector
            # x_values = [starting_point[0], end_point[0]]
            # y_values = [starting_point[1], end_point[1]]
            # z_values = [starting_point[2], end_point[2]]
            # axis.plot(x_values, y_values, z_values, label='Vector', color='blue')

            axis.set_xlim(-Config.space, Config.space)
            axis.set_ylim(-Config.space, Config.space)
            axis.set_zlim(0, (Config.center[2] + config.dist_to_opening) * 1.1)
            # axis.set_xlabel('X', fontname='Times New Roman')
            # axis.set_ylabel('Y', fontname='Times New Roman')
            # axis.set_zlabel('Z', fontname='Times New Roman')
            axis.set_aspect('equal', adjustable='box')
            ax.view_init(elev=30, azim=180, roll=0)

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

        # print(f"Dist To Destination: {np.linalg.norm(slots[0] - positions[0]):.2f}")

        positions = ctrller.get_positions()
        colliding_counter = get_collision_num(positions, 2 * config.fls_size)

        # metrics = ctrller.get_behavior()
        # print(f"D: {metrics[0]:.2f}, P: {metrics[1]:.2f}, M: {metrics[2]:.2f}, Collisions: {colliding_counter}")
        return end_flag, colliding_counter, positions, deltas

    if fresh_rate > 0:
        fig = plt.figure()
        fig.suptitle(f'Slot Speed: {config.v_Dest:.1f}, Radius: {config.radius:.1f}')
        manager = plt.get_current_fig_manager()
        manager.full_screen_toggle()
        ax = fig.add_subplot(111, projection='3d')

        # time_lable = fig.text(0.1, 0.9, f"Time: {0:.2f}", fontsize=12, color='k')
        # collision_lable = fig.text(0.1, 0.8, f"Collisions: {0:.0f}", fontsize=16, color='k')

        deltas = [None for _ in flss]
        speed_label = fig.text(0.1, 0.25,
                               f"  FLS 1: {flss[0].velocity:.2f}m/s\n{deltas[0]}\n  FLS 2: {flss[1].velocity:.2f}m/s\n{deltas[1]}\n  FLS 3: {flss[2].velocity:.2f}m/s\n{deltas[2]}\n  "
                               f"FLS 4: {flss[3].velocity:.2f}m/s\n{deltas[3]}\n  FLS 5: {flss[4].velocity:.2f}m/s\n{deltas[4]}",
                               fontsize=12, color='k', fontname='Times New Roman')

    step = 0
    end_flag = False

    traj_file_name = f'./results/traj_{config.v_Dest:.1f}_{config.radius:.1f}_R{wirte_traj:.0f}_ALL.csv'
    if wirte_traj > 0:
        positions = ctrller.get_positions()
        write_tractory(traj_file_name, positions,
                       f'{step * config.time_step:.2f}')

    quit_num = 0

    while quit_num < len(flss):
        if step == iteration:
            for fID in range(len(flss)):
                flss[fID].land()

        elif step > iteration:
            quit_count = 0
            for fls in flss:
                if fls.state == StateTypes.QUIT:
                    quit_count += 1
            quit_num = max([quit_count, quit_num])

        step += 1

        if step >= max_step:
            step = float('inf')
            break

        collisions = 0
        if fresh_rate > 0 and (step % fresh_rate) == 0:
            end_flag, collisions_num, positions, new_deltas = update_control(config, True, ax)

            collisions = max([collisions_num, collisions])

            # time_lable.set_text(f"Time: {step * config.time_step:.2f}")
            #
            # collision_lable.set_text(f"Collisions: {collisions_num:.0f}")
            for i in range(len(deltas)):
                if new_deltas[i] is None:
                    continue

                if deltas[i] is None or (deltas[i] == ' ' and new_deltas[i] != ' ') or (
                        deltas[i] != ' ' and new_deltas[i] == ' ' and flss[i].velocity > 0):
                    deltas[i] = new_deltas[i]

            speed_label.set_text(
                f"FLS 1: {flss[0].velocity:.2f} m/s\n{deltas[0]}\n\n  FLS 2: {flss[1].velocity:.2f} m/s\n{deltas[1]}\n\n  FLS 3: {flss[2].velocity:.2f} m/s\n{deltas[2]}\n\n  "
                f"FLS 4: {flss[3].velocity:.2f} m/s\n{deltas[3]}\n\n  FLS 5: {flss[4].velocity:.2f} m/s\n{deltas[4]}")

            # print(flss[0].velocity, f"Time: {step * config.time_step:.2f}")

            draw_opening(ax, np.array([1.01, 0.63, 1]), 0.06, color='dimgrey', alpha=0.8)

            # fig.savefig(f'./results/frames/{step:.0f}.png', dpi=300)
            plt.draw()
            plt.pause(config.time_step)
        else:
            end_flag, collisions_num, positions, _ = update_control(config)
            collisions = max([collisions_num, collisions])

        if wirte_traj and (step % wirte_traj) == 0:
            skip_list = []
            # for fls in flss:
                # if fls.state == StateTypes.DYN and fls.velocity == 0:
                #     skip_list.append(fls.ID)

            write_tractory(traj_file_name, positions,
                           f'{step * config.time_step:.2f}', skip_list)

    if fresh_rate > 0:
        plt.show()

    plt.close('all')
    return step, collisions


if __name__ == "__main__":

    # params = {
    #     'gamma_Acc': 0.25, 'd_v_0': 1.0, 'l_Acc': 2.5,
    #     'gamma_z': 0.25, 'd_z_0': 1.25, 'a_z': 1.0, 'L_z_2': 1.75,
    #     'gamma_Ali': 0.4, 'd_Ali_0': 1.0, 'l_Ali': 2.5, 'alpha_Ali': 1.0,
    #     'gamma_Att': 0.25, 'd_Att_0': 1.25, 'l_Att': 2.75, 'alpha_Att': 1.0,
    #     'gamma_perp': 0.5, 'gamma_parallel': 0.5, 'sigma_z': 1,
    #     'gamma_w': 1.2, 'e_w1': 1.25, 'e_w2': 0.0, 'l_w': 2.5,
    #     'center': np.array([0, 0, 0.8]), 'radius': 5, 'dist_to_opening': 0.2, 'normal_vector': np.array([0, 0, 1]),
    #     'gamma_Dest_h': 2, 'gamma_Dest_v': 0.4, 'l_Dest': 3, 'alfa_Dest_d': 2, 'alfa_Dest_v': 1,
    #     'v_Dest': 0.6, 'slot_num': 5,
    #     'time_step': 1/30, 'fls_size': 0.15/2,
    #     'path_policy': 0,
    #     'space': 1.5, 'init_altitude': 0.1
    # }

    # init_formations = np.array([[5, 0, 10]])
    fls_num = 5
    duration = 1 #minutes
    iterations = duration * 60 / Config.time_step
    deltas = [0.1]
    c_steps = [float('inf')]
    errors = np.array([[0, 0]])

    np.random.seed(4)

    fresh_rate = 1
    wirte_traj = 1
    delta = 0.1

    for slot_speed in [0.7]:
        Config.v_Dest = slot_speed
        for radius in [1]:
            Config.radius = radius
            opt_step, collisions = run_LAX(fls_num, Config, iterations, float('inf'), 0, 0, delta,
                                           fresh_rate=fresh_rate, wirte_traj=wirte_traj)
            print(f"Total Collision: {collisions}")