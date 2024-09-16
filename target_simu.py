import matplotlib as mpl

from FLS import FLS
from FlightPattern import FlightPattern
from Controller import Controller
from util import *
from config import Config
# matplotlib.use('TkAgg')
mpl.use('macosx')

params = {
    'gamma_Acc': 0.25, 'd_v_0': 1.0, 'l_Acc': 2.5,
    'gamma_z': 0.25, 'd_z_0': 1.25, 'a_z': 1.0, 'L_z_2': 1.75,
    'gamma_Ali': 0.4, 'd_Ali_0': 1.0, 'l_Ali': 2.5, 'alpha_Ali': 1.0,
    'gamma_Att': 0.25, 'd_Att_0': 1.25, 'l_Att': 2.75, 'alpha_Att': 1.0,
    'gamma_perp': 0.5, 'gamma_parallel': 0.5, 'sigma_z': 1,
    'gamma_w': 1.2, 'e_w1': 1.25, 'e_w2': 0.0, 'l_w': 2.5,
    'center': np.array([0, 0, 30]), 'radius': 5, 'dist_to_opening': 1, 'normal_vector': np.array([0, 1, 1]),
    'gamma_Dest_h': 2, 'gamma_Dest_v': 0.4, 'l_Dest': 3, 'alfa_Dest_d': 2, 'alfa_Dest_v': 1,
    'v_Dest': 0.7,
    'time_step': 1 / 20, 'fls_size': 0.25,
    'path_policy': 1
}


# np.random.seed(42)

def run_LAX(fls_num, config, c_step, speed_error, heading_error, delta, show_animation=True, init_formation=None, max_step=float('inf')):
    normal_vector = config.normal_vector/np.linalg.norm(config.normal_vector)
    flight_pattern = FlightPattern(config.center, config.radius, config.dist_to_opening, config.v_Dest, 10, config.time_step, normal_vector)
    if init_formation is None:
        init_formation = generate_points(-4, 4, -4, 4, 8, 12, fls_num, 2 * config.fls_size)

    # Initialize swarm with flss
    flss = [FLS(i, init_formation[i], [np.random.uniform(0, 2 * np.pi), np.random.uniform(0, 2 * np.pi)], 1.5,
                config.time_step, [1.5, 0], [1, 1], slot_ID=i) for i in range(fls_num)]

    ctrller = Controller(flss, flight_pattern, config.time_step, delta)

    # ctrller.predict_slots(config.v_Dest)

    def update_control(show_animation=False, axis=None):
        # ctrller.update_FLS_swarm(params)
        end_flag = ctrller.update_FLSs_linear_fp(params, speed_error, heading_error, c_step)
        # positions = swarm.get_positions()

        if show_animation:
            paths = ctrller.get_paths()
            slots = ctrller.get_slots()

            axis.cla()
            # draw_cylinder(axis, config.center, config.radius, 20)
            for i, path in enumerate(paths):
                path = np.array(path)
                axis.plot(path[:, 0], path[:, 1], path[:, 2], linewidth=2, alpha=0.8)  # Draw the path

                if ctrller.flss[i].state == StateTypes.SYNC:
                    axis.scatter(path[-1, 0], path[-1, 1], path[-1, 2], s=20, c='g')
                elif i == 0:
                    axis.scatter(path[-1, 0], path[-1, 1], path[-1, 2], s=20, c='r')
                else:
                    axis.scatter(path[-1, 0], path[-1, 1], path[-1, 2], s=20, c='b')  # Draw FLSs

            axis.scatter(slots[1:, 0], slots[1:, 1], slots[1:, 2], c='c', marker='o')
            axis.scatter(slots[0, 0], slots[0, 1], slots[0, 2], c='y', marker='o')

            starting_point = config.center
            end_point = starting_point + config.normal_vector
            x_values = [starting_point[0], end_point[0]]
            y_values = [starting_point[1], end_point[1]]
            z_values = [starting_point[2], end_point[2]]
            ax.plot(x_values, y_values, z_values, label='Vector', color='blue')

            axis.set_xlim(-config.radius, config.radius)
            axis.set_ylim(-config.radius, config.radius)
            axis.set_zlim(0, (config.center[2] + config.dist_to_opening) * 1.1)
            axis.set_xlabel('X')
            axis.set_ylabel('Y')
            axis.set_zlabel('Z')
            axis.set_aspect('equal', adjustable='box')

        # print(f"Dist To Destination: {np.linalg.norm(slots[0] - positions[0]):.2f}")

        # colliding_counter = get_collisions(positions, fls_size)

        # metrics = ctrller.get_behavior()
        # print(f"D: {metrics[0]:.2f}, P: {metrics[1]:.2f}, M: {metrics[2]:.2f}, Collisions: {colliding_counter}")
        return end_flag

    if show_animation:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        time_lable = fig.text(0.1, 0.9, f"Time: {0:.2f}", fontsize=12, color='k')

    step = 0
    end_flag = False
    while not end_flag:
        step += 1

        if step >= max_step:
            step = float('inf')
            break

        if show_animation:
            end_flag = update_control(True, ax)

            time_lable.set_text(f"Time: {step * config.time_step:.2f}")
            # print(flss[0].velocity, f"Time: {step * config.time_step:.2f}")

            plt.draw()
            plt.pause(config.time_step)
        else:
            end_flag = update_control()

    if show_animation:
        plt.show()

    return step


if __name__ == "__main__":

    init_formations = np.array([[5, 0, 10]])
    fls_num = 1
    # deltas = [0.05, 0.1, 0.15, 0.2]
    # c_steps = [1, 10, 100]
    # speed_errors = np.array([
    #     [-0.2, 0], [-0.1, 0], [-0.05, 0], [-0.01, 0],
    #     [0, 0], [0.01, 0], [0.05, 0], [0.1, 0], [0.2, 0]
    # ])

    deltas = [0.1]
    c_steps = [float('inf')]
    errors = np.array([[0, 0]])

    show_animation = True

    for delta in deltas:
        for init_coords, init_name in zip(init_formations, ['Far Initial Point', 'Close Initial Point']):

            opt_step = run_LAX(fls_num, Config, float('inf'), 0, 0, delta, show_animation=show_animation, init_formation=[init_coords])
            print(f"OPT Step: {opt_step}")

            # plot_data = [[opt_step * config.time_step for _ in errors]]
            # for c in c_steps:
            #     c_data = []
            #     for error in errors:
            #         speed_error, heading_error = error
            #         step = run_LAX(fls_num, c, speed_error, heading_error, delta, show_animation=False,
            #                        init_formation=[init_coords], max_step=10000)
            #         print(f"{init_name}, Delta={delta:.2f}, C={c}, Speed error: {error}, Step: {step}")
            #         c_data.append(step * config.time_step)
            #
            #     plot_data.append(c_data)
            # draw_line_chart(plot_data, errors[:, 0], ['OPT + [f"C={c:.0f}" for c in c_steps],
            #                 f"{init_name}, Delta={delta:.2f}", "Speed Error (%)", 'Time Cost (Second)',
            #                 f"./results/{init_name}_Delta={delta:.2f}_s_single.png")

            # heading_errors = np.array([
            #     [0, -np.radians(20)], [0, -np.radians(10)], [0, -np.radians(5)], [0, -np.radians(1)],
            #     [0, np.radians(0)], [0, np.radians(1)], [0, np.radians(5)], [0, np.radians(10)], [0, np.radians(20)]
            #      ])
            # heading_errors_degree = np.array([-20, -10, -5, -1, 0, 1, 5, 10, 20])
            # plot_data = [[opt_step * config.time_step for _ in heading_errors]]
            # for c in c_steps:
            #     c_data = []
            #     for error in heading_errors:
            #         speed_error, heading_error = error
            #         step = run_LAX(fls_num, c, speed_error, heading_error, delta, show_animation=False,
            #                        init_formation=[init_coords], max_step=10000)
            #         print(f"{init_name}, Delta={delta:.2f}, C={c}, Heading error: {error}, Step: {step}")
            #         c_data.append(step * config.time_step)
            #
            #     plot_data.append(c_data)
            # draw_line_chart(plot_data, heading_errors_degree, ['OPT + [f"C={c:.0f}" for c in c_steps],
            #                 f"{init_name}, Delta={delta:.2f}", "Heading Error (Degree)", 'Time Cost (Second)',
            #                 f"./results/{init_name}_Delta={delta:.2f}_h_single.png")
