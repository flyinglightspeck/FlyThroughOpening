import csv
import statistics

from FlightPattern import FlightPattern
from util import *
from config_local import Config

def analyse_collision_SD(config, shape_file, file_path):
    init_formation = load_shape(shape_file, shrink_min_dist=config.Q * (2 * config.fls_size),
                                shift_to_center=True, shift_bottom=0)
    init_formation = init_formation[init_formation[:, 2].argsort()[::-1]]

    normal_vector = config.normal_vector / np.linalg.norm(config.normal_vector)

    config.center[2] = np.max(init_formation[:, 2]) + max(np.max(init_formation[:, 2]) * 0.2,
                                                          (config.v_Dest ** 2) / (
                                                                  2 * config.acc_range[0]))
    flight_pattern = FlightPattern(config.center, config.radius, config.dist_to_opening,
                                   config.v_Dest,
                                   config.slot_num, config.time_step, normal_vector)

    exit_vec = flight_pattern.slots[0] - flight_pattern.get_slot_coord(0, -1)
    exit_vec = exit_vec / np.linalg.norm(exit_vec)

    exit_time = math.sqrt(2 * config.dist_to_opening / config.acc_range[0]) * 1.42

    opening_pos = flight_pattern.slots[0] + exit_time * config.v_Dest * exit_vec + np.array(
        config.normal_vector) * config.dist_to_opening

    speeds_2 = []
    speeds_3 = []
    collision_angle = {}
    dist_2 = []
    dist_3 = []

    with open(file_path, 'r') as file:
        lines = file.readlines()  # Skip the first two lines

        for line in lines[3:]:
            line = line.strip()
            if not line:
                continue

            dist = []
            speed = []

            parts = re.split(r'[\[\]\s]+', line)
            all_moving = True

            if len(parts) == 2:  # This line indicates a new timestep
                continue
            else:
                is_continuing = bool(int(parts[1]))
                num_fls = int(parts[2])

                if not is_continuing:
                    flight_pattern_FLS = []
                    for i in range(num_fls):
                        fls_id = int(parts[3 + i * 8])
                        if fls_id == 2 or fls_id == 3:
                            heading = angles_to_vector(
                                [float(item.replace('[', '')) for item in parts[8 + i * 8: 10 + i * 8]])
                            coord = np.array([float(item.replace('[', '')) for item in parts[5 + i * 8: 8 + i * 8]])
                            flight_pattern_FLS.append([coord, heading])

                    for i in range(num_fls):
                        fls_id = int(parts[3 + i * 8])
                        fls_status = int(parts[4 + i * 8])
                        fls_coord = np.array([float(item.replace('[', '')) for item in parts[5 + i * 8: 8 + i * 8]])
                        fls_heading = angles_to_vector(
                            [float(item.replace('[', '')) for item in parts[8 + i * 8: 10 + i * 8]])
                        fls_speed = float(parts[10 + i * 8])

                        # if fls_status != StateTypes.DYN.value:
                        #     all_moving = False

                        if fls_status == StateTypes.DYN.value and flight_pattern_FLS:
                            for coord, heading in flight_pattern_FLS:
                                if np.linalg.norm(fls_coord - coord) < config.fls_size * 2:
                                    angle = get_counter_clockwise_angle(fls_heading, heading,
                                                                        normal=np.array([0, 0, 1]))
                                    if fls_id in collision_angle:
                                        (collision_angle[fls_id]).append(angle)
                                    else:
                                        collision_angle[fls_id] = [angle]
                                    break

                        speed.append(fls_speed)

                        coordinates = [float(item.replace('[', '')) for item in parts[5 + i * 8: 8 + i * 8]]

                        dist.append(distance(opening_pos, coordinates))

                    if all_moving:
                        if num_fls == 2:
                            dist_2.append(statistics.mean(dist))
                            speeds_2.append(statistics.mean(speed))
                        elif num_fls == 3:

                            dist_3.append(statistics.mean(dist))
                            speeds_3.append(statistics.mean(speed))

    result = []
    for data in [dist_2, speeds_2, dist_3, speeds_3]:
        if data:
            result.append(statistics.mean(data))
        else:
            result.append(0)
    return result


directory = "./results/log"
txt_files = [f for f in os.listdir(directory) if f.endswith('.txt')]

data = []
for simulation_name in txt_files:

    display_cell_size = Config.fls_size
    config_settings = simulation_name.split('_')
    Config.Q = int((config_settings[2].split('Q'))[1])
    Config.slot_num = int(config_settings[5])
    Config.radius = (Config.slot_num/25)
    Config.shape = f"{config_settings[0]}_{config_settings[1]}"

    row = [Config.shape, Config.Q, (config_settings[3].split('S'))[1], config_settings[5]]

    # for name in ["SD", "FRT", "one"]:
    for name in ["salt"]:
        # if name != "one":
        #     simu_name = simulation_name[:-8] + simulation_name[-4:]
        # else:
        #     simu_name = simulation_name
        # file_path = f'{directory[:-3]}{name}/{simu_name}'
        simu_name = simulation_name
        file_path = f'{directory}/{simu_name}'

        values = analyse_collision_SD(Config, f"./assets/{Config.shape}.xyz", file_path)
        row.extend(values)
    data.append(row)

with open(f'./results/log/speed_dist.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    # Write the string and list as a new row
    for row in data:
        writer.writerow(row)

