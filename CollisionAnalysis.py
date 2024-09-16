
from util import *
from config_local import Config

if __name__ == "__main__":

    directory = "./results/log_salt"
    display_cell_size = Config.fls_size
    txt_files = [f for f in os.listdir(directory) if f.endswith('.txt')]
    info_list = []
    makespan_list = []

    for simulation_name in txt_files:
        file_path = f'{directory}/{simulation_name}'

        config_settings = simulation_name.split('_')
        Config.Q = int((config_settings[2].split('Q'))[1])
        Config.shape = f"{config_settings[0]}_{config_settings[1]}"

        # Config.v_Dest = float((((config_settings[6].split('D'))[1]).split('.txt'))[0])

        makespan = extract_makespan(file_path)
        makespan_list.append([config_settings[0], Config.Q, (config_settings[3].split('S'))[1], (config_settings[5].split('.'))[0], makespan])

        draw_collision(file_path, f'{directory}/collisions', Config,
                       f'{directory}/collisions/collisions.txt',
                       f'{directory}/collisions/speeds.csv', simulation_name)
        speed, dist = analyse_collision(Config, f"./assets/{Config.shape}.xyz", file_path, f"{directory}/collisions/{(simulation_name.split('.'))[0][:-4]}")
        info_list.append([config_settings[0], Config.Q, (config_settings[3].split('S'))[1], (config_settings[5].split('.'))[0], speed, dist])

        print(f"{simulation_name} Finished")

    makespan_log = f'{directory}/makespan.csv'
    with open(makespan_log, mode='w', newline='') as file:
        writer = csv.writer(file)

        # Write the string and list as a new row
        for row in makespan_list:
            writer.writerow(row)

    with open(f'{directory}/collision_info.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        # Write the string and list as a new row
        for row in info_list:
            writer.writerow(row)
