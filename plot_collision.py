import csv

from util import *
from config_local import Config

import matplotlib as mpl

if __name__ == "__main__":
    mpl.use('macosx')

    directory = "./results/log"
    display_cell_size = Config.fls_size
    simulation_name = "chess_408_Q3_S3.0_N_25"
    # simulation_name = "skateboard_1372_Q3_S3.0_N_25"

    file_path = f'{directory}/{simulation_name}.txt'
    ID = 15

    config_settings = simulation_name.split('_')
    Config.Q = int((config_settings[2].split('Q'))[1])
    Config.shape = f"{config_settings[0]}_{config_settings[1]}"

    trajectory, category, speeds = extract_collision_data(file_path, ID)
    involving_FLSs = len(trajectory[0][1])

    plot_collision(trajectory, involving_FLSs, f"./results/log/collisions/{simulation_name}_{ID}_{involving_FLSs}.png", show=True)

    plot_collision_overview(trajectory, involving_FLSs, Config, shape_file=f"./assets/{Config.shape}.xyz",
                            save_name=f"./results/log/collisions/{simulation_name}_{ID}_{involving_FLSs}_overview.png")