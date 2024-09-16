import itertools
import math
import os
import sys
import numpy as np

params = {
    'gamma_Acc': 0.25,
    'd_v_0': 1.0,
    'l_Acc': 2.5,
    'gamma_z': 0.25,
    'd_z_0': 1.25,
    'a_z': 1.0,
    'L_z_2': 1.75,
    'gamma_Ali': 0.4,
    'd_Ali_0': 1.0,
    'l_Ali': 2.5,
    'alpha_Ali': 1.0,
    'gamma_Att': 0.25,
    'd_Att_0': 1.25,
    'l_Att': 2.75,
    'alpha_Att': 1.0,
    'gamma_perp': 0.5,
    'gamma_parallel': 0.5,
    'sigma_z': 1,
    'gamma_w': 1.2,
    'e_w1': 1.25,
    'e_w2': 0.0,
    'l_w': 2.5,
    'center': "[0.0, 0.0, 0.0]",
    'radius': 1,
    'dist_to_opening': 0.2,
    'normal_vector': "[0.0, 0.0, 1.0]",
    'gamma_Dest_h': 2,
    'gamma_Dest_v': 0.4,
    'l_Dest': 3,
    'alfa_Dest_d': 2,
    'alfa_Dest_v': 1,
    'v_Dest': 0.7,
    'slot_num': 25,
    'time_step': 1 / 30,
    'fls_size': 0.15 / 2,
    'path_policy': 1,
    'space': 2,
    'init_altitude': 0.1,
    'Q': 1,
    'speed_range': "[1.5, 0]",
    'acc_range': "[1.0, 1.0]",
    'fresh_rate': 0,
    'wirte_traj': 0,
    'shape': "'test'",
    "results_path": "'/proj/nova-PG0/shuqin/collision/results_salt/'",
    "figrue_path": "'/proj/NOVA/shuqin/collision/frames_OPT/'",
    "inter_arrival_time": (2 * math.pi * 1)/(25 * 0.7)
    }

general_props = [
    {
        "keys": ["shape"],
        # "values": ["'chess_408'", "'palm_725'", "'kangaroo_972'", "'dragon_1147'", "'skateboard_1372'"],
        "values": ["'kangaroo_972'"],
    },
    # {
    #     "keys": ["fresh_rate"],
    #     "values": [1],
    # },
    {
        "keys": ["Q", "speed_range", "slot_num", "radius"],
        "values": [

            # {"Q": 1, "speed_range": "[1.5, 0]", "slot_num": 25, "radius": 1},
            # {"Q": 1, "speed_range": "[1.5, 0]", "slot_num": 50, "radius": 2},
            # {"Q": 1, "speed_range": "[1.5, 0]", "slot_num": 100, "radius": 4},
            # {"Q": 1, "speed_range": "[3, 0]", "slot_num": 100, "radius": 4},
            # {"Q": 1, "speed_range": "[15, 0]", "slot_num": 100, "radius": 4},

            # {"Q": 1, "speed_range": "[1.5, 0]", "slot_num": 25, "radius": 1},
            # {"Q": 1, "speed_range": "[3, 0]", "slot_num": 25, "radius": 1},
            {"Q": 1, "speed_range": "[15, 0]", "slot_num": 25, "radius": 1},
            #
            # {"Q": 3, "speed_range": "[1.5, 0]", "slot_num": 25, "radius": 1},
            # {"Q": 3, "speed_range": "[1.5, 0]", "slot_num": 50, "radius": 2},
            # {"Q": 3, "speed_range": "[1.5, 0]", "slot_num": 100, "radius": 4},
            #
            # {"Q": 3, "speed_range": "[1.5, 0]", "slot_num": 25, "radius": 1},
            # {"Q": 3, "speed_range": "[3, 0]", "slot_num": 25, "radius": 1},
            # {"Q": 3, "speed_range": "[15, 0]", "slot_num": 25, "radius": 1},
            #
            # {"Q": 10, "speed_range": "[1.5, 0]", "slot_num": 25, "radius": 1},
            # {"Q": 10, "speed_range": "[1.5, 0]", "slot_num": 50, "radius": 2},
            # {"Q": 10, "speed_range": "[1.5, 0]", "slot_num": 100, "radius": 4},
            # #
            # {"Q": 10, "speed_range": "[1.5, 0]", "slot_num": 25, "radius": 1},
            # {"Q": 10, "speed_range": "[3, 0]", "slot_num": 25, "radius": 1},
            # {"Q": 10, "speed_range": "[15, 0]", "slot_num": 25, "radius": 1},
        ],
    },
    {
        "keys": ["path_policy"],
        "values": [1],
    },
    {
        "keys": ["v_Dest"],
        "values": [0.7],
    },

    # {
    #     "keys": ["shape", "Q", "speed_range", "slot_num", "radius"],
    #     "values": [
    #         # {"shape": "'chess_408'", "Q": 1, "speed_range": "[1.5, 0]", "slot_num": 25, "radius": 1},
    #         {"shape": "'kangaroo_972'", "Q": 1, "speed_range": "[3, 0]", "slot_num": 100, "radius": 4},
    #         {"shape": "'kangaroo_972'", "Q": 1, "speed_range": "[15, 0]", "slot_num": 100, "radius": 4},
    #
    #         {"shape": "'palm_725'", "Q": 1, "speed_range": "[3, 0]", "slot_num": 100, "radius": 4},
    #         {"shape": "'palm_725'", "Q": 1, "speed_range": "[15, 0]", "slot_num": 100, "radius": 4},
    #
    #         {"shape": "'skateboard_1372'", "Q": 1, "speed_range": "[3, 0]", "slot_num": 100, "radius": 4},
    #         {"shape": "'skateboard_1372'", "Q": 1, "speed_range": "[15, 0]", "slot_num": 100, "radius": 4},
    #     ],
    # },
]

if __name__ == '__main__':
    if not os.path.exists('experiments'):
        os.makedirs('experiments', exist_ok=True)

    file_name = "config"
    class_name = "Config"
    props = general_props
    def_conf = params
    # if len(sys.argv) > 1:
    #     file_name = "test_config"
    #     class_name = "TestConfig"
    #     props = test_props
    #     def_conf = def_test_conf

    props_values = [p["values"] for p in props]
    print(props_values)
    combinations = list(itertools.product(*props_values))
    print(len(combinations))

    for j in range(len(combinations)):
        c = combinations[j]
        conf = def_conf.copy()
        for i in range(len(c)):
            for k in props[i]["keys"]:
                if isinstance(c[i], dict):
                    conf[k] = c[i][k]
                else:
                    conf[k] = c[i]
        with open(f'experiments/{file_name}{j}.py', 'w') as f:
            f.write(f'class {class_name}:\n')
            for key, val in conf.items():
                f.write(f'    {key} = {val}\n')
