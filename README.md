# FlyThroughOpening
An implementation of 3 algorithms to fly a swarm of drones through an opening.  The first technique, OPT, is time optimal but results in many collisions near the opening.  It motivates a multi-slot circular flight pattern that flies the drones through the opening one at a time.  Our implementation includes two algorithms for the drones to occupy their assigned slots, Shortest Distance (SD) and Fastest Rendezvous Time (FRT).

Authors:  Shuqin Zhu (shuqinzh@usc.edu) and Shahram Ghandeharizadeh (shahram@usc.edu)

## A Demonstration
Click the image below to watch the YouTube clip of the implementation using 5 Crazyflies with Vicon.

[![A Demonstration](https://github.com/flyinglightspeck/CircularFlightPattern/blob/main/simulation.png)](https://www.youtube.com/watch?v=_hcwj3lhY5g)

## Requirement
Python 3.9 with numpy, matplotlib, pandas, scipy.

## Instructions to run
The simulator runs either OPT, the Shortest Distance (SD) or the Fastest Rendezvous Time (FRT) algorithm.
To run OPT, execute the script OPT_shape_to_opening.py, issue the command `python OPT_shape_to_opening.py` or `python3 OPT_shape_to_opening.py`.
Adjust the value of path_policy in config.py to run FRT (`path_policy=0`) or SD (`path_policy=1`).
Next, run the simulator, execute the script simu_with_shapes.py, i.e., issue the command `python simu_with_shapes.py` or `python3 simu_with_shapes.py`.
The initial formation shape of the FLS can be selected by changing `shape` in config_local.py. One can select a name from existing files in `./assets/` directory.

## Acknowledgments
This research was supported in part by the NSF grants IIS-2232382 and CMMI-2425754.
We gratefully acknowledge CloudBank and CloudLab for the use of their resources to enable an evaluation of the alternative algorithms.
