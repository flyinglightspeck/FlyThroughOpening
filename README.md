# FlyThroughOpening
An implementation of 3 algorithms to fly a swarm of drones through an opening.  The first technique, OPT, is time optimal but results in many collisions near the opening.  It motivates a multi-slot circular flight pattern that flies the drones through the opening one at a time.  Our implementation includes two algorithms for the drones to occupy their assigned slots, Shortest Distance (SD) and Fastest Rendezvous Time (FRT).

Authors:  Shuqin Zhu (shuqinzh@usc.edu) and Shahram Ghandeharizadeh (shahram@usc.edu)

## A Demonstration
Click the image below to watch the YouTube clip of the implementation using 5 Crazyflies with Vicon.

[![A Demonstration](https://github.com/flyinglightspeck/CircularFlightPattern/blob/main/simulation.png)](https://www.youtube.com/watch?v=_hcwj3lhY5g)


## Acknowledgments
This research was supported in part by the NSF grants IIS-2232382 and CMMI-2425754.
