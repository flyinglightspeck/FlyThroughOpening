#!/bin/bash

source cloudlab_vars.sh

now=$(date +%d-%b-%H_%M_%S)


job_num=$(( (sets_of_configs + num_of_total_servers - 1) / num_of_total_servers ))

for (( i=1; i<num_of_total_servers; i++ )); do
    server_addr=${USERNAME}@node-$i.${HOSTNAME}
    echo "STARTING $i"
    ssh -oStrictHostKeyChecking=no -f "${server_addr}" "ulimit -n 99999 && cd Dronevision-FlightPattern && sudo nohup python3 OPT_shape_to_opening.py $i "$job_num" > my.log 2>&1 &" &
done
ulimit -n 99999 && sudo python3 OPT_shape_to_opening.py 0 "$job_num" > my.log 2>&1 &
