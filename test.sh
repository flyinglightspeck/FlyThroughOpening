#!/bin/bash

source cloudlab_vars.sh

for (( i=1; i<num_of_total_servers; i++ )); do
    job_num=$(( (sets_of_configs + num_of_total_servers - 1) / num_of_total_servers ))

    python3 simu_with_shapes.py $i "$job_num" 1
done
python3 simu_with_shapes.py 0 "$job_num" 1