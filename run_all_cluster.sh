#!/bin/bash

bash gen_conf_cluster.sh
sleep 5

for i in {0..0}
do
   bash start_cluster.sh "$i"
done
