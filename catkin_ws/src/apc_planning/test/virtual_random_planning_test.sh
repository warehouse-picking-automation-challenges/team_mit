#!/bin/bash
cd $APC_BASE/input
for i in `seq 1 100`;
do
    rosrun apc_planning interface_generator.py; cp apc.json /tmp/apc_$(date +%Y%m%d_%H%M%S).json; time rosrun apc_planning heuristic.py -s -v --jfilename apc.json;
    #rosrun apc_planning interface_generator.py; cp apc.json /tmp/apc_$(date +%Y%m%d_%H%M%S).json; time rosrun apc_planning heuristic.py -s -v --jfilename apc.json > /tmp/$(date +%Y%m%d_%H%M%S).log;
done
