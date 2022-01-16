#!/bin/sh
xterm -hold -e  "roslaunch my_robot world.launch " &
pid1=$!

sleep 7

xterm -hold -e  "roslaunch my_robot amcl.launch" &
pid2=$!

sleep 10

xterm -hold -e  "rosrun pick_objects pick_objects_node" &
pid3=$!

wait $pid1 $pid2 $pid3