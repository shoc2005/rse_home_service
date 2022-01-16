#!/bin/sh
xterm -hold -e  "roslaunch my_robot world.launch " &
pid1=$!

sleep 7

xterm  -hold -e  "roslaunch my_robot gmapping_test.launch" &
pid2=$!

wait $pid1 $pid2