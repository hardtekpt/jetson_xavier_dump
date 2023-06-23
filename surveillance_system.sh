#!/bin/bash

## change these to whatever you actually need
sim="sleep 2s; cd catkin_ws; roslaunch drone_sim_bringup real.launch; bash"
planner="sleep 20s; cd catkin_ws; source ../catkin_ws_python/devel/setup.bash; roslaunch mission_planner mission_planner.launch; bash"
manager="sleep 15s; cd catkin_ws_python; source devel/setup.bash; roslaunch network_manager wsn_network_manager.launch; bash"

## Modify terminator's config
sed -i.bak "s#SIM#$sim#; s#PLANNER#$planner#; s#MANAGER#$manager#;" ~/.config/terminator/config

## Launch a terminator instance using the new layout
terminator -l sur_system

## Return the original config file
mv ~/.config/terminator/config.bak ~/.config/terminator/config