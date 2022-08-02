#!/usr/bin/env bash

source /opt/ros/overlay_ws/devel/setup.bash
export path="/opt/ros/overlay_ws/src/local_sensing_intest/bags/maze4/2021-11-24_15h15/"
export parent=".."
ls $path$parent
export file="*-no-map.bag"
export bag_file=$(find $path$file)

roslaunch local_sensing_intest simulation_replay_set_init.launch

roslaunch local_sensing_intest simulation_replay_slam.launch