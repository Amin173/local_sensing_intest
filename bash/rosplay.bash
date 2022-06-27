#!/usr/bin/env bash

#/bin/bash /home/amin/catkin_ws/src/local_sensing_intest/bash/rosplay.bash

export path="/home/amin/catkin_ws/src/local_sensing_intest/bags/RA-L_data/maze4/2021-11-24_15h15/"
export parent=".."
ls $path$parent
export file="*.bag"
export bag_file=$(find $path$file)

roslaunch local_sensing_intest rosbag_replay_slam.launch
#rosbag info $bag_file
