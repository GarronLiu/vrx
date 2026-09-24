#!/bin/bash

gnome-terminal -t "record rosbag" -x bash -c "mkdir ~/bagfile;cd ~/bagfile;rosbag record -o identification.bag /mavros/local_position/odom /mavros/rc/override /tracker/solution_time /tracker/track_error;exec bash"

