#!/bin/bash
export ROS_MASTER_URI=http://192.168.43.200:11311
export DISPLAY=:0
source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash

exec "$@"
