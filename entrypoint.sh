#! /bin/bash

set -e  #enable error signal

#source the ROS installation
source /opt/ros/noetic/setup.bash

echo "Provided arguments: $@"

exec $@