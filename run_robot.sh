#! /bin/bash

cd /home/usr/ros2_ws/src
rm -rf build/ install/ log/
colcon build
source install/setup.bash && ros2 launch two_wheels_core two_wheels.launch.py