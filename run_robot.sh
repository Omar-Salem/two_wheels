#! /bin/bash

cd /home/usr/ros2_ws/
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash && ros2 launch two_wheels_core two_wheels.launch.py