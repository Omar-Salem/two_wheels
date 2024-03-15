#!/bin/bash
colcon build
source install/setup.bash
ros2 launch two_wheels sim_two_wheels.launch.py