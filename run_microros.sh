#! /bin/bash

cd /home/usr/microros_ws
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source /home/usr/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0