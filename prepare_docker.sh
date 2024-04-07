#! /bin/bash

sudo rm -rf ~/Volumes
mkdir -p ~/Volumes/ros2_ws/src
cd ~/Volumes
mkdir microros_ws
cd $_
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
cd ~/Volumes/ros2_ws/src
git clone https://github.com/Omar-Salem/two_wheels.git
git clone https://github.com/Slamtec/sllidar_ros2.git

# grant read and write permissions of the serial device
sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB1
mv two_wheels/* ~/Volumes/ros2_ws/src/
rm -rf two_wheels
docker system prune --volumes --all -f
docker build -t humble .