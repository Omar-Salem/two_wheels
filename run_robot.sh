#! /bin/bash

mkdir -p ros2_ws/src
cd $_
git clone https://github.com/Slamtec/sllidar_ros2.git
git clone https://github.com/Omar-Salem/two_wheels.git

mv two_wheels/* .
rm -rf two_wheels
cd ..
rm -rf build/ install/ log/
colcon build
source install/setup.bash && ros2 launch two_wheels_core two_wheels.launch.py