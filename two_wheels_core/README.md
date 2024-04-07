### Validate URDF file

```check_urdf /PATH/TO/URDF```

### Build and run simulation

```bash
cd ros2_ws/
rm -rf build/ install/ log/
colcon build && source install/setup.bash && ros2 launch two_wheels_core sim_two_wheels.launch.py
````

### Log into on Pi
```bash
ssh omar.salem@192.168.1.35
```

### Prepare docker on Pi (first time/on repo change)
```bash
sudo rm -rf ~/Volumes
mkdir -p ~/Volumes/ros2_ws/src
cd ~/Volumes
mkdir microros_ws
cd $_
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
cd ~/Volumes/ros2_ws/src
git clone https://github.com/Omar-Salem/two_wheels.git
git clone https://github.com/Slamtec/sllidar_ros2.git
pip install -U colcon-common-extensions
source sllidar_ros2/scripts/create_udev_rules.sh
mv two_wheels/* ~/Volumes/ros2_ws/src/
#mv two_wheels/Dockerfile ~/Volumes/ros2_ws/src/  # TODO needed?
rm -rf two_wheels
docker system prune --volumes --all -f
docker build -t humble .
```

### Build and run real

```bash
ls /dev/tty* | grep USB #sanity check to see if usb devices are reachable
docker run --rm --privileged -it -v ~/Volumes:/home/usr/ humble

ros2 topic list | grep motors #sanity check to see if topics exist

ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "
  linear:
    x: 1.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0"
````
