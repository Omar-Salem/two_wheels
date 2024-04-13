### Validate URDF file

```check_urdf /PATH/TO/URDF```

## Build and run simulation

```bash
cd ros2_ws/
rm -rf build/ install/ log/
colcon build && source install/setup.bash && ros2 launch two_wheels_core sim_two_wheels.launch.py
````

## Build and run real

[//]: # (### Log into on Pi)

[//]: # (```bash)

[//]: # (ssh omar.salem@192.168.1.35)

[//]: # (```)

### Prepare docker on Pi (_first time/on code change_)
```bash
HOST=omar.salem@192.168.1.35
scp prepare_docker.sh $HOST:/tmp/ && ssh -t $HOST "sudo -s bash /tmp/prepare_docker.sh"
```

### Run microros

```bash
ssh omar.salem@192.168.1.35
docker run --rm --privileged -it -v ~/Volumes:/home/usr/ humble
#contents of run_microros.sh
```

### Run robot

```bash
ssh omar.salem@192.168.1.35
docker exec -it $(docker container ls  | grep 'humble' | awk '{print $1}') bash
#contents of run_robot.sh
```

### Test
```bash
ssh omar.salem@192.168.1.35
docker run --rm --privileged -it -v ~/Volumes:/home/usr/ humble
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "
  linear:
    x: 0.5"

````

### Debug

```bash

ls /dev/tty* | grep USB #sanity check to see if usb devices are reachable
ros2 topic list | grep motors #sanity check to see if /two_wheels/motors_* topics exist

source ~/microros_ws/install/local_setup.bash
ros2 topic echo /two_wheels/motors_state #sanity check to see if topics emit

ros2 topic pub -r 10 /two_wheels/motors_cmd two_wheels_interfaces/msg/MotorsOdom "m1:
  velocity: 1.5
m2:
  velocity: 1.50"
````
