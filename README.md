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

### Docker run

```bash
ssh omar.salem@192.168.1.35 docker run --rm --privileged -it -v ~/Volumes:/home/usr/ humble
```

[//]: # (```bash)

[//]: # (ls /dev/tty* | grep USB #sanity check to see if usb devices are reachable)

[//]: # (docker run --rm --privileged -it -v ~/Volumes:/home/usr/ humble)

[//]: # ()
[//]: # (ros2 topic list | grep motors #sanity check to see if topics exist)

[//]: # ()
[//]: # (ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist ")

[//]: # (  linear:)

[//]: # (    x: 1.0)

[//]: # (    y: 0.0)

[//]: # (    z: 0.0)

[//]: # (  angular:)

[//]: # (    x: 0.0)

[//]: # (    y: 0.0)

[//]: # (    z: 0.0")

[//]: # (````)
