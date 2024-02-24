### Validate URDF file
```check_urdf /PATH/TO/URDF```

### Prepare docker on Pi
```
ssh omar.salem@192.168.1.35
mkdir -p ~/Volumes/ros2_ws/src
cd $_ 
git clone https://github.com/Omar-Salem/two_wheels.git
cd two_wheels
docker build -t humble .
docker run --device=/dev/ttyUSB0 -it -v ~/Volumes:/home/usr/ humble
cd /home/usr/ros2_ws && rm -rf build/ install/ log/ && colcon build --packages-select two_wheels && source install/setup.bash && ros2 launch two_wheels two_wheels.launch.py
```

### Compile and upload arduino sketch
```
cd ros2_ws
SKETCH=src/two_wheels/firmware/arduino/arduino.ino
ARDUINO=arduino:avr:uno
PORT=/dev/ttyUSB0
arduino-cli compile -b ${ARDUINO} ${SKETCH}
arduino-cli upload -p ${PORT} -b ${ARDUINO} ${SKETCH}
```

### Build and run

```
rm -rf build/ install/ log/
colcon build --packages-select two_wheels
source install/setup.bash
ros2 launch two_wheels sim_two_wheels.launch.py
```