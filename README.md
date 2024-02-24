``check_urdf /PATH/TO/URDF``

``docker build -t humble .``

``mkdir ~/Volumes && cd Volumes && mkdir ros2_ws && cd ros2_ws && mkdir src && cd src && git clone https://github.com/Omar-Salem/two_wheels.git``

``docker run -it -v ~/Volumes:/home/usr/ humble``

```
cd ros2_ws
SKETCH=src/two_wheels/firmware/arduino/arduino.ino
ARDUINO=arduino:avr:uno
PORT=/dev/ttyUSB0
arduino-cli compile -b ${ARDUINO} ${SKETCH}
arduino-cli upload -p ${PORT} -b ${ARDUINO} ${SKETCH}
```

`rm -rf build/ install/ log/ && colcon build --packages-select two_wheels && source install/setup.bash && ros2 launch two_wheels sim_two_wheels.launch.py`