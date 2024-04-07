#! /bin/bash


cd /home/usr/ros2_ws/src/two_wheels_core/firmware/esp32dev-microros
pio lib install # Install dependencies
pio run # Build the firmware
pio run --target upload # Flash the firmware