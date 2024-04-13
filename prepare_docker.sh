#! /bin/bash

sudo rm -rf ~/Volumes
wget https://raw.githubusercontent.com/Omar-Salem/two_wheels/master/Dockerfile

# grant read and write permissions to serial devices
sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB1

docker system prune --volumes --all -f
docker build -t humble .