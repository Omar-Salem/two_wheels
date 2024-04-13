FROM ros:humble
ARG USER=user
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update
#Install ROS
RUN apt-get install -y ros-${ROS_DISTRO}-launch-xml
RUN apt-get install -y ros-${ROS_DISTRO}-joint-state-publisher
RUN apt-get install -y ros-${ROS_DISTRO}-robot-state-publisher
RUN apt-get install -y ros-${ROS_DISTRO}-xacro
RUN apt-get install -y ros-${ROS_DISTRO}-ros2-control
RUN sudo apt-get install -y ros-${ROS_DISTRO}-rviz2
RUN apt-get install -y ros-${ROS_DISTRO}-ros2-controllers
RUN apt-get install -y ros-${ROS_DISTRO}-controller-manager
RUN apt-get install -y ros-${ROS_DISTRO}-tf-transformations
RUN apt-get install -y ros-${ROS_DISTRO}-tf2-ros
RUN apt-get install -y ros-${ROS_DISTRO}-tf2-tools
RUN apt-get install -y ros-dev-tools

RUN apt-get install -y ros-${ROS_DISTRO}-robot-localization
RUN apt-get install -y ros-${ROS_DISTRO}-navigation2
RUN apt-get install -y ros-${ROS_DISTRO}-slam-toolbox
RUN apt-get install -y ros-${ROS_DISTRO}-nav2-bringup
RUN apt-get install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp


RUN apt-get install -y ros-${ROS_DISTRO}-angles
#RUN apt-get install -y ros-${ROS_DISTRO}-rosbag2
#RUN apt-get install -y ros-${ROS_DISTRO}-behaviortree-cpp-v3
#RUN apt-get install -y ros-${ROS_DISTRO}-image-tools
#RUN apt-get install -y ros-${ROS_DISTRO}-cartographer
#RUN apt-get install -y ros-${ROS_DISTRO}-twist-mux
#RUN apt-get install -y ros-${ROS_DISTRO}-mqtt-client

RUN apt install -y python3-colcon-common-extensions
RUN apt install -y vim
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
RUN apt-get install -y git
RUN apt-get install -y usbutils
RUN apt-get install -y udev


#Install microros
RUN sudo apt install -y python3-rosdep2
RUN sudo apt-get install -y python3-pip

#Install platformio
RUN curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
RUN apt-get install -y python3.10-venv
RUN python3 get-platformio.py

#Run commands on startup
#CMD sh /home/usr/ros2_ws/src/run_microros.sh
#CMD sh /home/usr/ros2_ws/src/run_microros.sh & /home/usr/ros2_ws/src/run_robot.sh