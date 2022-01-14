#!bin/bash

cd $COLCON_WS

# clone the driver repo
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/robots/UR/Universal_Robots_ROS2_Driver

# clone all non-released dependencies
vcs import src/robots/UR --skip-existing --input src/robots/UR/UR_ROS2_galactic_non-released.repos

# install binaries dependencies
apt-get update
apt-get install -y libyaml-cpp-dev
apt-get install -y ros-$ROS_DISTRO-ros2-control
apt-get install -y ros-$ROS_DISTRO-ros2-controllers
