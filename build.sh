#!bin/bash

cd $COLCON_WS
# prepare build scripts

bash ../prepare_build.sh

# install ros dependencies
source /opt/ros/$ROS_DISTRO/setup.bash
apt-get update
rosdep update
rosdep install --ignore-src --from-paths src -y -r

# build all local packages
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --symlink-install
