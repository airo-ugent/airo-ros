# based on
cd $COLCON_WS

source /opt/ros/$ROS_DISTRO/setup.bash
# clone the driver repo
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/robots/UR/Universal_Robots_ROS2_Driver

# clone all non-released dependencies
vcs import src/robots/UR --skip-existing --input src/robots/UR/UR_ROS2_galactic_non-released.repos

# install binaries dependencies
apt-get install -y libyaml-cpp-dev
apt-get install ros-$ROS_DISTRO-ros2-control
apt-get install ros-$ROS_DISTRO-ros2-controllers

# install ros
rosdep install --ignore-src --from-paths src -y -r

# build all local packages
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --symlink-install
