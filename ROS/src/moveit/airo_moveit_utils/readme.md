# Airo Moveit Utils

a ROS2 package with some utitily executables for use with ROS2 / Moveit2


## Available utils

- Transform Streamer: prints the transform from a frame to a reference frame to the CLI. Both frames need to be in the tf2 tree. Can be used e.g. to obtain some poses of interest for later replay or to check callibrations etc.
`ros2 run airo_moveit_utils transform_streamer --ros-args -p reference_frame:=base_link -p frame:=tool0`


## Adding new utils
- create a .cpp file and implement your node.
- in the cmake file: find required dependencies, add the executable and install to make it discoverable by ROS2.
- document the executable ^
