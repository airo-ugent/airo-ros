# Moveit Move Group Interface (MGI) Bridge.

In this package a Node is created that serves as a bridge to the Moveit Move Group Interface (MGI) by exposing a number of services (messages are defined in the `airo_moveit_mgi_bridge_msgs` package).

## How to use

- (build & source)
- TODO:  launch file instructions

## Rationale
[MGI](https://moveit.picknik.ai/foxy/doc/move_group_interface/move_group_interface_tutorial.html) is the high-level C++ wrapper around the central Move Group node in Moveit, and is the recommended way to interact with Moveit.

As of now, there are no python bindings available for the MGI. This bridge provides a convenient interface (read: python interface) to some MGI features. The alternative is to address the low-level actions etc. of the Move Group node manually from python. This allows for greater flexibility (e.g. provide constraints to a planning request, ...) but this is not documented within the ROS tutorials or on other fora, which makes it a little harder to do (especially given the already steep ROS / Moveit learning curves).

## Available Services
- `MoveToPose`: (attempt to) Move the specified Frame to the specified Pose. If no frame is specified, Moveit's default frame (the current end-effector frame) is used.
- `GetFramePose`: get the Pose of the specified Frame.


## How to add a new service
- define a new service in the `/srv` folder of the msgs packages
- add the service to the `cmakelist.txt` in the msgs package
- create a subscription and implement the callback in `src/mgi_bridge.cpp`

cf the [ROS tutorials](https://docs.ros.org/en/galactic/Tutorials.html) for more information on creating ros services, ...
