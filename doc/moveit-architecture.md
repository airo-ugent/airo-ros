# Moveit(2) architecture
a 101 on moveit.

## Moveit layers
the Moveit framework consists of 3 layers in essence:


1. A C++ core of "plugins", aka external libraries such as [OMPL]() for planning or [FCL](https://github.com/flexible-collision-library/fcl) for collision checking.
2. A ROS layer on top of that, with a central `move_group` node that has actions, services and topics for motion planning.
3. An end-user layer: The [Move Group Interface](https://moveit.picknik.ai/galactic/doc/examples/move_group_interface/move_group_interface_tutorial.html) is the recommended interface and is an abstraction on top of the move group ROS layer, that handles a lot of the setup and configuration. Unfortunately, the MGI interface is [not yet ported]() to Moveit2. Furthermore, there is the [MoveitCpp]() interface, that has better performance, but should be used for high-demanding use cases.

More about Moveit's architecture can be found [here](https://moveit.ros.org/documentation/concepts/).


## Moveit hardware interface

Moveit has its own "controller managers", which makes it agnostic to the underlying hardware layers. Nonetheless it expects the controllers to expose ros2_control-like  interfaces:

- Jointrajectory controller with JointTrajectory action interface for planning robot trajectories
- Joint state publisher
- Grippercontroller that accepts GripperCommand actions
- (Forward controllers for closed-loop control)

More information at:
- https://ros-planning.github.io/moveit_tutorials/doc/controller_configuration/controller_configuration_tutorial.html
- https://picknik.ai/ros_control/.
