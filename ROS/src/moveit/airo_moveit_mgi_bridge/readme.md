# Moveit Move Group Interface (MGI) Bridge.

In this package a Node is created that serves as a bridge to the Moveit Move Group Interface (MGI) by exposing a number of services (messages are defined in the `airo_moveit_mgi_bridge_msgs` package).


## Rationale
[MGI](https://moveit.picknik.ai/foxy/doc/move_group_interface/move_group_interface_tutorial.html) is the high-level C++ wrapper around the central Move Group node in Moveit, and is the recommended way to interact with Moveit.

As of now, there are [no python bindings](https://github.com/ros-planning/moveit2/issues/314) available for the MGI. This bridge provides a convenient interface (read: python interface) to some MGI features. The alternative is to address the low-level actions etc. of the Move Group node manually from python. This allows for greater flexibility (e.g. provide constraints to a planning request, ...) but this is not documented within the ROS tutorials or on other fora, which makes it a little harder to do (especially given the already steep ROS / Moveit learning curves).

This package should be deprecated as soon as these bindings become available.

## How to use

Start the bridge node:
- (build & source)
- start  the robot (and gripper) controllers
- start Moveit for the configuration.
- `ros2 launch airo_moveit_mgi_bridge mgi_bridge.launch.py ur_type:=ur3e robot_ip:="placeholder" use_fake_hardware:=true planning_group:=ur_manipulator` to launch the bridge for a fake UR3e
(for now, some elements are hardcoded for the UR + Robotiq 2F85 in the launch file, which needs the URDF and SRDF as there is no global param server.. TODO-> make this configurable)
- Check if all services are exposed as expected using `ros2 service list`

- Address the services over the CLI or with a client library, see below for examples.

## Available Services
### MoveToPose

 (attempt to) Move the specified Frame to the specified Pose. If no frame is specified, Moveit's default frame (the current end-effector frame) is used.

`ros2 service call /moveit_mgi_bridge/move_to_pose airo_moveit_mgi_bridge_msgs/srv/MoveToPose '{frame: tool0, target_pose : {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {x: 1.0, y: 0.0, z: 0.0 w: 0.0}}}'`

### GetFramePose

Get the Pose of the specified Frame.

`ros2 service call /moveit_mgi_bridge/get_frame_pose airo_moveit_mgi_bridge_msgs/srv/GetFramePose 'frame : tool0'`

### ExecuteCartesianPath
Use the computeCartesian Path interface of the MGI to execute an array of waypoints. Use case for this function is to execute particular motions in collision free regions, e.g. move along an arc. Note that this function is a little brittle. For optimal results: Move the EEF close to the start point of the waypoint path and generate enough intermediate points. Alternatively, increase the `jump_treshold` parameter, although this should be used with care on a real robot.. Consider the PILZ planner for more robust Cartesian waypoint execution.

parameters:
- waypoints : Set of Poses to interpolate linearly
- eef_step : step size between waypoints for interpolated the joint-space trajectory [m]
- jump_treshold : a (strange) measure for how much the joint states can jump between waypoints (which are independently solved with IK), if too low the function will fail. If too high, robot motion can be very jerky.
- max_<..>_scaling : limit on the relative maximal velocity/acceleration of each joint.


See `examples/execute_cartesian_waypoints.py` for an example.

## How to add a new service
- define a new service in the `/srv` folder of the msgs packages
- add the service to the `cmakelist.txt` in the msgs package
- create a subscription and implement the callback in `src/mgi_bridge.cpp`

cf the [ROS tutorials](https://docs.ros.org/en/galactic/Tutorials.html) for more information on creating ros services, ...
