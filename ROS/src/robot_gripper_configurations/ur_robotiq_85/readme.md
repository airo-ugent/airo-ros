# UR +  Robotiq 85

These ROS packages integrate a UR robot with the Robotiq 85 gripper and make use of the ROS2 drivers of Universal Robots and drivers for the Robotiq Grippers.
The user is expected to be somewhat familiar with ROS2, ros_control and Moveit2.

If these packages are not working, have a look at the packages of the robot and the gripper to make sure those are still doing fine. It could be that some braking changes were made to the UR drivers, or ROS could be just doing its thing..

## Usage
### Starting the drivers and controllers
- start up the controllers for the gripper and the robot using `ros2 launch ur_robotiq_85_bringup ur_robotiq_85_control.launch.py ur_type:="ur3e" robot_ip:="dummy" use_fake_hardware:=true launch_rviz:=false`.
- optional: check if all controllers and action servers were started with `ros2 action list`, which should result in
```
/joint_trajectory_controller/follow_joint_trajectory
/robotiq_gripper_controller/gripper_cmd
```
indicating that both the robot and gripper are ready to receive commands.

To connect with a real robot: specify the robot IP (and add the gripper USB device, #TODO), and set `use_fake_hardware:=false`.

### Starting Moveit

- `ros2 launch ur_robotiq_85_bringup ur_robotiq_85_moveit.launch.py ur_type:=ur3e robot_ip:=".." use_fake_hardware:=true`

- RVIZ should start up, and you can now plan with the Robot.
- Test by moving the EEF to a different target pose and do `plan and execute`. Then switch to the `robotiq_85` planning group and select "closed" goal state, to close the gripper.
- You can now connect to the Move group node, either using the C++  move group interface or by using `the airo_moveit_mgi_bridge` from the CLI or in python.

### Switching the robot to servo mode
For real-time servoing, you need to take some additional steps:
- switch the UR controller to a feedfoward controller, that simply passes the commands instead of doing interpolation on a trajectory:
- start moveit servo
- start a streaming command, e.g. teleop with a joystick, a pose tracker or an RL-based controller.

### Simulation
Not implemented yet but here is a howto.
- generate urdf with xacro
- import in Unity
- define controllers that expose the action servers / joint topics that moveit is addressing.
- start up the simulation controllers, then start up the moveit launch file.

## Overview
### ur_robotiq_85_description
- contains the urdf files to create a description of the robot with gripper.
- contains a simple launch file to visualize this urdf.

### ur_robotiq_moveit_config
contains the moveit description for the UR + 2F85:
- SRDF description of the gripper and the robot + gripper. Most importantly this defines the "planning groups" (sets of joints) that moveit uses to plan, and disable collisions along adjacent links.

contains the required configuration files for moveit:
- `controllers.yaml` defines the available controllers that Moveit should use to execute trajectories for the planning groups
- `kinematics.yaml` sets up the Kinematics solver.
- `ompl_planning.yaml` sets up the OMPL planner.
- `ur_servo.yaml` contains the configuration for the Moveit servo node.

the last 3 configuration files are based on the `ur_moveit_config` package.

### ur_robotiq_85_bringup

contains 2 launch files:
- `ur_robotiq_85_control.launch.py`: creates the URDF description using xacro, starts up drivers and controllers for the UR and the gripper.
- `ur_robotiq_85_moveit.launch.py`: creates the URDF description and loads in moveit configuration. Starts up Moveit's move group node with OMPL and KDL.

These files are heavily based on the launch files from `ur_bringup` but extensive duplication was because o.a. ROS2 has no global param server and the URDF for the UR robots has a large number of parameters in it's xacro description.

To create new combinations of Robots and grippers, you can follow the these packages as template. Some additional pointers can be found [here](creating-new-configurations.md).
