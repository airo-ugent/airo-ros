## UR ROS2 packages
Here the official ros2 drivers and their non-released dependencies are cloned.

see https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver for more information about the drivers.

This repo includes a.o.
- driver for communication with UR robot
- description in URDF 
- configuration of controllers for ros2_control stack
- package for getting the factory kinematic callibration information from the UR.
- moveit configuration for planning, servoing,...
- launch files to bring up controllers, moveit,...

To get a quick indication if everything is workingas expected, run following in 2 different terminals after building and sourcing:
```
ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false
ros2 launch ur_bringup ur_moveit.launch.py ur_type:=ur5e robot_ip:="xxx.xxx" use_fake_hardware:=true launch_rviz:=true 
```
You show now see Moveit starting up and should be able to do motion planning with a simulated UR5e.