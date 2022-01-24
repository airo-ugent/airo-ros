# airo_teleop_joy

package that contains a node to take in joy messages and convert them into Twist commands for Moveit Servo and optionally to Grippercommands for a gripper.

The controller is used to create a Twist message for the selected robot frame (which can be switched between the base frame and a chosen EEF-frame) and to create a GripperCommand message to open or close the gripper (toggle mode).

This allows to
- create demonstrations for learning-based controllers
- perform action sequency manually (e.g. to see if it is feasible or to collect waypoint poses)
- ..

## Usage
- connect a controller and start the joy node `ros2 run joy joy_node`, which should return something like ```  [joy_node]: Opened joystick: Logitech Gamepad F310.  deadzone: 0.050000``` if the controller was found and configured.
- Start up the robot and gripper drivers/ controllers (see packages for robot/ gripper configurations), switch to an appropriate controller for moveit servo (feedforward controller usually)
- Start Moveit Move Group
- Start and activate Moveit Servo (requires configuration file!)
- start the teleop node using e.g. ` ros2 run airo_teleop_joy teleop_joy --ros-args -p publish_gripper:=true -p eef_planning_frame:="tool0"`, see `teleop_joy_node.py` for more details on the available parameters.

- Start teleoperating the robot.


## Example with simulated UR + Robotiq gripper
- #TODO
