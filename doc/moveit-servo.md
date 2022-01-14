The delta Joint state calculations an publishing can be found [here](https://github.com/ros-planning/moveit2/blob/main/moveit_ros/moveit_servo/src/servo_calcs.).

The node accepts Twists for any frame on the robot as well as joint jogs as input. The node outputs either JointTrajectory messages (for Trajectorycontrollers) or a List of values (for ForwardControllers).


[interesting video on Moveit Servo ](https://www.youtube.com/watch?v=t2V4llsSxW4&t=41s)
