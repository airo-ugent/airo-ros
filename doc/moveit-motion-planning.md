# Moveit Motion Planning Guide


## Motion Planning
Motion planning is generating a Robot Trajectory (consisting of desired positions and possible velocities/accelerations for a set of timesteps) for moving from pose A to pose B. Possibly while avoiding collisions known objects (including the robot itself) in the scene. Possibly with some additional constraints on the robot state during execution.

Motion planning can be done either in configuration (aka joint / state) space or in target (end-effector / cartesian) space. The space in which this happens impacts the type of constraints the planner can deal with (or at least how easy it is to deal with them).

### Motion planning with constraints
Some joint space constraints:

- constraints on the range of each joint (e.g. between -pi and pi)
- (objects in scene can be translated to joint space constraints)

some cartesian constraints:
- constraints on the orientation of the end-effector (e.g. having gripper pointing down)
- constraints on the position of the end-effector (e.g. having the EEF following a line or having it staying on a particular plane).

Note that start and end position can also be formulated as constraints at a specific time.

### Types of planners
If a planner generates collision-free trajectories, it is said to be a *global planner*.

If a planner can accept cartesian constrainst, it is said to be a *cartesian planner*.


Motion planning algorithms often resort to *sampling-based* planning (e.g. RRT or PRM), as *optimization-based* planning tends to be very difficult.

Motion planners can also be (probabilistic) *complete*, meaning that they will always find a solution if one exists. RRT-based planners for example are complete as the number of samples moves to infinity. Others, like Jacobian-based planners are not complte as they can get stuck in local minima (think joint limits and singularities).

Some motion planners are *real-time*, where realtime here refers to the ability to consistently have a certain control frequency (e.g. 100Hz). This is mostly important for streaming commands such as teleoperation or (visual) servoing. There generally exists a trade-off between real-time and complete.

## Motion Planning in Moveit2

### Motion planner integration
Planners are integrated as plugins, cf Moveit schematics.

### Available motion planners
#### OMPL
Moveit's default planner is [OMPL](https://ompl.kavrakilab.org/), which is a sampling-based path planner. This planner is probabilistically complete but not real-time.
By default, this planner works in configuration space. The planner handles cartesian constraints by either sampling in task-space and then using inverse kinematics or by using forward kinematics with rejection sampling to evaluate if the sampled configuration satisfies the cartesian constraints [source](https://vimeo.com/showcase/7812155/video/480482977#t=0h49m30s). The former approach can result in large joint movements, as there are usually multiple configurations that lead to the same pose (not necessarily close to each other in joint space). The latter approach cannot deal with constraints that have a small volume (as samples will almost never fall into that region), which makes it useless to e.g. plan on a line.An updated version of the OMPL planner improves on this by still planning in joint space but projecting each sample into the feasible region to overcome the zero-volume issue. Cf. Constrained OMPL below.

Note that OMPL is actually a Path planner, that only deals with the robot kinematics to find collision-free paths. To convert this path into a trajectory (which includes time-information or equivalently velocity/acceleration profiles), time parameterization is used.

#### PILZ
PILZ is an "industrial" motion generator.
It is a realtime cartesian planner that can generate PTP (point-to-point), LIN (linear) and CIRC (circular) motions.
It has been [ported](https://github.com/ros-planning/moveit2/pull/452) to Moveit2, but there is no tutorial yet. The tutorial for Noetic can be found [here](https://ros-planning.github.io/moveit_tutorials/doc/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html).

There is also a [ROS2 example](https://github.com/henningkayser/moveit_resources/blob/pr-port_prbt_packages/prbt_moveit_config/launch/demo.launch.py).

Cartesian planners were historically not worried about collision checking and assumed the robot workspace was free of obstacles. Therefore this motion generator does not consider collisions. Moveit only performs a final check to see if the path is collision-free and rejects the trajectory if it is not.


#### Moveit Servo

Moveit Servo is a soft real-time, Jacobian-based, cartesian planner. It gets to control rates >1kHz (which is a lot faster than the UR e needs) on a best effort. It avoids collisions and singularities, and slows down when moving close towards them. Unlike the other planners, it is not a pluging but a standalone ROS node.

[MOveit ROS2 tutorial](https://moveit.picknik.ai/foxy/doc/realtime_servo/realtime_servo_tutorial.html)

The UR ROS2 packages include a configuration for moveit servo, which is a very good starting point. See this [PR](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/pull/239).

More technical information [here](../moveit-servo.md)

#### Constrained OMPL
OMPL has integrated cartesian contstraints in 2018 and of late, a PhD Student from the KUL has done a [Summer of Code](https://moveit.ros.org/moveit/2020/09/10/ompl-constrained-planning-gsoc.html) to start integrating them in Moveit.

 This allows to use so-called "constraint-projection" to project the configuration space samples onto the valid regions, in which case the sampling planner does not even notice there are constraints. This is in contrast with Moveit's old "constraint-rejection" technique in which the OMPL samples were mapped back to task space and rejected if they did not satisfy the constraints. For constraints such as a 1D line (where the volume is zero), this approach fails notably. Furthermore, this constrained OMPL approach avoids large joint space jumps, resulting in smoother trajectories that can be executed on real robots, as explained in the gsoc blog post.

In Moveit's Constrained OMPL, only position constraints have been [merged](https://github.com/ros-planning/moveit2/pull/347) and a demo is available [here](). Orientation constraints seem to be [in progress](https://github.com/ros-planning/moveit2/issues/348).



### User-defined Waypoint planning
Sometimes you want the robot to follow a user-defined path (i.e. a provided set of waypoints), instead of it going from A to B with a set of constraints.

This is indeed [possible](https://answers.ros.org/question/261368/planning-complex-plans-using-multiple-waypoints-with-moveit/) with Moveit although a little tricky. Simply planning for each line in the path would result in jerky motion as the robot would stop at each waypoint and would result in a huge overhead.

There are a number of options though:
1. use the Cartesian Interpolator
2. use PILZ's "Line Segment functionality"
3. use the [Descartes](https://industrial-training-master.readthedocs.io/en/melodic/_source/session4/Descartes-Path-Planning.html) planner. (not ported to Moveit2..)

#### Cartesian Interpolator
The cartesian Interpolator is part of the Moveit Core and exposed in the move group interface ([see](https://moveit.picknik.ai/foxy/doc/move_group_interface/move_group_interface_tutorial.html#cartesian-paths)). It will literally interpolate between the waypoints and try to do inverse kinematics for each obtained pose, while keeping the joint jump under a specified distance. This is not guaranteed to work and depends heaviliy on the IK configuration.. This planner can also easily get stuck in local minima if it encounters a collision, as it does not "Plan" but simply interpolates.

### Time (re)parameterization
Some planners are actually path planners instead of motion planners (OMPL, cartesian interpolator). To create a trajectory, time information needs to be added in a post processing step. This can be a velocity profile for each waypoint and/or acceleration. It will also include the time at which the waypoint needs to be reached by the manipulator.

more about time parameterization [here](https://ros-planning.github.io/moveit_tutorials/doc/time_parameterization/time_parameterization_tutorial.html)


## Summary

use Moveit Servo for realtime servoing, use OMPL for non-realtime A-B Motion planning, with the cartesian constraints update if there are large joint jumps or zero-volume constraints. For Following a user defined path, use the Cartesian Interpolator (with care) or the PILZ planner.

---
Additional Sources / More information

https://manipulation.csail.mit.edu/trajectories.html

https://ompl.kavrakilab.org/OMPL_Primer.pdf

https://picknik.ai/cartesian%20planners/moveit/motion%20planning/2021/01/07/guide-to-cartesian-planners-in-moveit.html
