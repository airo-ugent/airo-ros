# airo-ros
ROS2 packages for controlling robots @ AIRO UGent using Moveit2.

Although the goal of this repo is to provide an easy-to-use interface with the hardware, some knowledge of ROS(2) and Moveit can be helpful to understand what is going on, and is most certainly required in order to extend the packages. Some useful information sources are listed [here](doc/information-sources.md)
## Repo structure

the ROS packages are located in the `ROS/src/` folder and are divided in 4 subfolders:
- `robots/`, which includes URDF descriptions, moveit configurations, drivers and controllers for robots (atm only the UR-series).
- `grippers/`, which does the same thing for all grippers we have in the lab.
- `robot_gripper_configurations/`, contains packages that provide a description and configuration for robot + gripper, leveraging packages from the above folders.
- `moveit`, contains all packages related to planning/control of the robots.

## Local development
To set up these packages locally, we recommend using a docker development environment to avoid having issues with ROS installations etc on your local device.
In VSCode, this is easy to set up (and the repo contains the required configuration files, see `.devcontainer/`). Of course you can manually start up the docker container and ssh into it somehow.

1. We pull docker images (see dev.dockerfile) to avoid building them ourselves. However, Docker only allows so many pulls per day if you are not authenticated. So, make an account at https://hub.docker.com/. Choose free account. Choose any username and any email address you own.
2. `docker login --username <username> `
`<access token or pwd>`
(Recommended to use [access tokens](https://docs.docker.com/docker-hub/access-tokens/) and use a [credentials store](https://docs.docker.com/engine/reference/commandline/login/#credentials-store) for login. )
2. Install [vscode](https://code.visualstudio.com/) and the [vscode remote development extension](https://code.visualstudio.com/docs/remote/containers).
3. Pull this repo
4. Provide access to the Xserver for the container (for opening RVIZ) by running `xhost + local:` in a host terminal.
5. Enable GPU sharing for docker containers with the [nvidia container toolkit](https://github.com/NVIDIA/nvidia-docker).
6. Install all dependencies and build the packages by running `bash build.sh` from the CLI in your container environment.

## Using this repo

As always, once you have built the packages you need to make them available to ROS by "sourcing" the environment.
From the `ROS/` folder, run `source install/setup.bash`. Now you can run `ros2 pkg list` and you should see the local packages in the list, indicating that they can be used by ROS.

### Use cases
This repo supports a number main use cases. Most of these will leverage Moveit2 for planning, collision checking etc.


The first step is always to bring up the ROS nodes that interface with the hardware (drivers, controllers) and the Moveit nodes for planning, collision checking etc.

To bring up these nodes, go to packages for the desired hardware setup (robot + gripper) under `robot_gripper_configurations` and start up the 2 launch files (see the readme of these packages).

Then, if required, add the collision scene of the robot to moveit, (not yet available.).

Then, start up the specific nodes for your use case.

#### Motion planning in a "sense-think-act" robot application.
This is a default capability of Moveit2, although the python interface is not available yet...
see the `airo_moveit_mgi_bridge` for a temporary setup to do this with python.

#### Teleoperation to collect demonstrations / sensor streams.
See `airo_teleop_joy` in the moveit folder.

#### Gym-interface to do closed-loop control for learning or to validate a learned controller.
Not yet available.
