"""
Launch file for controllers of the UR robot and robotiq 85 gripper
This launch file
 - creates the URDF description
 - passes this to a single robot_description node
 - starts up the controllers and driver for the UR
 - starts up the controller and driver for the Robotiq gripper
 - starts RVIZ (optionally)

 The launch file contains lots of duplicate code w.r.t the UR_ROS2_drivers control launch file unfortunately.
 This is mainly because the URDF description is needed (and there is no global param server..) for e.g. the robot description node, which means the urdf needs to be built with xacro etc.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument("robot_ip", description="IP address by which the robot can be reached.")
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )

    declared_arguments.append(DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"))

    # Initialize launch Arguments
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    prefix = LaunchConfiguration("prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # other variables.
    description_package = "ur_robotiq_85_description"
    description_file = "ur_robotiq_85.urdf.xacro"

    ur_description_package = "ur_description"
    calibration_package = "ur_description"  # link to package containing the robot calibration.

    robot_name = "ur"  # can be changed but changes must be matched in the controllers.yaml file for moveit!

    # joint controller to start
    initial_joint_controller = "joint_trajectory_controller"
    # package with control configuration for ur
    runtime_config_package = "ur_bringup"
    # file in ur_bringup package that declares the ros2_control controllers for the UR
    controllers_file = "ur_controllers.yaml"

    # generate the URDF of the robot + gripper.
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "joint_limits.yaml"]
    )
    # load from calibration package, which can optionally point to robot-specific calibration file.
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(calibration_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "visual_parameters.yaml"]
    )
    script_filename = PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "resources", "ros_control.urscript"])
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            # ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            robot_name,
            " ",
            "script_filename:=",
            script_filename,
            " ",
            "input_recipe_filename:=",
            input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            output_recipe_filename,
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "headless_mode:=",
            "false",
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # END OF UR DESCRIPTION

    # CONTROLLERS AND DRIVERS

    # common state publisher (takes in description of both robot and gripper.)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # UR driver and controllers, based on ur_bringup control launch file.

    controllers_config_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    rviz_config_file = PathJoinSubstitution([FindPackageShare(ur_description_package), "rviz", "view_robot.rviz"])

    # define update rate
    update_rate_config_file = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            ur_type,
            "_update_rate.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, update_rate_config_file, controllers_config_file],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    io_and_status_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["io_and_status_controller", "-c", "/controller_manager"],
    )

    speed_scaling_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "speed_scaling_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    force_torque_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "force_torque_sensor_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    forward_position_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["forward_position_controller", "-c", "/controller_manager", "--stopped"],
    )

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
    )

    # gripper, based on robotiq_85_bringup launch file.
    gripper_driver_node = Node(
        package="robotiq_85_driver",
        executable="robotiq_85_driver",
        name="robotiq_85_driver",
        condition=UnlessCondition(use_fake_hardware),
        parameters=[{"num_grippers": 1}, {"comport": "/dev/ttyUSB0"}, {"baud": "115200"}],
        output="screen",
    )

    gripper_controller_node = Node(
        package="robotiq_85_driver",
        executable="single_robotiq_85_action_server",
        name="robotiq_85_controller",
        condition=UnlessCondition(use_fake_hardware),
        output="screen",
    )

    fake_gripper_controller_node = Node(
        package="robotiq_85_driver",
        executable="robotiq_85_fake_action_server",
        name="robotiq_85_driver",
        condition=IfCondition(use_fake_hardware),
        output="screen",
    )

    # / CONTROLLERS AND DRIVERS

    # RVIZ
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        control_node,
        robot_state_publisher_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        io_and_status_controller_spawner,
        speed_scaling_state_broadcaster_spawner,
        force_torque_sensor_broadcaster_spawner,
        forward_position_controller_spawner_stopped,
        initial_joint_controller_spawner,
        fake_gripper_controller_node,
        gripper_driver_node,
        gripper_controller_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
