import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # planning_context
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "planning_group",
            default_value="panda_arm",
            description="Which group to plan for. Must be defined in the SRDF.",
        )
    )

    # load params
    planning_group_param = LaunchConfiguration("planning_group")

    robot_description_config = load_file("moveit_resources_panda_description", "urdf/panda.urdf")
    robot_description = {"robot_description": robot_description_config}

    robot_description_semantic_config = load_file("moveit_resources_panda_moveit_config", "config/panda.srdf")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    kinematics_yaml = load_yaml("moveit_resources_panda_moveit_config", "config/kinematics.yaml")

    planning_group = {"planning_group": planning_group_param}

    # MoveGroupInterface demo executable
    mgi_bridge = Node(
        name="moveit_mgi_bridge",
        package="airo_moveit_mgi_bridge",
        executable="moveit_mgi_bridge",
        # namespace='moveit_mgi_bridge', # defaulted to node name.
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, planning_group],
    )

    return LaunchDescription(declared_arguments + [mgi_bridge])
