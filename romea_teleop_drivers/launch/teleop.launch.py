# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from romea_mobile_base_description import get_mobile_base_description, get_command_type
from romea_teleop_description import complete_teleop_configuration
import yaml


def get_teleop_configuration(context):

    teleop_configuration_file_path = LaunchConfiguration(
        "teleop_configuration_file_path"
    ).perform(context)

    with open(teleop_configuration_file_path) as f:
        return yaml.safe_load(f)


def get_robot_type(context):
    return LaunchConfiguration("robot_type").perform(context)


def get_robot_model(context):
    return LaunchConfiguration("robot_model").perform(context)


def get_joystick_type(context):
    return LaunchConfiguration("joystick_type").perform(context)


def get_joystick_driver(context):
    return LaunchConfiguration("joystick_driver").perform(context)


def launch_setup(context, *args, **kwargs):

    robot_type = get_robot_type(context)
    robot_model = get_robot_model(context)
    joystick_type = get_joystick_type(context)
    joystick_driver = get_joystick_driver(context)
    teleop_configuration = get_teleop_configuration(context)

    print("robot_type", robot_type)
    print("robot_model", robot_model)
    print("joystick_type", joystick_type)
    print("joystick_driver", joystick_driver)

    mobile_base_info = get_mobile_base_description(robot_type, robot_model)
    teleop_configuration = complete_teleop_configuration(
        teleop_configuration, mobile_base_info, joystick_type, joystick_driver
    )

    print()

    teleop = Node(
        package="romea_teleop_drivers",
        executable=get_command_type(mobile_base_info) + "_teleop_node",
        name="teleop",
        parameters=[teleop_configuration],
        output="screen",
    )

    return [teleop]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("robot_type"))

    declared_arguments.append(DeclareLaunchArgument("robot_model", default_value=""))

    declared_arguments.append(DeclareLaunchArgument("joystick_type"))

    declared_arguments.append(
        DeclareLaunchArgument("joystick_driver", default_value="joy")
    )

    declared_arguments.append(DeclareLaunchArgument("teleop_configuration_file_path"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
