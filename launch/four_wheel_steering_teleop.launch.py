from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import yaml


def launch_setup(context, *args, **kwargs):

    joystick_type = LaunchConfiguration(
        "joystick_type"
    ).perform(context)

    output_message_type = LaunchConfiguration(
        "output_message_type"
    ).perform(context)

    output_message_priority = LaunchConfiguration(
        "output_message_priority"
    ).perform(context)

    base_description_yaml_filename = LaunchConfiguration(
        "base_description_yaml_filename"
    ).perform(context)

    joystick_remapping_yaml_file = (
        get_package_share_directory("romea_teleop")
        + "/config/"
        + joystick_type
        + "_two_axle_steering_remappings.yaml"
    )

    with open(base_description_yaml_filename, "r") as f:
        base_description_root = yaml.load(f, Loader=yaml.FullLoader)
        base_description_node = base_description_root["/**"]
        base_description_ros_params = base_description_node["ros__parameters"]
        base_info = base_description_ros_params["base_info"]

    wheels_steering_control_info = base_info["wheels_steering_control"]
    wheels_steering_command_info = wheels_steering_control_info["command"]
    maximal_steering_angle = wheels_steering_command_info["maximal_angle"]

    wheels_speed_control_info = base_info["wheels_speed_control"]
    wheels_speed_command_info = wheels_speed_control_info["command"]
    maximal_wheel_speed = wheels_speed_command_info["maximal_speed"]

    teleop = Node(
        package="romea_teleop",
        executable="two_axle_steering_teleop_node",
        name="teleop",
        parameters=[
            {"joystick.type": joystick_type},
            joystick_remapping_yaml_file,
            {"cmd_output.type": output_message_type},
            {"cmd_output.priority": int(output_message_priority)},
            {"cmd_range.maximal_linear_speed.slow_mode": 1.0},
            {"cmd_range.maximal_linear_speed.turbo_mode": maximal_wheel_speed},
            {"cmd_range.maximal_front_steering_angle": maximal_steering_angle},
            {"cmd_range.maximal_rear_steering_angle": maximal_steering_angle},
        ],
        remappings=[("cmd_two_axle_steering", "~/cmd_two_axle_steering")],
        output="screen",
    )

    return [teleop]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("joystick_type"))

    declared_arguments.append(DeclareLaunchArgument("base_description_yaml_filename"))

    default_output_message_type = "romea_mobile_base_msgs/TwoAxleSteeringCommand"
    declared_arguments.append(
        DeclareLaunchArgument(
            "output_message_type",
            default_value=default_output_message_type,
        )
    )

    default_output_message_priority = "-1"
    declared_arguments.append(
        DeclareLaunchArgument(
            "output_message_priority",
            default_value=default_output_message_priority,
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
