from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from romea_joystick_bringup import get_joystick_type, get_joystick_driver_pkg
from romea_mobile_base_bringup import get_base_type, get_base_model
import yaml

def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_base_meta_description(context):
    base_meta_description_filename = LaunchConfiguration(
        "base_meta_description_filename"
    ).perform(context)

    with open(base_meta_description_filename) as f:
        return yaml.safe_load(f)


def get_joystick_meta_description(context):
    joystick_meta_description_filename = LaunchConfiguration(
        "joystick_meta_description_filename"
    ).perform(context)

    with open(joystick_meta_description_filename) as f:
        return yaml.safe_load(f)


def get_teleop_configuration_filename(context):
    return LaunchConfiguration("teleop_configuration_filename").perform(context)


def launch_setup(context, *args, **kwargs):

    robot_namespace = get_robot_namespace(context)
    base_meta_description = get_base_meta_description(context)
    joystick_meta_description = get_joystick_meta_description(context)
    teleop_configuration_filename = get_teleop_configuration_filename(context)

    robot_type = get_base_type(base_meta_description)
    robot_model = get_base_model(base_meta_description)
    joystick_type = get_joystick_type(joystick_meta_description)
    joystick_driver = get_joystick_driver_pkg(joystick_meta_description)


    print("robot_type",robot_type)
    print("robot_model",robot_model)

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_teleop_drivers")
            + "/launch/teleop.launch.py"
        ),
        launch_arguments={
            "robot_type": robot_type,
            "robot_model": robot_model,
            "joystick_type": joystick_type,
            "joystick_driver": joystick_driver,
            "teleop_configuration_filename": teleop_configuration_filename,
        }.items(),
    )

    actions = [PushRosNamespace(robot_namespace), teleop]

    return [GroupAction(actions)]

    

def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("base_meta_description_filename"))

    declared_arguments.append(
        DeclareLaunchArgument("joystick_meta_description_filename")
    )

    declared_arguments.append(DeclareLaunchArgument("teleop_configuration_filename"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
