# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch_ros.actions import SetParameter, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from romea_common_bringup import device_prefix
from romea_joystick_bringup import JoystickMetaDescription
from romea_mobile_base_bringup import MobileBaseMetaDescription


def get_mode(context):
    return LaunchConfiguration("mode").perform(context)


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_base_meta_description(context):

    meta_description_file_path = LaunchConfiguration(
        "base_meta_description_file_path"
    ).perform(context)

    return MobileBaseMetaDescription(meta_description_file_path)


def get_joystick_meta_description(context):
    joystick_meta_description_file_path = LaunchConfiguration(
        "joystick_meta_description_file_path"
    ).perform(context)

    return JoystickMetaDescription(joystick_meta_description_file_path)


def get_teleop_configuration_file_path(context):
    return LaunchConfiguration("teleop_configuration_file_path").perform(context)


def launch_setup(context, *args, **kwargs):

    mode = get_mode(context)
    robot_namespace = get_robot_namespace(context)
    base_meta_description = get_base_meta_description(context)
    joystick_meta_description = get_joystick_meta_description(context)
    teleop_configuration_file_path = get_teleop_configuration_file_path(context)

    base_name = base_meta_description.get_name()
    robot_type = base_meta_description.get_type()
    robot_model = base_meta_description.get_model()
    # robot_model = str(base_meta_description.get_model() or "")
    joystick_type = joystick_meta_description.get_type()
    joystick_driver = joystick_meta_description.get_driver_package()
    joystick_name = joystick_meta_description.get_name()
    # joystick_namespace = joystick_meta_description.get_namespace()
    joystick_topic = device_prefix(robot_namespace, None, joystick_name)+"joy"

    # print("robot_type", robot_type)
    # print("robot_model", robot_model)
    # print("joystick_type", joystick_type)
    # print("joystick_driver", joystick_driver)

    launch_arguments = {
        "joystick_type": joystick_type,
        "joystick_driver": joystick_driver,
        "joystick_topic": joystick_topic,
        "teleop_configuration_file_path": teleop_configuration_file_path,
    }

    if robot_model is not None:
        launch_arguments["robot_model"] = robot_model

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory(robot_type+"_bringup")
            + "/launch/"+robot_type+"_teleop.launch.py"
        ),
        launch_arguments=launch_arguments.items(),
    )

    actions = [
        SetParameter(name="use_sim_time", value=(mode != "live")),
        PushRosNamespace(robot_namespace),
        PushRosNamespace(base_name),
        teleop
    ]

    return [GroupAction(actions)]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("base_meta_description_file_path"))

    declared_arguments.append(
        DeclareLaunchArgument("joystick_meta_description_file_path")
    )

    declared_arguments.append(DeclareLaunchArgument("teleop_configuration_file_path"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
