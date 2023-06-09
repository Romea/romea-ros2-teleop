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


from ament_index_python.packages import get_package_share_directory
import yaml


def get_cmd_output(teleop_configuration):
    return teleop_configuration["cmd_output"]


def get_cmd_range(teleop_configuration):
    return teleop_configuration["cmd_range"]


def get_joystick_mapping(teleop_configuration):

    if "joystick_mapping" in teleop_configuration:
        return teleop_configuration["joystick_mapping"]
    else:
        return None


def get_default_joystick_remapping(joystick_type, command_type):

    default_joystick_remapping_yaml_file = (
        get_package_share_directory("romea_teleop_description")
        + "/config/"
        + joystick_type
        + "_"
        + command_type
        + "_remappings.yaml"
    )

    with open(default_joystick_remapping_yaml_file) as f:
        return yaml.safe_load(f)


# def get_joystick_remapping(joystick_type, robot_model, robot_type,teleop_configuration):

#     joystick_mapping = get_joystick_mapping(teleop_configuration)

#     if not joystick_mapping:
#         joystick_type = get_joystick_type(joystick_meta_description)
#         command_type = get_command_type(
#             get_mobile_base_description(robot_type, robot_model)
#         )
#         joystick_mapping = get_default_joystick_remapping(joystick_type, command_type)
