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


# import pytest
import yaml
from romea_teleop_description import cmd_range_clamp, complete_teleop_configuration


def get_mobile_base_configuration_(type):
    with open("test/test_" + type + "_base_info.yaml") as f:
        return yaml.safe_load(f)


def get_teleop_configuration_(type):
    with open("test/test_" + type + "_teleop_config.yaml") as f:
        return yaml.safe_load(f)


def test_skid_steering_teleop_cmd_range_clamp():
    mobile_base = get_mobile_base_configuration_("skid_steering")

    user_cmd_range = {}
    user_cmd_range["maximal_linear_speed"] = {}
    user_cmd_range["maximal_angular_speed"] = {}

    cmd_range = cmd_range_clamp(mobile_base, user_cmd_range)
    assert cmd_range["maximal_linear_speed"]["slow_mode"] == 4.0
    assert cmd_range["maximal_linear_speed"]["turbo_mode"] == 4.0
    assert cmd_range["maximal_angular_speed"]["slow_mode"] == 4.0
    assert cmd_range["maximal_angular_speed"]["turbo_mode"] == 4.0

    user_cmd_range["maximal_linear_speed"]["slow_mode"] = 1.0
    user_cmd_range["maximal_linear_speed"]["turbo_mode"] = 2.0
    user_cmd_range["maximal_angular_speed"]["slow_mode"] = 0.5
    user_cmd_range["maximal_angular_speed"]["turbo_mode"] = 1.0

    cmd_range = cmd_range_clamp(mobile_base, user_cmd_range)
    assert cmd_range["maximal_linear_speed"]["slow_mode"] == 1.0
    assert cmd_range["maximal_linear_speed"]["turbo_mode"] == 2.0
    assert cmd_range["maximal_angular_speed"]["slow_mode"] == 0.5
    assert cmd_range["maximal_angular_speed"]["turbo_mode"] == 1.0


def test_complete_skid_steering_teleop_configuration():
    mobile_base = get_mobile_base_configuration_("skid_steering")
    teleop = get_teleop_configuration_("skid_steering")
    teleop = complete_teleop_configuration(teleop, mobile_base, "xbox", "joy")

    assert teleop["cmd_range"]["maximal_linear_speed"]["slow_mode"] == 1.0
    assert teleop["cmd_range"]["maximal_linear_speed"]["turbo_mode"] == 4.0
    assert teleop["cmd_range"]["maximal_angular_speed"]["slow_mode"] == 1.57
    assert teleop["cmd_range"]["maximal_angular_speed"]["turbo_mode"] == 4.0
    assert teleop["joystick_mapping"]["buttons"]["slow_mode"] == 4
    assert teleop["joystick_mapping"]["buttons"]["turbo_mode"] == 5
    assert teleop["joystick_mapping"]["axes"]["linear_speed"] == 1
    assert teleop["joystick_mapping"]["axes"]["angular_speed"] == 3


def test_omni_steering_teleop_cmd_range_clamp():
    mobile_base = get_mobile_base_configuration_("omni_steering")

    user_cmd_range = {}
    user_cmd_range["maximal_linear_speed"] = {}
    user_cmd_range["maximal_lateral_speed"] = {}
    user_cmd_range["maximal_angular_speed"] = {}

    cmd_range = cmd_range_clamp(mobile_base, user_cmd_range)
    assert cmd_range["maximal_linear_speed"]["slow_mode"] == 4.0
    assert cmd_range["maximal_linear_speed"]["turbo_mode"] == 4.0
    assert cmd_range["maximal_lateral_speed"]["slow_mode"] == 4.0
    assert cmd_range["maximal_lateral_speed"]["turbo_mode"] == 4.0
    assert cmd_range["maximal_angular_speed"]["slow_mode"] == 4.0
    assert cmd_range["maximal_angular_speed"]["turbo_mode"] == 4.0

    user_cmd_range["maximal_linear_speed"]["slow_mode"] = 1.0
    user_cmd_range["maximal_linear_speed"]["turbo_mode"] = 2.0
    user_cmd_range["maximal_lateral_speed"]["slow_mode"] = 1.0
    user_cmd_range["maximal_lateral_speed"]["turbo_mode"] = 2.0
    user_cmd_range["maximal_angular_speed"]["slow_mode"] = 0.5
    user_cmd_range["maximal_angular_speed"]["turbo_mode"] = 1.0

    cmd_range = cmd_range_clamp(mobile_base, user_cmd_range)
    assert cmd_range["maximal_linear_speed"]["slow_mode"] == 1.0
    assert cmd_range["maximal_linear_speed"]["turbo_mode"] == 2.0
    assert cmd_range["maximal_linear_speed"]["slow_mode"] == 1.0
    assert cmd_range["maximal_linear_speed"]["turbo_mode"] == 2.0
    assert cmd_range["maximal_angular_speed"]["slow_mode"] == 0.5
    assert cmd_range["maximal_angular_speed"]["turbo_mode"] == 1.0


def test_complete_omni_steering_teleop_configuration():
    mobile_base = get_mobile_base_configuration_("omni_steering")
    teleop = get_teleop_configuration_("omni_steering")
    teleop = complete_teleop_configuration(teleop, mobile_base, "xbox", "joy")

    assert teleop["cmd_range"]["maximal_linear_speed"]["slow_mode"] == 1.0
    assert teleop["cmd_range"]["maximal_linear_speed"]["turbo_mode"] == 4.0
    assert teleop["cmd_range"]["maximal_lateral_speed"]["slow_mode"] == 0.5
    assert teleop["cmd_range"]["maximal_lateral_speed"]["turbo_mode"] == 4.0
    assert teleop["cmd_range"]["maximal_angular_speed"]["slow_mode"] == 1.57
    assert teleop["cmd_range"]["maximal_angular_speed"]["turbo_mode"] == 4.0
    assert teleop["joystick_mapping"]["buttons"]["slow_mode"] == 4
    assert teleop["joystick_mapping"]["buttons"]["turbo_mode"] == 5
    assert teleop["joystick_mapping"]["axes"]["linear_speed"] == 1
    assert teleop["joystick_mapping"]["axes"]["lateral_speed"] == 0
    assert teleop["joystick_mapping"]["axes"]["angular_speed"] == 3


def test_one_axle_steering_teleop_cmd_range_clamp():
    mobile_base = get_mobile_base_configuration_("one_axle_steering")

    user_cmd_range = {}
    user_cmd_range["maximal_linear_speed"] = {}

    cmd_range = cmd_range_clamp(mobile_base, user_cmd_range)
    assert cmd_range["maximal_linear_speed"]["slow_mode"] == 4.0
    assert cmd_range["maximal_linear_speed"]["turbo_mode"] == 4.0
    assert cmd_range["maximal_steering_angle"] == 5.0

    user_cmd_range["maximal_linear_speed"]["slow_mode"] = 1.0
    user_cmd_range["maximal_linear_speed"]["turbo_mode"] = 2.0
    user_cmd_range["maximal_steering_angle"] = 3.0

    cmd_range = cmd_range_clamp(mobile_base, user_cmd_range)
    assert cmd_range["maximal_linear_speed"]["slow_mode"] == 1.0
    assert cmd_range["maximal_linear_speed"]["turbo_mode"] == 2.0
    assert cmd_range["maximal_steering_angle"] == 3.0


def test_complete_one_axle_steering_teleop_configuration():
    mobile_base = get_mobile_base_configuration_("one_axle_steering")
    teleop = get_teleop_configuration_("one_axle_steering")
    teleop = complete_teleop_configuration(teleop, mobile_base, "xbox", "joy")

    assert teleop["cmd_range"]["maximal_linear_speed"]["slow_mode"] == 1.0
    assert teleop["cmd_range"]["maximal_linear_speed"]["turbo_mode"] == 4.0
    assert teleop["cmd_range"]["maximal_steering_angle"] == 5.0
    assert teleop["joystick_mapping"]["buttons"]["slow_mode"] == 4
    assert teleop["joystick_mapping"]["buttons"]["turbo_mode"] == 5
    assert teleop["joystick_mapping"]["axes"]["linear_speed"] == 1
    assert teleop["joystick_mapping"]["axes"]["steering_angle"] == 3


def test_two_axle_steering_teleop_cmd_range_clamp():
    mobile_base = get_mobile_base_configuration_("two_axle_steering")

    user_cmd_range = {}
    user_cmd_range["maximal_linear_speed"] = {}

    cmd_range = cmd_range_clamp(mobile_base, user_cmd_range)
    assert cmd_range["maximal_linear_speed"]["slow_mode"] == 4.0
    assert cmd_range["maximal_linear_speed"]["turbo_mode"] == 4.0
    assert cmd_range["maximal_front_steering_angle"] == 5.0
    assert cmd_range["maximal_rear_steering_angle"] == 5.0

    user_cmd_range["maximal_linear_speed"]["slow_mode"] = 1.0
    user_cmd_range["maximal_linear_speed"]["turbo_mode"] = 2.0
    user_cmd_range["maximal_front_steering_angle"] = 3.0
    user_cmd_range["maximal_rear_steering_angle"] = 3.0

    cmd_range = cmd_range_clamp(mobile_base, user_cmd_range)
    assert cmd_range["maximal_linear_speed"]["slow_mode"] == 1.0
    assert cmd_range["maximal_linear_speed"]["turbo_mode"] == 2.0
    assert cmd_range["maximal_front_steering_angle"] == 3.0
    assert cmd_range["maximal_rear_steering_angle"] == 3.0


def test_complete_two_axle_steering_teleop_configuration():
    mobile_base = get_mobile_base_configuration_("two_axle_steering")
    teleop = get_teleop_configuration_("two_axle_steering")
    teleop = complete_teleop_configuration(teleop, mobile_base, "xbox", "joy")

    assert teleop["cmd_range"]["maximal_linear_speed"]["slow_mode"] == 1.0
    assert teleop["cmd_range"]["maximal_linear_speed"]["turbo_mode"] == 4.0
    assert teleop["cmd_range"]["maximal_front_steering_angle"] == 5.0
    assert teleop["cmd_range"]["maximal_rear_steering_angle"] == 0.0

    assert teleop["joystick_mapping"]["buttons"]["slow_mode"] == 4
    assert teleop["joystick_mapping"]["buttons"]["turbo_mode"] == 5
    assert teleop["joystick_mapping"]["axes"]["forward_speed"] == 2
    assert teleop["joystick_mapping"]["axes"]["backward_speed"] == 5
    assert teleop["joystick_mapping"]["axes"]["front_steering_angle"] == 0
    assert teleop["joystick_mapping"]["axes"]["rear_steering_angle"] == 3
