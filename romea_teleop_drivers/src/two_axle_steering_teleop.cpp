// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License

// std
#include <map>
#include <string>
#include <iostream>

// local
#include "romea_teleop_drivers/two_axle_steering_teleop.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
TwoAxleSteeringTeleop::TwoAxleSteeringTeleop(const rclcpp::NodeOptions & options)
: TeleopBase(options),
  maximal_linear_speeds_(),
  maximal_front_steering_angle_(0),
  maximal_rear_steering_angle_(0),
  sent_disable_msg_(false)
{
  try {
    declare_parameters_();
    init_joystick_();
    init_command_publisher_();
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), e.what());
  }
}

//-----------------------------------------------------------------------------
void TwoAxleSteeringTeleop::declare_command_ranges_()
{
  declare_maximal_linear_speeds(node_);
  declare_maximal_front_steering_angle(node_);
  declare_maximal_rear_steering_angle(node_);
}

//-----------------------------------------------------------------------------
void TwoAxleSteeringTeleop::get_command_ranges_()
{
  maximal_linear_speeds_ = get_maximal_linear_speeds(node_);
  maximal_front_steering_angle_ = get_maximal_front_steering_angle(node_);
  maximal_rear_steering_angle_ = get_maximal_rear_steering_angle(node_);
}

//-----------------------------------------------------------------------------
void TwoAxleSteeringTeleop::declare_joystick_axes_mapping_()
{
  declare_forward_speed_axe_mapping(node_);
  declare_backward_speed_axe_mapping(node_);
  declare_front_steering_angle_axe_mapping(node_);
  declare_rear_steering_angle_axe_mapping(node_);
}

//-----------------------------------------------------------------------------
void TwoAxleSteeringTeleop::declare_joystick_buttons_mapping_()
{
  declare_slow_mode_button_mapping(node_);
  declare_turbo_mode_button_mapping(node_);
}

//-----------------------------------------------------------------------------
std::map<std::string, int> TwoAxleSteeringTeleop::get_joystick_axes_mapping_()
{
  return {
    {"forward_speed", get_forward_speed_axe_mapping(node_)},
    {"backward_speed", get_backward_speed_axe_mapping(node_)},
    {"front_steering_angle", get_front_steering_angle_axe_mapping(node_)},
    {"rear_steering_angle", get_rear_steering_angle_axe_mapping(node_)},
  };
}

//-----------------------------------------------------------------------------
std::map<std::string, int> TwoAxleSteeringTeleop::get_joystick_buttons_mapping_()
{
  return {
    {"slow_mode", get_slow_mode_button_mapping(node_)},
    {"turbo_mode", get_turbo_mode_button_mapping(node_)}
  };
}

//-----------------------------------------------------------------------------
void TwoAxleSteeringTeleop::joystick_callback_(const Joystick & joy)
{
  core::TwoAxleSteeringCommand cmd_msg;
  if (joy.getButtonValue("turbo_mode")) {
    cmd_msg.longitudinalSpeed = (joy.getAxeValue("forward_speed") -
      joy.getAxeValue("backward_speed")) * maximal_linear_speeds_.turbo_mode / 2;
    cmd_msg.frontSteeringAngle =
      joy.getAxeValue("front_steering_angle") * maximal_front_steering_angle_;
    cmd_msg.rearSteeringAngle =
      joy.getAxeValue("rear_steering_angle") * maximal_rear_steering_angle_;
    cmd_pub_->publish(cmd_msg);
    sent_disable_msg_ = false;
  } else if (joy.getButtonValue("slow_mode")) {
    cmd_msg.longitudinalSpeed = (joy.getAxeValue("forward_speed") -
      joy.getAxeValue("backward_speed")) * maximal_linear_speeds_.slow_mode / 2;
    cmd_msg.frontSteeringAngle =
      joy.getAxeValue("front_steering_angle") * maximal_front_steering_angle_;
    cmd_msg.rearSteeringAngle =
      joy.getAxeValue("rear_steering_angle") * maximal_rear_steering_angle_;
    cmd_pub_->publish(cmd_msg);
    sent_disable_msg_ = false;
  } else {
    // When mode button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg_) {
      cmd_pub_->publish(cmd_msg);
      sent_disable_msg_ = true;
    }
  }
}

}  // namespace ros2
}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::TwoAxleSteeringTeleop)
