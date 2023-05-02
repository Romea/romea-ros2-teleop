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
// limitations under the License.


#ifndef ROMEA_TELEOP__JOYSTICK_PARAMETERS_HPP_
#define ROMEA_TELEOP__JOYSTICK_PARAMETERS_HPP_

// std
#include <string>
#include <memory>

// ros
#include "rclcpp/rclcpp.hpp"


namespace romea
{

void declare_forward_speed_axe_mapping(std::shared_ptr<rclcpp::Node> node);

void declare_backward_speed_axe_mapping(std::shared_ptr<rclcpp::Node> node);

void declare_linear_speed_axe_mapping(std::shared_ptr<rclcpp::Node> node);

void declare_lateral_speed_axe_mapping(std::shared_ptr<rclcpp::Node> node);

void declare_angular_speed_axe_mapping(std::shared_ptr<rclcpp::Node> node);

void declare_steering_angle_axe_mapping(std::shared_ptr<rclcpp::Node> node);

void declare_front_steering_angle_axe_mapping(std::shared_ptr<rclcpp::Node> node);

void declare_rear_steering_angle_axe_mapping(std::shared_ptr<rclcpp::Node> node);

void declare_slow_mode_button_mapping(std::shared_ptr<rclcpp::Node> node);

void declare_turbo_mode_button_mapping(std::shared_ptr<rclcpp::Node> node);


int get_backward_speed_axe_mapping(std::shared_ptr<rclcpp::Node> node);

int get_forward_speed_axe_mapping(std::shared_ptr<rclcpp::Node> node);

int get_linear_speed_axe_mapping(std::shared_ptr<rclcpp::Node> node);

int get_lateral_speed_axe_mapping(std::shared_ptr<rclcpp::Node> node);

int get_angular_speed_axe_mapping(std::shared_ptr<rclcpp::Node> node);

int get_steering_angle_axe_mapping(std::shared_ptr<rclcpp::Node> node);

int get_front_steering_angle_axe_mapping(std::shared_ptr<rclcpp::Node> node);

int get_rear_steering_angle_axe_mapping(std::shared_ptr<rclcpp::Node> node);

int get_slow_mode_button_mapping(std::shared_ptr<rclcpp::Node> node);

int get_turbo_mode_button_mapping(std::shared_ptr<rclcpp::Node> node);

}  // namespace romea

#endif  // ROMEA_TELEOP__JOYSTICK_PARAMETERS_HPP_
