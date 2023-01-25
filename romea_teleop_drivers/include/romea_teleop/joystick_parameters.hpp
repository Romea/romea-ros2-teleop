// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

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
