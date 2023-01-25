// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_TELEOP__COMMAND_PARAMETERS_HPP_
#define ROMEA_TELEOP__COMMAND_PARAMETERS_HPP_


// std
#include <memory>
#include <string>

// ros
#include "rclcpp/rclcpp.hpp"

namespace romea
{

struct MaximalSpeeds
{
  double slow_mode;
  double turbo_mode;
};

void declare_maximal_steering_angle(std::shared_ptr<rclcpp::Node> node);

void declare_maximal_front_steering_angle(std::shared_ptr<rclcpp::Node> node);

void declare_maximal_rear_steering_angle(std::shared_ptr<rclcpp::Node> node);

void declare_maximal_linear_speeds(std::shared_ptr<rclcpp::Node> node);

void declare_maximal_lateral_speeds(std::shared_ptr<rclcpp::Node> node);

void declare_maximal_angular_speeds(std::shared_ptr<rclcpp::Node> node);


double get_maximal_steering_angle(std::shared_ptr<rclcpp::Node> node);

double get_maximal_front_steering_angle(std::shared_ptr<rclcpp::Node> node);

double get_maximal_rear_steering_angle(std::shared_ptr<rclcpp::Node> node);

MaximalSpeeds get_maximal_linear_speeds(std::shared_ptr<rclcpp::Node> node);

MaximalSpeeds get_maximal_lateral_speeds(std::shared_ptr<rclcpp::Node> node);

MaximalSpeeds get_maximal_angular_speeds(std::shared_ptr<rclcpp::Node> node);


void declare_command_output_message_type(std::shared_ptr<rclcpp::Node> node);

std::string get_command_output_message_type(std::shared_ptr<rclcpp::Node> node);

void declare_command_output_message_priority(std::shared_ptr<rclcpp::Node> node);

int get_command_output_message_priority(std::shared_ptr<rclcpp::Node> node);


}  // namespace romea

#endif  // ROMEA_TELEOP__COMMAND_PARAMETERS_HPP_
