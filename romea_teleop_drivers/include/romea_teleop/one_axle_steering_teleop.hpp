// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_TELEOP__ONE_AXLE_STEERING_TELEOP_HPP_
#define ROMEA_TELEOP__ONE_AXLE_STEERING_TELEOP_HPP_

// std
#include <map>
#include <string>

// romea
#include "romea_teleop/teleop_base.hpp"

namespace romea
{

class OneAxleSteeringTeleop : public TeleopBase<OneAxleSteeringCommand>
{
public:
  ROMEA_TELEOP_PUBLIC
  explicit OneAxleSteeringTeleop(const rclcpp::NodeOptions & options);

  ROMEA_TELEOP_PUBLIC
  virtual ~OneAxleSteeringTeleop() = default;

private:
  void declare_joystick_axes_mapping_() override;

  void declare_joystick_buttons_mapping_() override;

  std::map<std::string, int> get_joystick_axes_mapping_() override;

  std::map<std::string, int> get_joystick_buttons_mapping_() override;

  void  declare_command_ranges_() override;

  void get_command_ranges_() override;

  void joystick_callback_(const Joystick & joy)override;

private:
  MaximalSpeeds maximal_linear_speeds_;
  double maximal_steering_angle_;
  bool sent_disable_msg_;
};

}  // namespace romea

#endif  // ROMEA_TELEOP__ONE_AXLE_STEERING_TELEOP_HPP_
