#ifndef ROMEA_TELEOP_TWO_AXLE_STEERING_TELEOP_HPP_
#define ROMEA_TELEOP_TWO_AXLE_STEERING_TELEOP_HPP_

// std
#include <map>
#include <string>

// romea
#include "romea_teleop/teleop_base.hpp"

namespace romea
{

class TwoAxleSteeringTeleop : public TeleopBase<TwoAxleSteeringCommand>
{
public:
  ROMEA_TELEOP_PUBLIC
  explicit TwoAxleSteeringTeleop(const rclcpp::NodeOptions & options);

  ROMEA_TELEOP_PUBLIC
  virtual ~TwoAxleSteeringTeleop() = default;

private:
  void declare_joystick_axes_mapping_() override;

  void declare_joystick_buttons_mapping_() override;

  std::map<std::string, int> get_joystick_axes_mapping_() override;

  std::map<std::string, int> get_joystick_buttons_mapping_() override;

  void  declare_command_ranges_() override;

  void get_command_ranges_() override;

  void joystick_callback_(const Joystick &joy)override;

private:
  MaximalSpeeds maximal_linear_speeds_;
  double maximal_front_steering_angle_;
  double maximal_rear_steering_angle_;
  bool sent_disable_msg_;
};

}  // namespace romea

#endif  // ROMEA_TELEOP_TWO_AXLE_STEERING_TELEOP_HPP_
