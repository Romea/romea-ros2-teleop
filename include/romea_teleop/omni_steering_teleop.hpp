#ifndef ROMEA_TELEOP_OMNI_STEERING_TELEOP_HPP_
#define ROMEA_TELEOP_OMNI_STEERING_TELEOP_HPP_


// romea
#include "romea_teleop/teleop_base.hpp"

namespace romea
{

class OmniSteeringTeleop : public TeleopBase<OmniSteeringCommand>
{
public:
  ROMEA_TELEOP_PUBLIC
  explicit OmniSteeringTeleop(const rclcpp::NodeOptions & options);

  ROMEA_TELEOP_PUBLIC
  virtual ~OmniSteeringTeleop() = default;

private:
  void declare_joystick_remappings_() override;

  Remappings get_joystick_remappings_() override;

  void  declare_command_ranges_() override;

  void get_command_ranges_() override;

  void joystick_callback_(const Joystick & joy)override;

private:
  MaximalSpeeds maximal_linear_speeds_;
  MaximalSpeeds maximal_lateral_speeds_;
  MaximalSpeeds maximal_angular_speeds_;
  bool sent_disable_msg_;
};

}  // namespace romea

#endif  // ROMEA_TELEOP_OMNI_STEERING_TELEOP_HPP_
