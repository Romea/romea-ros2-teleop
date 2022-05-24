#ifndef _SkidSteeringTeleop_HPP
#define _SkidSteeringTeleop_HPP


//romea
#include "romea_teleop/teleop_base.hpp"

namespace romea
{

class SkidSteeringTeleop : public TeleopBase<SkidSteeringCommand>
{
public:

  ROMEA_TELEOP_PUBLIC
  SkidSteeringTeleop(const rclcpp::NodeOptions & options);

  ROMEA_TELEOP_PUBLIC
  virtual ~SkidSteeringTeleop()=default;

private:

  virtual void declare_joystick_remappings_() override;

  virtual Remappings get_joystick_remappings_() override;

  virtual void  declare_command_ranges_() override;

  virtual void get_command_ranges_() override;

  virtual void joystick_callback_(const Joystick &joy)override;

private:

  MaximalSpeeds maximal_linear_speeds_;
  MaximalSpeeds maximal_angular_speeds_;
  bool sent_disable_msg_;
};

}
#endif
