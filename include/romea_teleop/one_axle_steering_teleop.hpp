#ifndef _OneAxleSteeringTeleop_HPP
#define _OneAxleSteeringTeleop_HPP


//roma
#include "teleop_base.hpp"

namespace romea
{

class OneAxleSteeringTeleop : public TeleopBase<OneAxleSteeringCommand>
{
public:

  ROMEA_TELEOP_PUBLIC
  OneAxleSteeringTeleop(const rclcpp::NodeOptions & options);

  ROMEA_TELEOP_PUBLIC
  virtual ~OneAxleSteeringTeleop()=default;

private:

  virtual void declare_joystick_remappings_() override;

  virtual Remappings get_joystick_remappings_() override;

  virtual void  declare_command_ranges_() override;

  virtual void get_command_ranges_() override;

  virtual void joystick_callback_(const Joystick & joy)override;

private:

  MaximalSpeeds maximal_linear_speeds_;
  double maximal_steering_angle_;
  bool sent_disable_msg_;
};

}
#endif
