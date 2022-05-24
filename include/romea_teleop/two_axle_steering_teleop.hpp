#ifndef _TwoAxleSteeringTeleop_HPP
#define _TwoAxleSteeringTeleop_HPP

//romea
#include "romea_teleop/teleop_base.hpp"

namespace romea
{

class TwoAxleSteeringTeleop : public TeleopBase<TwoAxleSteeringCommand>
{
public:

  ROMEA_TELEOP_PUBLIC
  TwoAxleSteeringTeleop(const rclcpp::NodeOptions & options);

  ROMEA_TELEOP_PUBLIC
  virtual ~TwoAxleSteeringTeleop()=default;

private:

  virtual void declare_joystick_remappings_() override;

  virtual Remappings get_joystick_remappings_() override;

  virtual void  declare_command_ranges_() override;

  virtual void get_command_ranges_() override;

  virtual void joystick_callback_(const Joystick &joy)override;

private:

  MaximalSpeeds maximal_linear_speeds_;
  double maximal_front_steering_angle_;
  double maximal_rear_steering_angle_;
  bool sent_disable_msg_;

};

}

#endif
