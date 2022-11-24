#include "romea_teleop/one_axle_steering_teleop.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
OneAxleSteeringTeleop::OneAxleSteeringTeleop(const rclcpp::NodeOptions & options):
  TeleopBase(options),
  maximal_linear_speeds_(),
  maximal_steering_angle_(0),
  sent_disable_msg_(false)
{
  declare_parameters_();
  init_command_publisher_();
  init_joystick_();
}

//-----------------------------------------------------------------------------
void OneAxleSteeringTeleop::declare_command_ranges_()
{
  declare_maximal_linear_speeds(node_);
  declare_maximal_steering_angle(node_);
}

//-----------------------------------------------------------------------------
void OneAxleSteeringTeleop::get_command_ranges_()
{
  maximal_linear_speeds_ = get_maximal_linear_speeds(node_);
  maximal_steering_angle_ = get_maximal_steering_angle(node_);
}

//-----------------------------------------------------------------------------
void OneAxleSteeringTeleop::declare_joystick_remappings_()
{
  declare_linear_speed_axe_remap(node_);
  declare_steering_angle_axe_remap(node_);
  declare_slow_mode_button_remap(node_);
  declare_turbo_mode_button_remap(node_);
}

//-----------------------------------------------------------------------------
OneAxleSteeringTeleop::Remappings OneAxleSteeringTeleop::get_joystick_remappings_()
{
  return {
    {"linear_speed", get_linear_speed_axe_remap(node_)},
    {"steering_angle", get_steering_angle_axe_remap(node_)},
    {"slow_mode", get_slow_mode_button_remap(node_)},
    {"turbo_mode", get_turbo_mode_button_remap(node_)}
  };
}

//-----------------------------------------------------------------------------
void OneAxleSteeringTeleop::joystick_callback_(const Joystick &joy)
{
  OneAxleSteeringCommand cmd_msg;

  if (joy.getButtonValue("turbo_mode"))
  {
    cmd_msg.longitudinalSpeed = joy.getAxeValue("linear_speed")*maximal_linear_speeds_.turbo_mode;
    cmd_msg.steeringAngle = joy.getAxeValue("steering_angle")*maximal_steering_angle_;
    cmd_pub_->publish(cmd_msg);
    sent_disable_msg_ = false;
  } else if (joy.getButtonValue("slow_mode")) {
    cmd_msg.longitudinalSpeed = joy.getAxeValue("linear_speed")*maximal_linear_speeds_.slow_mode;
    cmd_msg.steeringAngle = joy.getAxeValue("steering_angle")*maximal_steering_angle_;
    cmd_pub_->publish(cmd_msg);
    sent_disable_msg_ = false;
  } else {
    if (!sent_disable_msg_)
    {
      cmd_pub_->publish(cmd_msg);
      sent_disable_msg_ = true;
    }
  }
}

}  // namespace romea

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(romea::OneAxleSteeringTeleop)
