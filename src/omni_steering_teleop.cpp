#include "romea_teleop/omni_steering_teleop.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
OmniSteeringTeleop::OmniSteeringTeleop(const rclcpp::NodeOptions &options):
  TeleopBase(options),
  maximal_linear_speeds_(),
  maximal_lateral_speeds_(),
  maximal_angular_speeds_(),
  sent_disable_msg_(false)
{
  declare_parameters_();
  init_command_publisher_();
  init_joystick_();
}

//-----------------------------------------------------------------------------
void OmniSteeringTeleop::declare_command_ranges_()
{
  declare_maximal_linear_speeds(node_);
  declare_maximal_lateral_speeds(node_);
  declare_maximal_angular_speeds(node_);
}

//-----------------------------------------------------------------------------
void OmniSteeringTeleop::get_command_ranges_()
{
  maximal_linear_speeds_ = get_maximal_linear_speeds(node_);
  maximal_lateral_speeds_ = get_maximal_lateral_speeds(node_);
  maximal_angular_speeds_ = get_maximal_angular_speeds(node_);
}

//-----------------------------------------------------------------------------
void OmniSteeringTeleop::declare_joystick_remappings_()
{
  declare_linear_speed_axe_remap(node_);
  declare_lateral_speed_axe_remap(node_);
  declare_angular_speed_axe_remap(node_);
  declare_slow_mode_button_remap(node_);
  declare_turbo_mode_button_remap(node_);
}

//-----------------------------------------------------------------------------
OmniSteeringTeleop::Remappings OmniSteeringTeleop::get_joystick_remappings_()
{
  return {
    {"linear_speed", get_linear_speed_axe_remap(node_)},
    {"lateral_speed", get_lateral_speed_axe_remap(node_)},
    {"angular_speed", get_angular_speed_axe_remap(node_)},
    {"slow_mode", get_slow_mode_button_remap(node_)},
    {"turbo_mode", get_turbo_mode_button_remap(node_)}
  };
}

//-----------------------------------------------------------------------------
void OmniSteeringTeleop::joystick_callback_(const Joystick &joy)
{
  OmniSteeringCommand cmd_msg;
  if (joy.getButtonValue("turbo_mode"))
  {
    cmd_msg.longitudinalSpeed = joy.getAxeValue("linear_speed")*maximal_linear_speeds_.turbo_mode;
    cmd_msg.lateralSpeed = joy.getAxeValue("lateral_speed")*maximal_lateral_speeds_.turbo_mode;
    cmd_msg.angularSpeed = joy.getAxeValue("angular_speed")*maximal_angular_speeds_.turbo_mode;
    cmd_pub_->publish(cmd_msg);
    sent_disable_msg_ = false;
  } else if (joy.getButtonValue("slow_mode")) {
    cmd_msg.longitudinalSpeed = joy.getAxeValue("linear_speed")*maximal_linear_speeds_.slow_mode;
    cmd_msg.lateralSpeed = joy.getAxeValue("lateral_speed")*maximal_lateral_speeds_.slow_mode;
    cmd_msg.angularSpeed = joy.getAxeValue("angular_speed")*maximal_angular_speeds_.slow_mode;
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
RCLCPP_COMPONENTS_REGISTER_NODE(romea::OmniSteeringTeleop)
