#include "romea_teleop/teleop_base.hpp"

#include <romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommand.hpp>

namespace romea {

//-----------------------------------------------------------------------------
template <class CommandType>
TeleopBase<CommandType>::TeleopBase(const rclcpp::NodeOptions &options):
  node_(std::make_shared<rclcpp::Node>("teleop_node", options)),
  joy_(nullptr),
  cmd_pub_(nullptr)
{
}

//-----------------------------------------------------------------------------
template <class CommandType>
void TeleopBase<CommandType>::declare_parameters_()
{
  declare_command_output_message_type(node_);
  declare_command_output_priority(node_);
  declare_command_ranges_();

  declare_joystick_type(node_);
  declare_joystick_remappings_();
}

//-----------------------------------------------------------------------------
template <class CommandType>
void TeleopBase<CommandType>::init_joystick_()
{
  auto joystick_type = get_joystick_type(node_);
  auto joystick_remappings = get_joystick_remappings_();
  joy_ = std::make_unique<Joystick>(node_,joystick_type,joystick_remappings,true);
  joy_->registerOnReceivedMsgCallback(std::bind(&TeleopBase::joystick_callback_, this, std::placeholders::_1));
}

//-----------------------------------------------------------------------------
template <class CommandType>
void TeleopBase<CommandType>::init_command_publisher_()
{
  auto msg_type  = get_command_output_message_type(node_);
  cmd_pub_ = std::make_unique<CmdPubType>(node_,msg_type);
  get_command_ranges_();
}

//-----------------------------------------------------------------------------
template <class CommandType>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
TeleopBase<CommandType>::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

template class TeleopBase<SkidSteeringCommand>;
template class TeleopBase<OmniSteeringCommand>;
template class TeleopBase<OneAxleSteeringCommand>;
template class TeleopBase<TwoAxleSteeringCommand>;

}

