// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// romea
#include <romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommand.hpp>

// std
#include <memory>
#include <utility>
#include <string>

// local
#include "romea_teleop/teleop_base.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
template<class CommandType>
TeleopBase<CommandType>::TeleopBase(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("teleop_node", options)),
  joy_(nullptr),
  cmd_pub_(nullptr),
  cmd_mux_client_(node_)
{
}

//-----------------------------------------------------------------------------
template<class CommandType>
TeleopBase<CommandType>::~TeleopBase()
{
  try {
    cmd_mux_client_.unsubscribe(cmd_pub_->get_topic_name());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), e.what());
  }
}
//-----------------------------------------------------------------------------
template<class CommandType>
void TeleopBase<CommandType>::declare_parameters_()
{
  declare_command_output_message_type(node_);
  declare_command_output_message_priority(node_);
  declare_command_ranges_();

  declare_joystick_axes_mapping_();
  declare_joystick_buttons_mapping_();
}

//-----------------------------------------------------------------------------
template<class CommandType>
void TeleopBase<CommandType>::init_joystick_()
{
  auto axes_mapping = get_joystick_axes_mapping_();
  auto buttons_mapping = get_joystick_buttons_mapping_();
  joy_ = std::make_unique<Joystick>(node_, axes_mapping, buttons_mapping);

  auto callback = std::bind(&TeleopBase::joystick_callback_, this, std::placeholders::_1);
  joy_->registerOnReceivedMsgCallback(std::move(callback));
}

//-----------------------------------------------------------------------------
template<class CommandType>
void TeleopBase<CommandType>::init_command_publisher_()
{
  get_command_ranges_();
  int priority = get_command_output_message_priority(node_);
  std::string msg_type = get_command_output_message_type(node_);

  cmd_pub_ = make_command_publisher<CommandType>(node_, msg_type);
  cmd_pub_->activate();

  if (priority != -1) {
    cmd_mux_client_.subscribe(cmd_pub_->get_topic_name(), priority, 0.05);
  }
}


//-----------------------------------------------------------------------------
template<class CommandType>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
TeleopBase<CommandType>::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

template class TeleopBase<SkidSteeringCommand>;
template class TeleopBase<OmniSteeringCommand>;
template class TeleopBase<OneAxleSteeringCommand>;
template class TeleopBase<TwoAxleSteeringCommand>;

}  // namespace romea
