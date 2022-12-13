#ifndef ROMEA_TELEOP_TELEOP_BASE_HPP_
#define ROMEA_TELEOP_TELEOP_BASE_HPP_

// std
#include <map>
#include <string>
#include <memory>

// ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

// romea
#include "romea_teleop/command_parameters.hpp"
#include "romea_teleop/joystick_parameters.hpp"
#include "romea_teleop/visibility_control.h"

#include <romea_mobile_base_utils/control/command_publisher.hpp>
#include <romea_cmd_mux_utils/cmd_mux_interface.hpp>
#include <romea_joystick_utils/joystick.hpp>

namespace romea
{

template <class CommandType>
class TeleopBase
{
public:
  using CmdPublisher = PublisherBase<CommandType>;

public:
  ROMEA_TELEOP_PUBLIC
  explicit TeleopBase(const rclcpp::NodeOptions & options);

  ROMEA_TELEOP_PUBLIC
  virtual ~TeleopBase();

  ROMEA_TELEOP_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

protected:
  void declare_parameters_();

  void init_command_publisher_();

  void init_joystick_();

  virtual void joystick_callback_(const Joystick & joy) = 0;

  virtual void declare_command_ranges_() = 0;

  virtual void get_command_ranges_() = 0;

  virtual void declare_joystick_axes_mapping_() = 0;

  virtual void declare_joystick_buttons_mapping_() = 0;

  virtual std::map<std::string, int> get_joystick_axes_mapping_() = 0;

  virtual std::map<std::string, int> get_joystick_buttons_mapping_() = 0;

protected :

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<Joystick> joy_;
  std::shared_ptr<CmdPublisher> cmd_pub_;
  CmdMuxInterface cmd_mux_client_;
};

}  // namespace romea

#endif  // ROMEA_TELEOP_TELEOP_BASE_HPP_
