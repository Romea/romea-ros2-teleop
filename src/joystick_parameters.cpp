#include "romea_teleop/joystick_parameters.hpp"
#include <romea_common_utils/params/node_parameters.hpp>

namespace  {

const std::string JOYSTICK_TYPE_PARAM_NAME="joystick.type";

const std::string FORWARD_SPEED_AXE_REMAP_PARAM_NAME = "joystick.remapping.axes.forward_speed";
const std::string BACKWARD_SPEED_AXE_REMAP_PARAM_NAME = "joystick.remapping.axes.backward_speed";
const std::string LINEAR_SPEED_AXE_REMAP_PARAM_NAME = "joystick.remapping.axes.linear_speed";
const std::string LATERAL_SPEED_AXE_REMAP_PARAM_NAME = "joystick.remapping.axes.lateral_speed";
const std::string ANGULAR_SPEED_AXE_REMAP_PARAM_NAME = "joystick.remapping.axes.angular_speed";
const std::string STEERING_ANGLE_AXE_REMAP_PARAM_NAME = "joystick.remapping.axes.steering_angle";
const std::string FRONT_STEERING_ANGLE_AXE_REMAP_PARAM_NAME = "joystick.remapping.axes.front_steering_angle";
const std::string REAR_STEERING_ANGLE_AXE_REMAP_PARAM_NAME = "joystick.remapping.axes.rear_steering_angle";
const std::string SLOW_MODE_BUTTON_REMAP_PARAM_NAME = "joystick.remapping.buttons.slow_mode";
const std::string TURBO_MODE_BUTTON_REMAP_PARAM_NAME = "joystick.remapping.buttons.turbo_mode";

}

namespace romea {



//-----------------------------------------------------------------------------
void declare_forward_speed_axe_remap(std::shared_ptr<rclcpp::Node> node)
{
  romea::declare_parameter<std::string>(node,FORWARD_SPEED_AXE_REMAP_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_backward_speed_axe_remap(std::shared_ptr<rclcpp::Node> node)
{
  romea::declare_parameter<std::string>(node,BACKWARD_SPEED_AXE_REMAP_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_linear_speed_axe_remap(std::shared_ptr<rclcpp::Node> node)
{
  romea::declare_parameter<std::string>(node,LINEAR_SPEED_AXE_REMAP_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_lateral_speed_axe_remap(std::shared_ptr<rclcpp::Node> node)
{
  romea::declare_parameter<std::string>(node,LATERAL_SPEED_AXE_REMAP_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_angular_speed_axe_remap(std::shared_ptr<rclcpp::Node> node)
{
  romea::declare_parameter<std::string>(node,ANGULAR_SPEED_AXE_REMAP_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_steering_angle_axe_remap(std::shared_ptr<rclcpp::Node> node)
{
  romea::declare_parameter<std::string>(node,STEERING_ANGLE_AXE_REMAP_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_front_steering_angle_axe_remap(std::shared_ptr<rclcpp::Node> node)
{
  romea::declare_parameter<std::string>(node,FRONT_STEERING_ANGLE_AXE_REMAP_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_rear_steering_angle_axe_remap(std::shared_ptr<rclcpp::Node> node)
{
  romea::declare_parameter<std::string>(node,REAR_STEERING_ANGLE_AXE_REMAP_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_slow_mode_button_remap(std::shared_ptr<rclcpp::Node> node)
{
  romea::declare_parameter<std::string>(node,SLOW_MODE_BUTTON_REMAP_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_turbo_mode_button_remap(std::shared_ptr<rclcpp::Node> node)
{
  romea::declare_parameter<std::string>(node,TURBO_MODE_BUTTON_REMAP_PARAM_NAME);
}


//-----------------------------------------------------------------------------
std::string get_forward_speed_axe_remap(std::shared_ptr<rclcpp::Node> node)
{
  return romea::get_parameter<std::string>(node,FORWARD_SPEED_AXE_REMAP_PARAM_NAME);
}

//-----------------------------------------------------------------------------
std::string get_backward_speed_axe_remap(std::shared_ptr<rclcpp::Node> node)
{
  return romea::get_parameter<std::string>(node,BACKWARD_SPEED_AXE_REMAP_PARAM_NAME);
}

//-----------------------------------------------------------------------------
std::string get_linear_speed_axe_remap(std::shared_ptr<rclcpp::Node> node)
{
  return romea::get_parameter<std::string>(node,LINEAR_SPEED_AXE_REMAP_PARAM_NAME);
}

//-----------------------------------------------------------------------------
std::string get_lateral_speed_axe_remap(std::shared_ptr<rclcpp::Node> node)
{
  return romea::get_parameter<std::string>(node,LATERAL_SPEED_AXE_REMAP_PARAM_NAME);
}

//-----------------------------------------------------------------------------
std::string get_angular_speed_axe_remap(std::shared_ptr<rclcpp::Node> node)
{
  return romea::get_parameter<std::string>(node,ANGULAR_SPEED_AXE_REMAP_PARAM_NAME);
}

//-----------------------------------------------------------------------------
std::string get_steering_angle_axe_remap(std::shared_ptr<rclcpp::Node> node)
{
  return romea::get_parameter<std::string>(node,STEERING_ANGLE_AXE_REMAP_PARAM_NAME);
}

//-----------------------------------------------------------------------------
std::string get_front_steering_angle_axe_remap(std::shared_ptr<rclcpp::Node> node)
{
  return romea::get_parameter<std::string>(node,FRONT_STEERING_ANGLE_AXE_REMAP_PARAM_NAME);
}

//-----------------------------------------------------------------------------
std::string get_rear_steering_angle_axe_remap(std::shared_ptr<rclcpp::Node> node)
{
  return romea::get_parameter<std::string>(node,REAR_STEERING_ANGLE_AXE_REMAP_PARAM_NAME);
}

//-----------------------------------------------------------------------------
std::string get_slow_mode_button_remap(std::shared_ptr<rclcpp::Node> node)
{
  return romea::get_parameter<std::string>(node,SLOW_MODE_BUTTON_REMAP_PARAM_NAME);
}

//-----------------------------------------------------------------------------
std::string get_turbo_mode_button_remap(std::shared_ptr<rclcpp::Node> node)
{
  return romea::get_parameter<std::string>(node,TURBO_MODE_BUTTON_REMAP_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_joystick_type(std::shared_ptr<rclcpp::Node> node)
{
  romea::declare_parameter<std::string>(node,JOYSTICK_TYPE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
std::string get_joystick_type(std::shared_ptr<rclcpp::Node> node)
{
  return romea::get_parameter<std::string>(node,JOYSTICK_TYPE_PARAM_NAME);
}

}
