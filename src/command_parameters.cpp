#include "romea_teleop/command_parameters.hpp"
#include <romea_common_utils/params/node_parameters.hpp>

namespace  {

const std::string COMMAND_RANGE_MAXIMAL_STEERING_ANGLE_PARAM_NAME = "cmd_range.maximal_steering_angle";
const std::string COMMAND_RANGE_MAXIMAL_FRONT_STEERING_ANGLE_PARAM_NAME = "cmd_range.maximal_front_steering_angle";
const std::string COMMAND_RANGE_MAXIMAL_REAR_STEERING_ANGLE_PARAM_NAME = "cmd_range.maximal_rear_steering_angle";
const std::string COMMAND_RANGE_MAXIMAL_LINEAR_SPEED_PARAM_NAME = "cmd_range.maximal_linear_speed";
const std::string COMMAND_RANGE_MAXIMAL_LATERAL_SPEED_PARAM_NAME = "cmd_range.maximal_lateral_speed";
const std::string COMMAND_RANGE_MAXIMAL_ANGULAR_SPEED_PARAM_NAME = "cmd_range.maximal_angular_speed";

const std::string SLOW_MODE_SUFFIX = ".slow_mode";
const std::string TURBO_MODE_SUFFIX = ".turbo_mode";

const std::string COMMAND_MESSAGE_TYPE_PARAM_NAME="cmd_output.type";
const std::string COMMAND_PRIORITY_PARAM_NAME="cmd_output.priority";


//-----------------------------------------------------------------------------
void declare_maximal_speeds(std::shared_ptr<rclcpp::Node> node,
                            const std::string & speed_param_name)
{
  romea::declare_parameter<double>(
        node, speed_param_name + SLOW_MODE_SUFFIX);

  romea::declare_parameter_with_default<double>(
        node, speed_param_name + TURBO_MODE_SUFFIX,
        std::numeric_limits<double>::quiet_NaN());

}

//-----------------------------------------------------------------------------
romea::MaximalSpeeds get_maximal_speeds(std::shared_ptr<rclcpp::Node> node,
                                 const std::string & speed_param_name)
{
  romea::MaximalSpeeds maximal_speeds;

  maximal_speeds.slow_mode = romea::get_parameter<double>(
        node, speed_param_name + SLOW_MODE_SUFFIX);

  maximal_speeds.turbo_mode= romea::get_parameter<double>(
        node, speed_param_name + TURBO_MODE_SUFFIX);

  if(std::isnan(maximal_speeds.turbo_mode))
  {
    maximal_speeds.turbo_mode = maximal_speeds.turbo_mode;
  }

  return maximal_speeds;
}


}

namespace romea {


//-----------------------------------------------------------------------------
void declare_maximal_steering_angle(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<double>(node,COMMAND_RANGE_MAXIMAL_STEERING_ANGLE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_maximal_front_steering_angle(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<double>(node,COMMAND_RANGE_MAXIMAL_FRONT_STEERING_ANGLE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_maximal_rear_steering_angle(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<double>(node,COMMAND_RANGE_MAXIMAL_REAR_STEERING_ANGLE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_maximal_linear_speeds(std::shared_ptr<rclcpp::Node> node)
{
  declare_maximal_speeds(node,COMMAND_RANGE_MAXIMAL_LINEAR_SPEED_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_maximal_lateral_speeds(std::shared_ptr<rclcpp::Node> node)
{
  declare_maximal_speeds(node,COMMAND_RANGE_MAXIMAL_LATERAL_SPEED_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_maximal_angular_speeds(std::shared_ptr<rclcpp::Node> node)
{
  declare_maximal_speeds(node,COMMAND_RANGE_MAXIMAL_ANGULAR_SPEED_PARAM_NAME);
}

//-----------------------------------------------------------------------------
double get_maximal_steering_angle(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<double>(node,COMMAND_RANGE_MAXIMAL_STEERING_ANGLE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
double get_maximal_front_steering_angle(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<double>(node,COMMAND_RANGE_MAXIMAL_FRONT_STEERING_ANGLE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
double get_maximal_rear_steering_angle(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<double>(node,COMMAND_RANGE_MAXIMAL_REAR_STEERING_ANGLE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
MaximalSpeeds get_maximal_linear_speeds(std::shared_ptr<rclcpp::Node> node)
{
  return get_maximal_speeds(node,COMMAND_RANGE_MAXIMAL_LINEAR_SPEED_PARAM_NAME);
}

//-----------------------------------------------------------------------------
MaximalSpeeds get_maximal_lateral_speeds(std::shared_ptr<rclcpp::Node> node)
{
  return get_maximal_speeds(node,COMMAND_RANGE_MAXIMAL_LATERAL_SPEED_PARAM_NAME);
}

//-----------------------------------------------------------------------------
MaximalSpeeds get_maximal_angular_speeds(std::shared_ptr<rclcpp::Node> node)
{
  return get_maximal_speeds(node,COMMAND_RANGE_MAXIMAL_ANGULAR_SPEED_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_command_output_message_type(std::shared_ptr<rclcpp::Node> node)
{
  romea::declare_parameter<std::string>(node,COMMAND_MESSAGE_TYPE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
std::string get_command_output_message_type(std::shared_ptr<rclcpp::Node> node)
{
  return romea::get_parameter<std::string>(node,COMMAND_MESSAGE_TYPE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_command_output_priority(std::shared_ptr<rclcpp::Node> node)
{
  romea::declare_parameter_with_default<int>(node,COMMAND_PRIORITY_PARAM_NAME,0);
}

//-----------------------------------------------------------------------------
double get_command_output_priority(std::shared_ptr<rclcpp::Node> node)
{
  return romea::get_parameter<int>(node,COMMAND_PRIORITY_PARAM_NAME);
}


}
