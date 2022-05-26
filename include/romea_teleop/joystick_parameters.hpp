#ifndef _JoystickParameters_HPP
#define _JoystickParameters_HPP

//ros
#include <rclcpp/rclcpp.hpp>

//std
#include <string>

namespace romea
{

   void declare_forward_speed_axe_remap(std::shared_ptr<rclcpp::Node> node);

   void declare_backward_speed_axe_remap(std::shared_ptr<rclcpp::Node> node);

   void declare_linear_speed_axe_remap(std::shared_ptr<rclcpp::Node> node);

   void declare_lateral_speed_axe_remap(std::shared_ptr<rclcpp::Node> node);

   void declare_angular_speed_axe_remap(std::shared_ptr<rclcpp::Node> node);

   void declare_steering_angle_axe_remap(std::shared_ptr<rclcpp::Node> node);

   void declare_front_steering_angle_axe_remap(std::shared_ptr<rclcpp::Node> node);

   void declare_rear_steering_angle_axe_remap(std::shared_ptr<rclcpp::Node> node);

   void declare_slow_mode_button_remap(std::shared_ptr<rclcpp::Node> node);

   void declare_turbo_mode_button_remap(std::shared_ptr<rclcpp::Node> node);


   std::string get_backward_speed_axe_remap(std::shared_ptr<rclcpp::Node> node);

   std::string get_forward_speed_axe_remap(std::shared_ptr<rclcpp::Node> node);

   std::string get_linear_speed_axe_remap(std::shared_ptr<rclcpp::Node> node);

   std::string get_lateral_speed_axe_remap(std::shared_ptr<rclcpp::Node> node);

   std::string get_angular_speed_axe_remap(std::shared_ptr<rclcpp::Node> node);

   std::string get_steering_angle_axe_remap(std::shared_ptr<rclcpp::Node> node);

   std::string get_front_steering_angle_axe_remap(std::shared_ptr<rclcpp::Node> node);

   std::string get_rear_steering_angle_axe_remap(std::shared_ptr<rclcpp::Node> node);

   std::string get_slow_mode_button_remap(std::shared_ptr<rclcpp::Node> node);

   std::string get_turbo_mode_button_remap(std::shared_ptr<rclcpp::Node> node);


   void declare_joystick_type(std::shared_ptr<rclcpp::Node> node);

   std::string get_joystick_type(std::shared_ptr<rclcpp::Node> node);

}
#endif
