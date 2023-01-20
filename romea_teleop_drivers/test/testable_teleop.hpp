// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef TESTABLE_TELEOP_HPP_
#define TESTABLE_TELEOP_HPP_

// ros
#include <rclcpp/rclcpp.hpp>

// std
#include <memory>
#include <map>
#include <string>


template<class TeleopType>
class TestableTeleop : public TeleopType
{
public:
  explicit TestableTeleop(const rclcpp::NodeOptions & options)
  : TeleopType(options)
  {
  }

  std::shared_ptr<rclcpp::Node> get_node() const
  {
    return TeleopType::node_;
  }

  std::map<std::string, int> get_mapping() const
  {
    return TeleopType::joy_->get_mapping();
  }
};


#endif   // TESTABLE_TELEOP_HPP_
