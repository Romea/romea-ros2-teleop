// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include <rclcpp/rclcpp.hpp>
#include "romea_teleop/skid_steering_teleop.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::executors::SingleThreadedExecutor exec;
  romea::SkidSteeringTeleop teleop(options);
  exec.add_node(teleop.get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
