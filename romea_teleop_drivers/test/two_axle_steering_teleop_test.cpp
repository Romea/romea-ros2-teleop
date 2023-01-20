// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// gtest
#include <gtest/gtest.h>

// romea
#include <romea_common_utils/listeners/data_listener.hpp>

// std
#include <map>
#include <memory>
#include <string>

// local
#include "../test/test_helper.h"
#include "testable_teleop.hpp"
#include "romea_teleop/two_axle_steering_teleop.hpp"

using TestableTwoAxleSteeringTeleop = TestableTeleop<romea::TwoAxleSteeringTeleop>;
using TwoAxleSteeringCommandListener = romea::DataListenerBase<romea::TwoAxleSteeringCommand>;

class MessageJoystickPublisher
{
public:
  MessageJoystickPublisher(
    const std::shared_ptr<rclcpp::Node> & node,
    const std::map<std::string, int> & joystick_mapping)
  : joystick_mapping_(joystick_mapping),
    joy_pub_(node->create_publisher<sensor_msgs::msg::Joy>("joystick/joy", 1))
  {
  }

  void publish(
    const double & backward_speed_axe_value,
    const double & forward_speed_axe_value,
    const double & front_steering_angle_axe_value,
    const double & rear_steering_angle_axe_value,
    const int slow_mode_button_value,
    const int turbo_mode_button_value)
  {
    sensor_msgs::msg::Joy msg;
    msg.axes.resize(20, 0);
    msg.buttons.resize(20, 0);
    msg.axes[0] = backward_speed_axe_value;
    msg.axes[1] = forward_speed_axe_value;
    msg.axes[2] = front_steering_angle_axe_value;
    msg.axes[3] = rear_steering_angle_axe_value;
    msg.buttons[0] = slow_mode_button_value;
    msg.buttons[1] = turbo_mode_button_value;
    joy_pub_->publish(msg);
  }

private:
  std::map<std::string, int> joystick_mapping_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Joy>> joy_pub_;
};


class TestTwoAxleSteeringTeleop : public ::testing::Test
{
public:
  TestTwoAxleSteeringTeleop()
  : teleop(),
    joy_pub(),
    cmd_sub()
  {
  }

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  template<typename MgsType>
  void make_listener(const std::string & topic_name)
  {
    cmd_sub = romea::make_data_listener<romea::TwoAxleSteeringCommand, MgsType>(
      teleop->get_node(), topic_name, romea::best_effort(1));
  }

  void init()
  {
    rclcpp::NodeOptions no;

    no.arguments(
      {"--ros-args",
        "--params-file",
        std::string(TEST_DIR) + "/two_axle_steering_teleop.yaml"});

    teleop = std::make_unique<TestableTwoAxleSteeringTeleop>(no);

    joy_pub = std::make_unique<MessageJoystickPublisher>(
      teleop->get_node(), teleop->get_mapping());


    std::string message_type = romea::get_command_output_message_type(teleop->get_node());

    if (message_type == "four_wheel_steering_msgs/FourWheelSteering") {
      return make_listener<four_wheel_steering_msgs::msg::FourWheelSteering>("cmd_4ws");
    } else if (message_type == "romea_mobile_base_msgs/TwoAxleSteeringCommand") {
      return make_listener<romea_mobile_base_msgs::msg::TwoAxleSteeringCommand>(
        "cmd_two_axle_steering");
    }
  }

  void sendJoyMsgAndWait(
    const double & backward_speed,
    const double & forward_speed,
    const double & front_steering_angle,
    const double & rear_steering_angle,
    const int slow_mode,
    const int turbo_mode)
  {
    joy_pub->publish(
      backward_speed,
      forward_speed,
      front_steering_angle,
      rear_steering_angle,
      slow_mode,
      turbo_mode);

    rclcpp::spin_some(teleop->get_node());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(teleop->get_node());
  }


  std::unique_ptr<TestableTwoAxleSteeringTeleop> teleop;
  std::unique_ptr<MessageJoystickPublisher> joy_pub;
  std::shared_ptr<TwoAxleSteeringCommandListener> cmd_sub;
};

TEST_F(TestTwoAxleSteeringTeleop, testSlowMode)
{
  init();
  sendJoyMsgAndWait(1.0, -1.0, 1.0, 1.0, 1, 0);

  EXPECT_DOUBLE_EQ(cmd_sub->get_data().longitudinalSpeed, 1.0);
  EXPECT_DOUBLE_EQ(cmd_sub->get_data().frontSteeringAngle, 3.0);
  EXPECT_DOUBLE_EQ(cmd_sub->get_data().rearSteeringAngle, 4.0);
}

TEST_F(TestTwoAxleSteeringTeleop, testTurboMode)
{
  init();
  sendJoyMsgAndWait(-1.0, 1.0, -1.0, -1.0, 0, 1);

  EXPECT_DOUBLE_EQ(cmd_sub->get_data().longitudinalSpeed, -2.0);
  EXPECT_DOUBLE_EQ(cmd_sub->get_data().frontSteeringAngle, -3.0);
  EXPECT_DOUBLE_EQ(cmd_sub->get_data().rearSteeringAngle, -4.0);
}

TEST_F(TestTwoAxleSteeringTeleop, testNoCmd)
{
  init();
  sendJoyMsgAndWait(1.0, 1.0, 1.0, 1.0, 0, 0);

  EXPECT_DOUBLE_EQ(cmd_sub->get_data().longitudinalSpeed, 0.0);
  EXPECT_DOUBLE_EQ(cmd_sub->get_data().frontSteeringAngle, 0.0);
  EXPECT_DOUBLE_EQ(cmd_sub->get_data().rearSteeringAngle, 0.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
