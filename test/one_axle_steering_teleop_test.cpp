//gtest
#include <gtest/gtest.h>

//romea
#include "test_helper.h"
#include "testable_teleop.hpp"
#include "romea_teleop/one_axle_steering_teleop.hpp"
#include <romea_mobile_base_utils/control/command_listener.hpp>

using TestableOneAxleSteeringTeleop = TestableTeleop<romea::OneAxleSteeringTeleop>;
using OneAxleSteeringCommandListener = romea::CommandListener<romea::OneAxleSteeringCommand>;

class MessageJoystickPublisher
{
public :

  MessageJoystickPublisher(const std::shared_ptr<rclcpp::Node> & node,
                           const std::map<std::string,int> & joystick_mapping):
    joystick_mapping_(joystick_mapping),
    joy_pub_(node->create_publisher<sensor_msgs::msg::Joy>("joy",1))
  {

  }

  void publish(const double & linear_speed_axe_value,
               const double & steering_angle_axe_value,
               const int slow_mode_button_value,
               const int turbo_mode_button_value)
  {
    sensor_msgs::msg::Joy msg;
    msg.axes.resize(20,0);
    msg.buttons.resize(20,0);
    msg.axes[joystick_mapping_["linear_speed"]]=linear_speed_axe_value;
    msg.axes[joystick_mapping_["steering_angle"]]=steering_angle_axe_value;
    msg.buttons[joystick_mapping_["slow_mode"]]= slow_mode_button_value;
    msg.buttons[joystick_mapping_["turbo_mode"]]= turbo_mode_button_value;
    joy_pub_->publish(msg);

  }

private:

  std::map<std::string,int> joystick_mapping_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Joy>> joy_pub_;
};



class TestOneAxleSteeringTeleop : public ::testing::Test
{

public :

  TestOneAxleSteeringTeleop():
    teleop(),
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

  void init(const std::string & joystick_type)
  {
    rclcpp::NodeOptions no;

    no.arguments(
    {"--ros-args",
     "--params-file",
     std::string(TEST_DIR)+"/one_axle_steering_teleop_"+joystick_type+".yaml"});

    teleop = std::make_unique<TestableOneAxleSteeringTeleop>(no);

    joy_pub = std::make_unique<MessageJoystickPublisher>(
          teleop->get_node(),teleop->get_mapping());

    cmd_sub = std::make_unique<OneAxleSteeringCommandListener>(
          teleop->get_node(),romea::get_command_output_message_type(teleop->get_node()));

  }

  void sendJoyMsgAndWait(const double &linear_speed,
                         const double &steering_angle,
                         const int slow_mode,
                         const int turbo_mode)
  {
    joy_pub->publish(linear_speed,
                     steering_angle,
                     slow_mode,
                     turbo_mode);

    rclcpp::spin_some(teleop->get_node());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(teleop->get_node());
  }


  std::unique_ptr<TestableOneAxleSteeringTeleop> teleop;
  std::unique_ptr<MessageJoystickPublisher> joy_pub;
  std::unique_ptr<OneAxleSteeringCommandListener> cmd_sub;

};

TEST_F(TestOneAxleSteeringTeleop, testSlowModeXbox)
{
  init("xbox");
  sendJoyMsgAndWait(1.0,1.0,1,0);

  EXPECT_DOUBLE_EQ(cmd_sub->get_command().longitudinalSpeed,1.0);
  EXPECT_DOUBLE_EQ(cmd_sub->get_command().steeringAngle,3.0);
}

TEST_F(TestOneAxleSteeringTeleop, testTurboModeXbox)
{
  init("xbox");
  sendJoyMsgAndWait(-1.0,-1.0,0,1);

  EXPECT_DOUBLE_EQ(cmd_sub->get_command().longitudinalSpeed,-2.0);
  EXPECT_DOUBLE_EQ(cmd_sub->get_command().steeringAngle,-3.0);
}

TEST_F(TestOneAxleSteeringTeleop, testNoCmdXbox)
{
  init("xbox");
  sendJoyMsgAndWait(1.0,1.0,0,0);

  EXPECT_DOUBLE_EQ(cmd_sub->get_command().longitudinalSpeed,0.0);
  EXPECT_DOUBLE_EQ(cmd_sub->get_command().steeringAngle,0.0);
}

TEST_F(TestOneAxleSteeringTeleop, testSlowModeDualshock4)
{
  init("dualshock4");
  sendJoyMsgAndWait(-0.525,0.525,1,0);

  EXPECT_NEAR(cmd_sub->get_command().longitudinalSpeed,-0.5,0.001);
  EXPECT_NEAR(cmd_sub->get_command().steeringAngle,1.5,0.001);
}

TEST_F(TestOneAxleSteeringTeleop, testTurboModeDualshock4)
{
  init("dualshock4");
  sendJoyMsgAndWait(0.525,-0.525,0,1);

  EXPECT_NEAR(cmd_sub->get_command().longitudinalSpeed,1.0,0.001);
  EXPECT_NEAR(cmd_sub->get_command().steeringAngle,-1.5,0.001);
}

TEST_F(TestOneAxleSteeringTeleop, testNoCmdDualshock4)
{
  init("dualshock4");
  sendJoyMsgAndWait(1.0,1.0,0,0);

  EXPECT_DOUBLE_EQ(cmd_sub->get_command().longitudinalSpeed,0.0);
  EXPECT_DOUBLE_EQ(cmd_sub->get_command().steeringAngle,0.0);
}

//int main(int argc, char** argv)
//{
//  testing::InitGoogleTest(&argc, argv);
//  ros::init(argc, argv, "ackermann_teleop_test");

//  int ret = RUN_ALL_TESTS();
//  ros::shutdown();
//  return ret;
//}
