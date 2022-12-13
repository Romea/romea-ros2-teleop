// gtest
#include <gtest/gtest.h>

// romea
#include "test_helper.h"
#include "testable_teleop.hpp"
#include "romea_teleop/omni_steering_teleop.hpp"
#include <romea_common_utils/listeners/data_listener.hpp>

using TestableOmniSteeringTeleop = TestableTeleop<romea::OmniSteeringTeleop>;
using OmniSteeringCommandListener = romea::DataListenerBase<romea::OmniSteeringCommand>;

class MessageJoystickPublisher
{
public :

  MessageJoystickPublisher(const std::shared_ptr<rclcpp::Node> & node,
                           const std::map<std::string, int> & joystick_mapping):
    joystick_mapping_(joystick_mapping),
    joy_pub_(node->create_publisher<sensor_msgs::msg::Joy>("joy", 1))
  {
  }

  void publish(const double & linear_speed_axe_value,
               const double & lateral_speed_axe_value,
               const double & angular_speed_axe_value,
               const int slow_mode_button_value,
               const int turbo_mode_button_value)
  {
    sensor_msgs::msg::Joy msg;
    msg.axes.resize(20, 0);
    msg.buttons.resize(20, 0);
    msg.axes[0] = linear_speed_axe_value;
    msg.axes[1] = lateral_speed_axe_value;
    msg.axes[2] = angular_speed_axe_value;
    msg.buttons[0] = slow_mode_button_value;
    msg.buttons[1] = turbo_mode_button_value;
    joy_pub_->publish(msg);
  }

private:
  std::map<std::string, int> joystick_mapping_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Joy>> joy_pub_;
};



class TestOmniSteeringTeleop : public ::testing::Test
{
public :

  TestOmniSteeringTeleop():
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

  template<typename MgsType>
  void make_listener(const std::string & topic_name)
  {
    cmd_sub  = romea::make_data_listener<romea::OmniSteeringCommand, MgsType>(
          teleop->get_node(), topic_name, romea::best_effort(1));
  }

  void init()
  {
    rclcpp::NodeOptions no;

    no.arguments(
    {"--ros-args",
     "--params-file",
     std::string(TEST_DIR)+"/omni_steering_teleop.yaml"});

    teleop = std::make_unique<TestableOmniSteeringTeleop>(no);

    joy_pub = std::make_unique<MessageJoystickPublisher>(
          teleop->get_node(), teleop->get_mapping());

    std::string message_type = romea::get_command_output_message_type(teleop->get_node());

    if ( message_type == "geometry_msgs/Twist")
    {
      make_listener<geometry_msgs::msg::Twist>("cmd_vel");
    } else if (message_type == "romea_mobile_base_msgs/OmniSteeringCommand") {
      make_listener<romea_mobile_base_msgs::msg::OmniSteeringCommand>("cmd_omni_steering");
    }
  }

  void sendJoyMsgAndWait(const double &linear_speed,
                         const double &lateral_speed,
                         const double &angular_speed,
                         const int slow_mode,
                         const int turbo_mode)
  {
    joy_pub->publish(linear_speed,
                     lateral_speed,
                     angular_speed,
                     slow_mode,
                     turbo_mode);

    rclcpp::spin_some(teleop->get_node());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(teleop->get_node());
  }


  std::unique_ptr<TestableOmniSteeringTeleop> teleop;
  std::unique_ptr<MessageJoystickPublisher> joy_pub;
  std::shared_ptr<OmniSteeringCommandListener> cmd_sub;
};

TEST_F(TestOmniSteeringTeleop, testSlowMode)
{
  init();
  sendJoyMsgAndWait(1.0, 1.0, 1.0, 1, 0);

  EXPECT_DOUBLE_EQ(cmd_sub->get_data().longitudinalSpeed, 1.0);
  EXPECT_DOUBLE_EQ(cmd_sub->get_data().lateralSpeed, 3.0);
  EXPECT_DOUBLE_EQ(cmd_sub->get_data().angularSpeed, 5.0);
}

TEST_F(TestOmniSteeringTeleop, testTurboMode)
{
  init();
  sendJoyMsgAndWait(-1.0, -1.0, -1.0, 0, 1);

  EXPECT_DOUBLE_EQ(cmd_sub->get_data().longitudinalSpeed, -2.0);
  EXPECT_DOUBLE_EQ(cmd_sub->get_data().lateralSpeed, -4.0);
  EXPECT_DOUBLE_EQ(cmd_sub->get_data().angularSpeed, -6.0);
}

TEST_F(TestOmniSteeringTeleop, testNoCmd)
{
  init();
  sendJoyMsgAndWait(1.0, 1.0, 1.0, 0, 0);

  EXPECT_DOUBLE_EQ(cmd_sub->get_data().longitudinalSpeed, 0.0);
  EXPECT_DOUBLE_EQ(cmd_sub->get_data().lateralSpeed, 0.0);
  EXPECT_DOUBLE_EQ(cmd_sub->get_data().angularSpeed, 0.0);
}

int main(int argc, char** argv)
{
 testing::InitGoogleTest(&argc, argv);
 int ret = RUN_ALL_TESTS();
 return ret;
}
