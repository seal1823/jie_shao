#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"

using namespace std;
using std::placeholders::_1;

class JoyTeleop : public rclcpp::Node {
 public:
  JoyTeleop();

 private:
  void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg);
  void updateParameters();
  void timerCallback();
  void publishZeroMessage();
  void set_ServoPoses(float theta);

  double lin_scale, ang_scale, Tur_lin_scale, Tur_ang_scale, srv_scale;
  int deadmanButton, lin_vel, ang_vel, srv_pos, turboButton, stopButton,
      servoButton;
  bool canMove;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPub;
  std_msgs::msg::Int16MultiArray SP_msg;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr ServoPose;
  rclcpp::TimerBase::SharedPtr timeout;
};

JoyTeleop::JoyTeleop() : Node("agribot_joystick") {
  joySub = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyTeleop::joyCallback, this, _1));
  twistPub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  ServoPose = this->create_publisher<std_msgs::msg::Int16MultiArray>("ServoPose", 100);

  updateParameters();
}

void JoyTeleop::joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
  // process and publish
  geometry_msgs::msg::Twist twistMsg;

  // check deadman switch
  bool switchActive = msg->buttons[deadmanButton];
  bool enable_turbo_button = msg->buttons[turboButton];
  bool stopActive = msg->buttons[stopButton];
  bool activeSrv = msg->buttons[servoButton];

  if (!stopActive) {
    if (switchActive) {
      twistMsg.linear.x = lin_scale * msg->axes[lin_vel];
      twistMsg.angular.z = ang_scale * msg->axes[ang_vel] * M_PI / 2;
      canMove = true;
      twistPub->publish(twistMsg);
    } else if (enable_turbo_button) {
      twistMsg.linear.x = Tur_lin_scale * msg->axes[lin_vel];
      twistMsg.angular.z = Tur_ang_scale * msg->axes[ang_vel] * M_PI / 2;
      canMove = true;
      twistPub->publish(twistMsg);
    } else if (activeSrv) {
      twistMsg.linear.y = 1;
      twistMsg.linear.x = lin_scale * msg->axes[lin_vel];
      twistMsg.angular.z = ang_scale * msg->axes[ang_vel] * M_PI / 2;
      canMove = true;
      twistPub->publish(twistMsg);
    } else if (canMove == true) {
      for (int i = 0; i < 5; ++i) {
        publishZeroMessage();
      }
      canMove = false;
    }
  } else {
    set_ServoPoses(900);  // stop
    for (int i = 0; i < 5; ++i) {
      publishZeroMessage();
    }
    canMove = false;
  }
}

void JoyTeleop::set_ServoPoses(float theta) {
  SP_msg.data.clear();
  SP_msg.data.push_back((int16_t)theta);
  SP_msg.data.push_back((int16_t)theta);
  ServoPose->publish(SP_msg);
}

void JoyTeleop::updateParameters() {
  this->declare_parameter("Tur_lin_scale", 2.0);
  this->declare_parameter("Tur_ang_scale", 1.0);
  this->declare_parameter("lin_scale", 0.2);
  this->declare_parameter("ang_scale", 0.5);
  this->declare_parameter("srv_scale", 90.0);
  this->declare_parameter("axis_angular", 2);
  this->declare_parameter("axis_linear", 1);
  this->declare_parameter("axis_servo", 0);
  this->declare_parameter("axis_turbo", 4);
  this->declare_parameter("axis_deadman", 5);
  this->declare_parameter("axis_stop", 7);

  Tur_lin_scale = this->get_parameter("Tur_lin_scale").as_double();
  Tur_ang_scale = this->get_parameter("Tur_ang_scale").as_double();
  lin_scale = this->get_parameter("lin_scale").as_double();
  ang_scale = this->get_parameter("ang_scale").as_double();
  srv_scale = this->get_parameter("srv_scale").as_double();
  ang_vel = this->get_parameter("axis_angular").as_int();
  lin_vel = this->get_parameter("axis_linear").as_int();
  srv_pos = this->get_parameter("axis_servo").as_int();
  turboButton = this->get_parameter("axis_turbo").as_int();
  deadmanButton = this->get_parameter("axis_deadman").as_int();
  stopButton = this->get_parameter("axis_stop").as_int();
  servoButton = this->get_parameter("axis_servo").as_int();
}

void JoyTeleop::timerCallback() {
  publishZeroMessage();
}

void JoyTeleop::publishZeroMessage() {
  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0;
  msg.angular.z = 0;
  twistPub->publish(msg);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyTeleop>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}