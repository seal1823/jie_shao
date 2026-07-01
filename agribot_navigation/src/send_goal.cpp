#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

using namespace std;
using std::placeholders::_1;

class SendGoalNode : public rclcpp::Node {
 public:
  SendGoalNode() : Node("send_goals_node") {
    // Initialize publisher for move_base_simple/goal
    goal_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10);

    RCLCPP_INFO(this->get_logger(), "SendGoalNode initialized");
  }

  void sendGoal(double x, double y, double theta_deg) {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();

    goal.pose.position.x = x;
    goal.pose.position.y = y;

    double radians = theta_deg * (M_PI / 180);
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, radians);

    goal.pose.orientation = tf2::toMsg(quaternion);

    RCLCPP_INFO(this->get_logger(), "Sending goal to: x = %f, y = %f, theta = %f", x, y,
                theta_deg);
    goal_pub_->publish(goal);
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SendGoalNode>();

  double x = 1.0, y = 1.0, theta = 0.0;

  try {
    if (argc >= 4) {
      x = atof(argv[1]);
      y = atof(argv[2]);
      theta = atof(argv[3]);
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(node->get_logger(), "Using default values");
  }

  node->sendGoal(x, y, theta);

  // Use a timer to spin for a while to let the message be sent
  auto timer = node->create_wall_timer(
      std::chrono::seconds(2), [node]() { rclcpp::shutdown(); });

  rclcpp::spin(node);
  return 0;
}