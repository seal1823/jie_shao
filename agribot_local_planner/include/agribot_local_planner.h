#ifndef ATTRACTOR_GUIDED_NAVIGATION_AGRIBOT_LOCAL_PLANNER_H_
#define ATTRACTOR_GUIDED_NAVIGATION_AGRIBOT_LOCAL_PLANNER_H_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_core/controller.hpp>

#include <fstream>
#include <string>
#include <vector>

namespace agribot_local_planner {

class AgribotLocalPlanner : public nav2_core::Controller {
 public:
  AgribotLocalPlanner();
  ~AgribotLocalPlanner();

  // nav2 local planner plugin functions
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                 std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path &path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &velocity,
      nav2_core::GoalChecker *goal_checker) override;

  void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

  void publishPlan(const std::vector<geometry_msgs::msg::PoseStamped>& path,
                   const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub);

  double getGoalPositionDistance(const geometry_msgs::msg::PoseStamped& global_pose,
                                 double goal_x, double goal_y);
  double getGoalOrientationAngleDifference(const geometry_msgs::msg::PoseStamped& global_pose,
                                           double goal_th);

  bool isGoalReached();

  std::vector<double> getEulerAngles(geometry_msgs::msg::PoseStamped& Pose);

  double LinearPIDController(nav_msgs::msg::Odometry& base_odometry,
                             double next_t_x, double next_t_y);
  double AngularPIDController(nav_msgs::msg::Odometry& base_odometry,
                              double target_th_w, double robot_orien);

 private:
  void emergencyStopCallback(const std_msgs::msg::Bool::ConstSharedPtr stop_msg);

  void robotStops() {
    goal_reached_ = true;
    RCLCPP_INFO(logger_, "Robot will stop.");
  }

  void getTransformedPosition(geometry_msgs::msg::PoseStamped &pose,
                              double *x, double *y, double *theta);

  // costmap
  nav2_costmap_2d::Costmap2DROS* costmap_ros_;
  nav2_costmap_2d::Costmap2D* costmap_;

  // robot state
  tf2::Vector3 robot_curr_pose;
  double robot_curr_orien;
  std::vector<double> final_orientation;

  // tf
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // publishers and subscribers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr curr_pose_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;

  // params
  std::string base_frame_;
  bool initialized_, goal_reached_, rotating_to_goal_;
  int plan_index_, last_plan_index_;
  std::vector<geometry_msgs::msg::PoseStamped> global_plan_;

  double p_window_, o_window_;
  double p_precision_, o_precision_;

  double max_vel_lin_, min_vel_lin_, max_incr_lin_;
  double max_vel_ang_, min_vel_ang_, max_incr_ang_;
  double k_p_lin_, k_i_lin_, k_d_lin_;
  double k_p_ang_, k_i_ang_, k_d_ang_;
  double d_t_;

  rclcpp::Time last_time;
  rclcpp::Logger logger_{rclcpp::get_logger("agribot_local_planner")};

  double error_lin_, error_ang_;
  double integral_lin_, integral_ang_;
};

}  // namespace agribot_local_planner

#endif  // ATTRACTOR_GUIDED_NAVIGATION_AGRIBOT_LOCAL_PLANNER_H_