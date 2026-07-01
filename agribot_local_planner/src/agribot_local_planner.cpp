#include "agribot_local_planner.h"
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_util/node_utils.hpp>

using namespace std;

// register planner
PLUGINLIB_EXPORT_CLASS(agribot_local_planner::AgribotLocalPlanner, nav2_core::Controller)

namespace agribot_local_planner {

constexpr double kControllerFrequency = 20.0;

AgribotLocalPlanner::AgribotLocalPlanner()
    : initialized_(false), goal_reached_(false) {

}

AgribotLocalPlanner::~AgribotLocalPlanner() {
}

void AgribotLocalPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {

  if (!initialized_) {
    auto node = parent.lock();
    logger_ = node->get_logger();
    tf_ = tf;
    costmap_ros_ = costmap_ros.get();
    costmap_ = costmap_ros_->getCostmap();

    // Declare and get parameters
    nav2_util::declare_parameter_if_not_declared(node, name + ".pos_window", rclcpp::ParameterValue(0.7));
    nav2_util::declare_parameter_if_not_declared(node, name + ".orient_window", rclcpp::ParameterValue(0.7));
    nav2_util::declare_parameter_if_not_declared(node, name + ".pos_precision", rclcpp::ParameterValue(0.2));
    nav2_util::declare_parameter_if_not_declared(node, name + ".orient_precision", rclcpp::ParameterValue(0.2));
    nav2_util::declare_parameter_if_not_declared(node, name + ".max_vel_lin", rclcpp::ParameterValue(0.4));
    nav2_util::declare_parameter_if_not_declared(node, name + ".min_vel_lin", rclcpp::ParameterValue(0.0));
    nav2_util::declare_parameter_if_not_declared(node, name + ".max_incr_lin", rclcpp::ParameterValue(0.3));
    nav2_util::declare_parameter_if_not_declared(node, name + ".max_vel_ang", rclcpp::ParameterValue(0.4));
    nav2_util::declare_parameter_if_not_declared(node, name + ".min_vel_ang", rclcpp::ParameterValue(-0.4));
    nav2_util::declare_parameter_if_not_declared(node, name + ".max_incr_ang", rclcpp::ParameterValue(0.25));
    nav2_util::declare_parameter_if_not_declared(node, name + ".k_p_lin", rclcpp::ParameterValue(2.0));
    nav2_util::declare_parameter_if_not_declared(node, name + ".k_i_lin", rclcpp::ParameterValue(0.04));
    nav2_util::declare_parameter_if_not_declared(node, name + ".k_d_lin", rclcpp::ParameterValue(0.0));
    nav2_util::declare_parameter_if_not_declared(node, name + ".k_p_ang", rclcpp::ParameterValue(2.0));
    nav2_util::declare_parameter_if_not_declared(node, name + ".k_i_ang", rclcpp::ParameterValue(0.0));
    nav2_util::declare_parameter_if_not_declared(node, name + ".k_d_ang", rclcpp::ParameterValue(0.0));

    node->get_parameter(name + ".pos_window", p_window_);
    node->get_parameter(name + ".orient_window", o_window_);
    node->get_parameter(name + ".pos_precision", p_precision_);
    node->get_parameter(name + ".orient_precision", o_precision_);
    node->get_parameter(name + ".max_vel_lin", max_vel_lin_);
    node->get_parameter(name + ".min_vel_lin", min_vel_lin_);
    node->get_parameter(name + ".max_incr_lin", max_incr_lin_);
    node->get_parameter(name + ".max_vel_ang", max_vel_ang_);
    node->get_parameter(name + ".min_vel_ang", min_vel_ang_);
    node->get_parameter(name + ".max_incr_ang", max_incr_ang_);
    node->get_parameter(name + ".k_p_lin", k_p_lin_);
    node->get_parameter(name + ".k_i_lin", k_i_lin_);
    node->get_parameter(name + ".k_d_lin", k_d_lin_);
    node->get_parameter(name + ".k_p_ang", k_p_ang_);
    node->get_parameter(name + ".k_i_ang", k_i_ang_);
    node->get_parameter(name + ".k_d_ang", k_d_ang_);

    // Create publishers
    target_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/target_pose", 10);
    curr_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/current_pose", 10);

    base_frame_ = "base_link";
    plan_index_ = 0;
    last_plan_index_ = 0;
    goal_reached_ = false;
    initialized_ = true;

    robot_curr_pose.setValue(0, 0, 0);
    robot_curr_orien = 0;

    // time interval
    double controller_freqency;
    node->get_parameter("controller_frequency", controller_freqency);
    d_t_ = 1.0 / controller_freqency;

    RCLCPP_INFO(logger_, "Agribot local planner initialized!");
  } else {
    RCLCPP_WARN(logger_, "Agribot local planner has already been initialized.");
  }
}

void AgribotLocalPlanner::cleanup() {
  RCLCPP_INFO(logger_, "Cleaning up agribot local planner");
}

void AgribotLocalPlanner::activate() {
  RCLCPP_INFO(logger_, "Activating agribot local planner");
}

void AgribotLocalPlanner::deactivate() {
  RCLCPP_INFO(logger_, "Deactivating agribot local planner");
}

// plugin functions
void AgribotLocalPlanner::setPlan(const nav_msgs::msg::Path &path){
  if (!initialized_) {
    RCLCPP_ERROR(logger_, "Agribot local planner has not been initialized.");
    return;
  }
  // set new plan
  global_plan_.clear();
  global_plan_ = path.poses;
  // reset plan parameters
  plan_index_ = 0;
  goal_reached_ = false;
  // reset pid
  integral_lin_ = integral_ang_ = 0.0;
  error_lin_ = error_ang_ = 0.0;
}

void AgribotLocalPlanner::setSpeedLimit(const double &speed_limit, const bool &percentage) {
  // Apply speed limit
  if (percentage) {
    max_vel_lin_ *= speed_limit;
  } else {
    max_vel_lin_ = speed_limit;
  }
}

void AgribotLocalPlanner::publishPlan(const std::vector<geometry_msgs::msg::PoseStamped>& path,
                                       const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub) {
  // given an empty path we won't do anything
  if (path.empty()) return;

  // create a path message
  nav_msgs::msg::Path gui_path;
  gui_path.poses.resize(path.size());
  gui_path.header.frame_id = path[0].header.frame_id;
  gui_path.header.stamp = rclcpp::Time(0);

  // Extract the plan in world co-ordinates, we assume the path is all in the
  // same frame
  for (unsigned int i = 0; i < path.size(); i++) {
    gui_path.poses[i] = path[i];
  }

  pub->publish(gui_path);
}

double AgribotLocalPlanner::getGoalPositionDistance(
    const geometry_msgs::msg::PoseStamped& global_pose, double goal_x, double goal_y) {
  return hypot(goal_x - global_pose.pose.position.x,
               goal_y - global_pose.pose.position.y);
}

double AgribotLocalPlanner::getGoalOrientationAngleDifference(
    const geometry_msgs::msg::PoseStamped& global_pose, double goal_th) {
  tf2::Quaternion q(
      global_pose.pose.orientation.x, global_pose.pose.orientation.y,
      global_pose.pose.orientation.z, global_pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  double angle = goal_th - yaw;
  double a = fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
  if (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  return a;
}

bool AgribotLocalPlanner::isGoalReached() {
  if (!initialized_) {
    RCLCPP_ERROR(logger_, "Agribot local planner has not been initialized.");
    return false;
  }
  if (goal_reached_) {
    RCLCPP_INFO(logger_, "Goal reached...");
    return true;
  }
  if (plan_index_ >= (int)global_plan_.size() - 1) {
    double orien_diff = fabs(final_orientation[2] - robot_curr_orien);
    double pos_dist = getGoalPositionDistance(global_plan_.back(),
        robot_curr_pose.x(), robot_curr_pose.y());
    if (orien_diff <= o_precision_ && pos_dist <= p_precision_) {
      goal_reached_ = true;
      robotStops();
      RCLCPP_INFO(logger_, "Goal has been reached!");
    }
  }
  return goal_reached_;
}

std::vector<double> AgribotLocalPlanner::getEulerAngles(geometry_msgs::msg::PoseStamped& Pose) {
  std::vector<double> EulerAngles;
  EulerAngles.resize(3, 0);
  tf2::Quaternion q(Pose.pose.orientation.x, Pose.pose.orientation.y,
                   Pose.pose.orientation.z, Pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(EulerAngles[0], EulerAngles[1], EulerAngles[2]);
  return EulerAngles;
}

double AgribotLocalPlanner::LinearPIDController(nav_msgs::msg::Odometry& base_odometry,
                                                  double next_t_x, double next_t_y) {
  double vel_curr = hypot(base_odometry.twist.twist.linear.y,
                          base_odometry.twist.twist.linear.x);
  double vel_target = hypot(next_t_x, next_t_y) / d_t_;

  if (fabs(vel_target) > max_vel_lin_) {
    vel_target = copysign(max_vel_lin_, vel_target);
  }

  double err_vel = vel_target - vel_curr;

  integral_lin_ += err_vel * d_t_;
  double derivative_lin = (err_vel - error_lin_) / d_t_;
  double incr_lin =
      k_p_lin_ * err_vel + k_i_lin_ * integral_lin_ + k_d_lin_ * derivative_lin;
  error_lin_ = err_vel;

  if (fabs(incr_lin) > max_incr_lin_) incr_lin = copysign(max_incr_lin_, incr_lin);

  double x_velocity = vel_curr + incr_lin;
  if (fabs(x_velocity) > max_vel_lin_) x_velocity = copysign(max_vel_lin_, x_velocity);
  if (fabs(x_velocity) < min_vel_lin_) x_velocity = copysign(min_vel_lin_, x_velocity);

  return x_velocity;
}

double AgribotLocalPlanner::AngularPIDController(nav_msgs::msg::Odometry& base_odometry,
                                                   double target_th_w, double robot_orien) {
  double orien_err = target_th_w - robot_orien;
  if(orien_err > M_PI) orien_err -= (2* M_PI);
  if(orien_err < -M_PI) orien_err += (2* M_PI);
  double target_vel_ang = (orien_err) / d_t_;
  if (fabs(target_vel_ang) > max_vel_ang_) {
    target_vel_ang = copysign(max_vel_ang_, target_vel_ang);
  }

  double vel_ang = base_odometry.twist.twist.angular.z;
  double error_ang = target_vel_ang - vel_ang;
  integral_ang_ += error_ang * d_t_;
  double derivative_ang = (error_ang - error_ang_) / d_t_;
  double incr_ang = k_p_ang_ * error_ang + k_i_ang_ * integral_ang_ +
                    k_d_ang_ * derivative_ang;
  error_ang_ = error_ang;

  if (fabs(incr_ang) > max_incr_ang_) incr_ang = copysign(max_incr_ang_, incr_ang);

  double th_velocity = copysign(vel_ang + incr_ang, target_vel_ang);
  if (fabs(th_velocity) > max_vel_ang_) th_velocity = copysign(max_vel_ang_, th_velocity);
  if (fabs(th_velocity) < min_vel_ang_) th_velocity = copysign(min_vel_ang_, th_velocity);

  return th_velocity;
}

geometry_msgs::msg::TwistStamped AgribotLocalPlanner::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker *goal_checker) {
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = base_frame_;
  cmd_vel.header.stamp = rclcpp::Time(0);

  if (!initialized_) {
    RCLCPP_ERROR(logger_, "PID planner has not been initialized.");
    return cmd_vel;
  }

  if (goal_reached_) {
    RCLCPP_INFO(logger_, "Goal reached without motion.");
    return cmd_vel;
  }

  // next target
  geometry_msgs::msg::PoseStamped target;

  double t_x, t_y, t_th;
  double x_vel = 0, th_vel = 0;
  double t_th_w = 0.0;

  // looking for the next point in the path far enough with minimum difference in angle
  while (plan_index_ < (int)global_plan_.size()) {
    target = global_plan_[plan_index_];
    int next_plan_index = min(((int)global_plan_.size()) - 1, plan_index_ + 1);
    t_th_w = atan2((global_plan_[next_plan_index].pose.position.y -
                    global_plan_[plan_index_].pose.position.y),
                   (global_plan_[next_plan_index].pose.position.x -
                    global_plan_[plan_index_].pose.position.x));
    tf2::Quaternion th_target_quat;
    th_target_quat.setRPY(0, 0, t_th_w);
    target.pose.orientation = tf2::toMsg(th_target_quat);
    getTransformedPosition(target, &t_x, &t_y, &t_th);

    if (hypot(t_x, t_y) > p_window_ || fabs(t_th) > o_window_) {
      break;
    }
    plan_index_++;
  }

  if (plan_index_ >= (int)global_plan_.size() - 1) {
    getTransformedPosition(global_plan_.back(), &t_x, &t_y, &t_th);
  }

  // get robot pose
  geometry_msgs::msg::PoseStamped global_pose;
  if (!costmap_ros_->getRobotPose(global_pose)) {
    RCLCPP_WARN(logger_, "Could not get robot pose");
    return cmd_vel;
  }

  robot_curr_pose.setValue(global_pose.pose.position.x,
                           global_pose.pose.position.y,
                           global_pose.pose.position.z);

  tf2::Quaternion q_orient(
      global_pose.pose.orientation.x, global_pose.pose.orientation.y,
      global_pose.pose.orientation.z, global_pose.pose.orientation.w);
  tf2::Matrix3x3 m_orient(q_orient);
  double roll, pitch, yaw;
  m_orient.getRPY(roll, pitch, yaw);
  robot_curr_orien = yaw;

  // get final goal orientation
  final_orientation = getEulerAngles(global_plan_.back());

  double pos_dist = getGoalPositionDistance(global_plan_.back(),
      robot_curr_pose.x(), robot_curr_pose.y());

  if (pos_dist <= p_precision_) {
    if (fabs(final_orientation[2] - robot_curr_orien) <= o_precision_) {
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.linear.y = 0.0;
      cmd_vel.twist.angular.z = 0.0;
      rotating_to_goal_ = false;
      goal_reached_ = true;
      RCLCPP_INFO(logger_, "pose and orientation are reached...");
    } else {
      // create fake odometry for angular PID
      nav_msgs::msg::Odometry base_odom;
      base_odom.twist.twist.angular.z = velocity.angular.z;
      th_vel = AngularPIDController(base_odom, final_orientation[2], robot_curr_orien);
      cmd_vel.twist.linear.x = 0;
      cmd_vel.twist.linear.y = 0;
      cmd_vel.twist.angular.z = th_vel;
    }
  } else {
    // create fake odometry for PID controllers
    nav_msgs::msg::Odometry base_odom;
    base_odom.twist.twist.linear.x = velocity.linear.x;
    base_odom.twist.twist.linear.y = velocity.linear.y;
    base_odom.twist.twist.angular.z = velocity.angular.z;

    x_vel = LinearPIDController(base_odom, t_x, t_y);

    if (plan_index_ >= (int)global_plan_.size() - 5) {
      t_th_w = final_orientation[2];
    }
    th_vel = AngularPIDController(base_odom, t_th_w, robot_curr_orien);

    cmd_vel.twist.linear.x = x_vel;
    cmd_vel.twist.linear.y = 0;
    cmd_vel.twist.angular.z = th_vel;
  }

  // publish next target pose
  target.header.frame_id = "map";
  target.header.stamp = rclcpp::Time(0);
  target_pose_pub_->publish(target);

  // publish robot pose
  geometry_msgs::msg::PoseStamped curr_pose_msg;
  curr_pose_msg.header.frame_id = "map";
  curr_pose_msg.header.stamp = rclcpp::Time(0);
  tf2::Quaternion curr_orien_quat;
  curr_orien_quat.setRPY(0, 0, robot_curr_orien);
  curr_pose_msg.pose.position.x = robot_curr_pose.x();
  curr_pose_msg.pose.position.y = robot_curr_pose.y();
  curr_pose_msg.pose.position.z = robot_curr_pose.z();
  curr_pose_msg.pose.orientation = tf2::toMsg(curr_orien_quat);
  curr_pose_pub_->publish(curr_pose_msg);

  return cmd_vel;
}

void AgribotLocalPlanner::getTransformedPosition(
    geometry_msgs::msg::PoseStamped &pose, double *x, double *y, double *theta) {
  try {
    geometry_msgs::msg::TransformStamped ps = tf_->lookupTransform(
        base_frame_, pose.header.frame_id, tf2::TimePointZero);
    *x = ps.transform.translation.x;
    *y = ps.transform.translation.y;
    tf2::Quaternion q(ps.transform.rotation.x, ps.transform.rotation.y,
                      ps.transform.rotation.z, ps.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    *theta = yaw;
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(logger_, "TF lookup failed: %s", ex.what());
    *x = 0; *y = 0; *theta = 0;
  }
}

void AgribotLocalPlanner::emergencyStopCallback(
    const std_msgs::msg::Bool::ConstSharedPtr stop_msg) {
  if (stop_msg->data) {
    goal_reached_ = true;
  }
}

};  // namespace agribot_local_planner