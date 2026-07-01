#include "agribot_visual_servoing/crop_row_detector.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <cmath>
#include <map>
#include <memory>
#include <string>

enum class NavState {
  FOLLOW_AISLE,      // drive along current aisle (forward or backward)
  TURN,              // in-place turn toward a target yaw
  CROSS_AISLE,       // drive perpendicular across row ends to next aisle
  STOP,              // brief brake before turning to avoid aisle overshoot
  DONE               // finished all aisles
};

class VisualServoingNode : public rclcpp::Node {
public:
  VisualServoingNode() : Node("agribot_vs"), state_(NavState::FOLLOW_AISLE), current_aisle_idx_(0), drive_dir_(+1) {
    // Declare parameters
    this->declare_parameter<std::string>("camera_topic", "/front_camera/image_raw");
    this->declare_parameter<std::string>("back_camera_topic", "/back_camera/image_raw");
    this->declare_parameter<double>("linear_velocity", 0.3);
    this->declare_parameter<double>("angular_gain", 0.8);
    this->declare_parameter<double>("lateral_gain", 0.5);
    this->declare_parameter<int>("green_h_low", 35);
    this->declare_parameter<int>("green_s_low", 50);
    this->declare_parameter<int>("green_v_low", 50);
    this->declare_parameter<int>("green_h_high", 85);
    this->declare_parameter<int>("green_s_high", 255);
    this->declare_parameter<int>("green_v_high", 255);
    this->declare_parameter<double>("max_angular_vel", 0.8);
    this->declare_parameter<double>("min_angular_vel", 0.05);
    this->declare_parameter<std::string>("pose_topic", "/pose");
    this->declare_parameter<double>("row_end_x", 3.0);      // end of row in +x
    this->declare_parameter<double>("row_start_x", -5.5);   // end of row in -x
    this->declare_parameter<double>("heading_gain", 0.5);
    // U-path parameters
    this->declare_parameter<std::vector<double>>("aisle_y_positions", std::vector<double>{-2.0, 0.0, 2.0});
    this->declare_parameter<double>("u_path_drive_speed", 0.3);
    this->declare_parameter<double>("u_path_turn_speed", 0.4);
    this->declare_parameter<double>("u_path_turn_tolerance", 0.15);
    this->declare_parameter<double>("u_path_y_tolerance", 0.3);
    // Centerline stability parameters
    this->declare_parameter<double>("min_lane_width_px", 30.0);
    this->declare_parameter<double>("max_lane_width_px", 220.0);
    this->declare_parameter<double>("max_center_jitter", 0.25);
    this->declare_parameter<double>("min_confidence", 0.2);
    // Control loop rate (Hz).  The state machine runs at this rate on a
    // timer, independent of the image callback rate, so the robot keeps
    // moving even if camera frames stop arriving.
    this->declare_parameter<double>("control_rate_hz", 20.0);

    // Get parameters
    std::string camera_topic = this->get_parameter("camera_topic").as_string();
    std::string back_camera_topic = this->get_parameter("back_camera_topic").as_string();
    pose_topic_ = this->get_parameter("pose_topic").as_string();
    row_end_x_ = this->get_parameter("row_end_x").as_double();
    row_start_x_ = this->get_parameter("row_start_x").as_double();
    heading_gain_ = this->get_parameter("heading_gain").as_double();
    linear_vel_ = this->get_parameter("linear_velocity").as_double();
    angular_gain_ = this->get_parameter("angular_gain").as_double();
    lateral_gain_ = this->get_parameter("lateral_gain").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
    min_angular_vel_ = this->get_parameter("min_angular_vel").as_double();
    // Aisle / snake-path parameters
    aisle_y_positions_ = this->get_parameter("aisle_y_positions").as_double_array();
    // Sort ascending so snake path goes from low y to high y
    std::sort(aisle_y_positions_.begin(), aisle_y_positions_.end());
    u_path_drive_speed_ = this->get_parameter("u_path_drive_speed").as_double();
    u_path_turn_speed_ = this->get_parameter("u_path_turn_speed").as_double();
    u_path_turn_tol_ = this->get_parameter("u_path_turn_tolerance").as_double();
    u_path_y_tol_ = this->get_parameter("u_path_y_tolerance").as_double();

    // Configure both detectors (front and back) with the same parameters.
    // They are separate instances so the EMA / prediction state for the
    // front camera does not get polluted by back-camera frames (and vice
    // versa, which would happen if they shared state).
    //
    // Use get_parameter_or for the parameters that don't have explicit
    // declare_parameter calls (min_contour_area, morph_kernel_size) so a
    // missing or un-declared parameter doesn't kill the node constructor.
    auto configure_detector = [this](CropRowDetector &d) {
      d.setGreenLower(
        this->get_parameter("green_h_low").as_int(),
        this->get_parameter("green_s_low").as_int(),
        this->get_parameter("green_v_low").as_int());
      d.setGreenUpper(
        this->get_parameter("green_h_high").as_int(),
        this->get_parameter("green_s_high").as_int(),
        this->get_parameter("green_v_high").as_int());
      d.setMinContourArea(this->get_parameter_or("min_contour_area", 80.0));
      d.setMorphKernelSize(this->get_parameter_or("morph_kernel_size", 7));
      d.setMinLaneWidthPx(static_cast<float>(this->get_parameter("min_lane_width_px").as_double()));
      d.setMaxLaneWidthPx(static_cast<float>(this->get_parameter("max_lane_width_px").as_double()));
      d.setMaxCenterJitter(static_cast<float>(this->get_parameter("max_center_jitter").as_double()));
      d.setMinConfidence(static_cast<float>(this->get_parameter("min_confidence").as_double()));
    };
    configure_detector(front_detector_);
    configure_detector(back_detector_);

    // Setup subscribers
    front_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      camera_topic, rclcpp::QoS(1).reliable(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        this->imageCallback(msg, true);
      });
    back_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      back_camera_topic, rclcpp::QoS(1).reliable(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        this->imageCallback(msg, false);
      });
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic_, 10,
      std::bind(&VisualServoingNode::poseCallback, this, std::placeholders::_1));

    // Setup publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    annotated_image_pubs_["front"] = this->create_publisher<sensor_msgs::msg::Image>(
      "/visual_servoing/annotated_image_front", 10);
    annotated_image_pubs_["back"] = this->create_publisher<sensor_msgs::msg::Image>(
      "/visual_servoing/annotated_image_back", 10);
    path_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/visual_servoing/path_markers", 10);
    mask_image_pubs_["front"] = this->create_publisher<sensor_msgs::msg::Image>(
      "/visual_servoing/vegetation_mask_front", 10);
    mask_image_pubs_["back"] = this->create_publisher<sensor_msgs::msg::Image>(
      "/visual_servoing/vegetation_mask_back", 10);

    // Periodic control tick: drives the state machine at a fixed rate so
    // the robot keeps moving even if image callbacks stop arriving.
    double control_rate_hz = this->get_parameter("control_rate_hz").as_double();
    if (control_rate_hz < 1.0) control_rate_hz = 1.0;
    auto period = std::chrono::duration<double>(1.0 / control_rate_hz);
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&VisualServoingNode::controlTick, this));

    RCLCPP_INFO(this->get_logger(), "Visual Servoing Node started");
    RCLCPP_INFO(this->get_logger(), "Camera topic: %s", camera_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Linear velocity: %.2f m/s", linear_vel_);
    RCLCPP_INFO(this->get_logger(), "Angular gain: %.2f", angular_gain_);
    RCLCPP_INFO(this->get_logger(), "Lateral gain: %.2f", lateral_gain_);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg, bool is_front) {
    // Convert ROS image to OpenCV
    cv::Mat rgb_image;
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      rgb_image = cv_ptr->image;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    if (rgb_image.empty()) return;

    // Detect crop rows.  Use a separate detector per camera so the EMA
    // and prediction state for the front camera do not get polluted by
    // back-camera frames (the two cameras see mirrored images, so mixing
    // their EMAs would produce inconsistent lateral-error signs).
    CropRowDetector &detector = is_front ? front_detector_ : back_detector_;
    CropRowResult result = detector.detect(rgb_image);

    // Create annotation image
    cv::Mat annotated = rgb_image.clone();
    detector.drawResults(annotated, result);

    // Publish annotated image with camera-specific suffix
    std::string suffix = is_front ? "front" : "back";
    publishAnnotatedImage(annotated, msg->header, suffix);

    // Publish vegetation mask
    if (!result.mask.empty()) {
      publishMaskImage(result.mask, msg->header, suffix);
    }

    // Store result for the control loop
    if (is_front) {
      front_result_ = result;
      front_result_.header = msg->header;
      front_result_valid_ = true;
    } else {
      back_result_ = result;
      back_result_.header = msg->header;
      back_result_valid_ = true;
    }

    // Periodic detection debug log — tells the user how many rows the
    // detector is seeing and what it decided to do with them.
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[DETECT %s] rows=%zu conf=%.2f%s%s%s",
      is_front ? "front" : "back ",
      result.row_lines.size(),
      result.confidence,
      result.predicted ? " PRED" : "",
      result.reject_reason.empty() ? "" : " reject=",
      result.reject_reason.c_str());

    // Pick the active result for the current navigation state
    CropRowResult active_result;
    if (state_ == NavState::FOLLOW_AISLE && drive_dir_ == +1) {
      active_result = front_result_;
    } else if (state_ == NavState::FOLLOW_AISLE && drive_dir_ == -1) {
      active_result = back_result_;
    }
    const std_msgs::msg::Header &active_header = active_result.header;
    (void)active_header;

    // Drive the state machine on a wall timer (see controlTick) so the
    // robot keeps moving even if image callbacks stop arriving.  Here we
    // only publish path markers when we have a fresh detection.
    if (active_result.detected) {
      publishPathMarkers(active_result, active_header);
    }
  }

  void controlTick() {
    // Pick the active result for the current navigation state
    CropRowResult active_result;
    if (state_ == NavState::FOLLOW_AISLE && drive_dir_ == +1) {
      active_result = front_result_;
    } else if (state_ == NavState::FOLLOW_AISLE && drive_dir_ == -1) {
      active_result = back_result_;
    }
    const std_msgs::msg::Header &active_header = active_result.header;
    (void)active_header;

    // State machine for snake (zigzag) row coverage
    geometry_msgs::msg::Twist cmd;

    switch (state_) {
      case NavState::FOLLOW_AISLE: {
        bool reached_end = (drive_dir_ == +1) ? reachedRowEnd() : reachedRowStart();
        if (reached_end) {
          if (current_aisle_idx_ + 1 < static_cast<int>(aisle_y_positions_.size())) {
            // Turn to cross direction (+y since aisles are sorted ascending)
            turn_target_yaw_ = M_PI_2;
            next_state_after_turn_ = NavState::CROSS_AISLE;
            RCLCPP_INFO(this->get_logger(),
              "Aisle %d done (y=%.2f), turning to cross to next aisle (y=%.2f)",
              current_aisle_idx_, aisle_y_positions_[current_aisle_idx_],
              aisle_y_positions_[current_aisle_idx_ + 1]);
            state_ = NavState::TURN;
          } else {
            RCLCPP_INFO(this->get_logger(),
              "Last aisle finished, stopping.");
            state_ = NavState::DONE;
          }
          cmd.linear.x = 0.0;
          cmd.angular.z = 0.0;
          cmd_vel_pub_->publish(cmd);
          return;
        }

        if (active_result.detected) {
          publishPathMarkers(active_result, active_header);
        }

        if (drive_dir_ == +1 && active_result.detected) {
          // Forward driving: front camera, invert errors (see computeVisualServoCommand)
          cmd = computeVisualServoCommand(active_result, -1.0, drive_dir_);
        } else if (drive_dir_ == -1 && back_result_.detected) {
          // Backward driving: use the back camera so we still get lateral
          // correction in reverse.  Sign flip matches the forward case
          // (we still want the lane-center to be at image center, but the
          // back camera is mirrored, so we invert the steering command).
          cmd = computeVisualServoCommand(back_result_, +1.0, drive_dir_);
        } else {
          // No detection from either camera — drive at a reduced speed
          // and rely on heading keeper only.
          cmd.linear.x = drive_dir_ * linear_vel_ * 0.5;
          cmd.angular.z = 0.0;
        }
        // Heading keeper: keep yaw aligned with current drive direction
        if (current_pose_set_) {
          double target_yaw = (drive_dir_ == +1) ? 0.0 : M_PI;
          double heading_error = normalizeAngle(current_yaw_ - target_yaw);
          cmd.angular.z -= heading_gain_ * heading_error;
          if (cmd.angular.z > max_angular_vel_) cmd.angular.z = max_angular_vel_;
          if (cmd.angular.z < -max_angular_vel_) cmd.angular.z = -max_angular_vel_;
        }
        break;
      }

      case NavState::TURN: {
        double error = std::abs(normalizeAngle(current_yaw_ - turn_target_yaw_));
        double delta = normalizeAngle(turn_target_yaw_ - current_yaw_);
        double angular_cmd = (delta > 0.0) ? u_path_turn_speed_ : -u_path_turn_speed_;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
          "[TURN] yaw=%.3f target=%.3f delta=%.3f drive_dir=%d -> angular.z=%.3f",
          current_yaw_, turn_target_yaw_, delta, drive_dir_, angular_cmd);
        if (error < u_path_turn_tol_) {
          RCLCPP_INFO(this->get_logger(),
            "TURN complete (yaw=%.3f), entering state %d",
            current_yaw_, static_cast<int>(next_state_after_turn_));
          state_ = next_state_after_turn_;
          cmd.linear.x = 0.0;
          cmd.angular.z = 0.0;
        } else {
          cmd.linear.x = 0.0;
          // Shortest-direction turn
          cmd.angular.z = angular_cmd;
        }
        break;
      }

      case NavState::CROSS_AISLE: {
        double target_y = aisle_y_positions_[current_aisle_idx_ + 1];
        double remaining = current_pose_.position.y - target_y;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "[DBG] CROSS_AISLE y=%.2f target=%.2f remaining=%.2f",
          current_pose_.position.y, target_y, remaining);
        // Stop when we reach or slightly overshoot the target aisle.  Using
        // `remaining >= -tol` handles inertia overshoot that `abs(remaining)`
        // alone would miss.
        if (remaining >= -u_path_y_tol_) {
          current_aisle_idx_++;
          drive_dir_ *= -1;  // alternate drive direction for snake pattern
          double target_yaw = (drive_dir_ == +1) ? 0.0 : M_PI;
          turn_target_yaw_ = target_yaw;
          next_state_after_turn_ = NavState::FOLLOW_AISLE;
          RCLCPP_INFO(this->get_logger(),
            "CROSS_AISLE complete (y=%.3f target=%.3f remaining=%.3f) -> aisle=%d drive_dir=%d turn_target=%.3f",
            current_pose_.position.y, target_y, remaining, current_aisle_idx_, drive_dir_, turn_target_yaw_);
          // Brake before turning so inertia does not carry us past the aisle.
          next_state_after_stop_ = NavState::TURN;
          state_ = NavState::STOP;
          stop_start_time_ = this->now();
          cmd.linear.x = 0.0;
          cmd.angular.z = 0.0;
        } else {
          cmd.linear.x = u_path_drive_speed_;
          // Heading keeper: maintain yaw ≈ +π/2 (facing +y)
          if (current_pose_set_) {
            double heading_error = normalizeAngle(current_yaw_ - M_PI_2);
            cmd.angular.z = -heading_gain_ * heading_error;
            if (cmd.angular.z > max_angular_vel_) cmd.angular.z = max_angular_vel_;
            if (cmd.angular.z < -max_angular_vel_) cmd.angular.z = -max_angular_vel_;
          } else {
            cmd.angular.z = 0.0;
          }
        }
        break;
      }

      case NavState::STOP: {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        double speed = current_twist_set_
                         ? std::hypot(current_twist_.linear.x, current_twist_.linear.y)
                         : 999.0;
        bool timed_out = (this->now() - stop_start_time_).seconds() > 1.0;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
          "[STOP] speed=%.3f (timeout=%d)", speed, timed_out);
        if (speed < 0.05 || timed_out) {
          RCLCPP_INFO(this->get_logger(),
            "STOP done (speed=%.3f), entering state %d", speed,
            static_cast<int>(next_state_after_stop_));
          state_ = next_state_after_stop_;
        }
        break;
      }

      case NavState::DONE:
      default:
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        break;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "State=%d, cmd: linear=%.3f, angular=%.3f, lat=%.3f, ang=%.3f, conf=%.2f%s%s%s",
      static_cast<int>(state_), cmd.linear.x, cmd.angular.z,
      active_result.lateral_error, active_result.angular_error,
      active_result.confidence,
      active_result.predicted ? " PRED" : "",
      active_result.reject_reason.empty() ? "" : " reject=",
      active_result.reject_reason.c_str());

    cmd_vel_pub_->publish(cmd);
  }

  geometry_msgs::msg::Twist computeVisualServoCommand(const CropRowResult &result,
                                                    double sign = 1.0,
                                                    int drive_dir = +1) {
    geometry_msgs::msg::Twist cmd;
    // Front camera: image x right corresponds to robot right. A positive
    // lateral error (lane to the right of image center) means the robot is
    // too far left and needs to turn right, which is negative angular.z in
    // the standard ROS convention. Pass sign=-1.0 for the front camera.
    double ang_z = sign * (angular_gain_ * result.angular_error
                         + lateral_gain_ * result.lateral_error);
    if (std::abs(ang_z) > max_angular_vel_) {
      ang_z = (ang_z > 0) ? max_angular_vel_ : -max_angular_vel_;
    }
    if (std::abs(ang_z) < min_angular_vel_) {
      ang_z = 0.0;
    }
    cmd.linear.x = drive_dir * linear_vel_;
    cmd.angular.z = ang_z;
    return cmd;
  }

  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_ = msg->pose.pose;
    current_yaw_ = quaternionToYaw(current_pose_.orientation);
    current_twist_ = msg->twist.twist;
    current_pose_set_ = true;
    current_twist_set_ = true;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Pose updated: x=%.2f, y=%.2f, yaw=%.2f, speed=%.3f",
      current_pose_.position.x, current_pose_.position.y, current_yaw_,
      std::hypot(current_twist_.linear.x, current_twist_.linear.y));
  }

  static double quaternionToYaw(const geometry_msgs::msg::Quaternion &q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  bool reachedRowEnd() const {
    if (!current_pose_set_) return false;
    return current_pose_.position.x > row_end_x_;
  }

  bool reachedRowStart() const {
    if (!current_pose_set_) return false;
    return current_pose_.position.x < row_start_x_;
  }

  static double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  void publishAnnotatedImage(const cv::Mat &annotated,
                             const std_msgs::msg::Header &header,
                             const std::string &suffix) {
    cv_bridge::CvImage cv_image;
    cv_image.header = header;
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image.image = annotated;
    auto pub_it = annotated_image_pubs_.find(suffix);
    if (pub_it != annotated_image_pubs_.end()) {
      pub_it->second->publish(*cv_image.toImageMsg());
    }
  }

  void publishMaskImage(const cv::Mat &mask,
                        const std_msgs::msg::Header &header,
                        const std::string &suffix) {
    cv_bridge::CvImage cv_image;
    cv_image.header = header;
    cv_image.encoding = sensor_msgs::image_encodings::MONO8;
    cv_image.image = mask;
    auto pub_it = mask_image_pubs_.find(suffix);
    if (pub_it != mask_image_pubs_.end()) {
      pub_it->second->publish(*cv_image.toImageMsg());
    }
  }

  void publishPathMarkers(const CropRowResult &result,
                          const std_msgs::msg::Header &header) {
    visualization_msgs::msg::MarkerArray markers;

    // Marker 1: Path centerline as a line strip
    visualization_msgs::msg::Marker line_marker;
    line_marker.header = header;
    line_marker.header.frame_id = "camera_link_front"; // image frame
    line_marker.ns = "planned_path";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.05; // line width

    // Red color for planned path
    line_marker.color.r = 1.0;
    line_marker.color.g = 0.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;

    // Convert image coordinates to 3D points (simplified: project to a plane in front of camera)
    // Using a simple pinhole model: x = (u - cx) * Z / fx, y = (v - cy) * Z / fy
    // We place points at different depths Z
    float cx = 320.0f, cy = 240.0f;
    float fx = 500.0f, fy = 500.0f; // approximate focal lengths

    // Create points along the centerline at different depths
    cv::Point2f top = result.center_line.first;
    cv::Point2f bottom = result.center_line.second;

    for (float t = 0; t <= 1.0f; t += 0.1f) {
      float u = top.x + (bottom.x - top.x) * t;
      float v = top.y + (bottom.y - top.y) * t;
      float Z = 0.5f + t * 3.0f; // depth: 0.5m to 3.5m

      geometry_msgs::msg::Point p;
      p.x = (u - cx) * Z / fx;
      p.y = (v - cy) * Z / fy;
      p.z = Z;
      line_marker.points.push_back(p);
    }

    markers.markers.push_back(line_marker);

    // Marker 2: Detected crop rows as lines
    for (size_t i = 0; i < result.row_lines.size(); ++i) {
      visualization_msgs::msg::Marker row_marker;
      row_marker.header = header;
      row_marker.header.frame_id = "camera_link_front";
      row_marker.ns = "detected_rows";
      row_marker.id = static_cast<int>(i);
      row_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      row_marker.action = visualization_msgs::msg::Marker::ADD;
      row_marker.scale.x = 0.03;

      // Green for detected rows
      row_marker.color.r = 0.0;
      row_marker.color.g = 1.0;
      row_marker.color.b = 0.0;
      row_marker.color.a = 0.8;

      const auto &line = result.row_lines[i];
      for (float t = 0; t <= 1.0f; t += 0.1f) {
        float u = line.first.x + (line.second.x - line.first.x) * t;
        float v = line.first.y + (line.second.y - line.first.y) * t;
        float Z = 0.5f + t * 3.0f;

        geometry_msgs::msg::Point p;
        p.x = (u - cx) * Z / fx;
        p.y = (v - cy) * Z / fy;
        p.z = Z;
        row_marker.points.push_back(p);
      }

      markers.markers.push_back(row_marker);
    }

    path_markers_pub_->publish(markers);
  }

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr front_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr back_image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // State
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> annotated_image_pubs_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> mask_image_pubs_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_markers_pub_;

  // Detector
  CropRowDetector front_detector_;
  CropRowDetector back_detector_;

  // Stored detection results
  CropRowResult front_result_;
  CropRowResult back_result_;
  bool front_result_valid_ = false;
  bool back_result_valid_ = false;

  // Control parameters
  double linear_vel_;
  double angular_gain_;
  double lateral_gain_;
  double max_angular_vel_;
  double min_angular_vel_;

  // State machine for U-path coverage
  NavState state_;
  geometry_msgs::msg::Pose current_pose_;
  double current_yaw_ = 0.0;
  bool current_pose_set_ = false;
  geometry_msgs::msg::Twist current_twist_;
  bool current_twist_set_ = false;

  // Row-end / turn parameters
  std::string pose_topic_;
  double row_end_x_;
  double row_start_x_;
  double heading_gain_;

  // Snake-path parameters
  std::vector<double> aisle_y_positions_;
  int current_aisle_idx_;
  int drive_dir_ = +1;            // +1 for +x, -1 for -x
  double u_path_drive_speed_;
  double u_path_turn_speed_;
  double u_path_turn_tol_;
  double u_path_y_tol_;
  double turn_target_yaw_ = 0.0;
  NavState next_state_after_turn_ = NavState::FOLLOW_AISLE;
  NavState next_state_after_stop_ = NavState::FOLLOW_AISLE;
  rclcpp::Time stop_start_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisualServoingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}