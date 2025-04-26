#include "my_components/pre_approach_component.hpp"
#include <chrono>
#include <cmath>

namespace my_components {

PreApproach::PreApproach(const rclcpp::NodeOptions &options)
    : Node("pre_approach_node", options), rotating_(false),
      total_rotation_(0.0), initial_yaw_(0.0), current_yaw_(0.0),
      obstacle_dist_(0.3), goal_rotation_(-M_PI / 2.) {

  rclcpp::QoS qos_profile(
      rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
  qos_profile.reliable();

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos_profile,
      std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(),
      std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));
  pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PreApproach::preapproach_execute, this));

  move_cmd_.linear.x = 0.0;
  move_cmd_.angular.z = 0.0;
  RCLCPP_INFO(this->get_logger(), "Pre-Approach shelf component initialized.");
}

void PreApproach::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (msg->ranges.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty laser scan data.");
    return;
  }
  laser_ranges_ = msg->ranges;
}

void PreApproach::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  current_yaw_ = yaw;
}

void PreApproach::preapproach_execute(void) {
  if (laser_ranges_.empty()) {
    RCLCPP_WARN(this->get_logger(), "Laser data not received yet.");
    return;
  }

  float laser_forward = get_min_distance(laser_ranges_, 500, 580);

  if (!rotating_ && laser_forward > obstacle_dist_) {
    // Move forward
    move_cmd_.linear.x = 0.3;
    move_cmd_.angular.z = 0.0;

  } else if (!rotating_ && laser_forward <= obstacle_dist_) {
    // Start rotating
    RCLCPP_INFO(this->get_logger(), "Reached wall, starting rotation.");
    move_cmd_.linear.x = 0.0;
    move_cmd_.angular.z = -0.4;
    rotating_ = true;
    total_rotation_ = 0.0;
    initial_yaw_ = current_yaw_;

  } else if (rotating_) {
    // Check rotation
    double delta_yaw = current_yaw_ - initial_yaw_;
    if (delta_yaw > M_PI)
      delta_yaw -= 2 * M_PI;
    if (delta_yaw < -M_PI)
      delta_yaw += 2 * M_PI;
    RCLCPP_INFO(this->get_logger(), "Heading: %.2f / %.2f",
                delta_yaw * 180.0 / M_PI, goal_rotation_ * 180.0 / M_PI);

    if (std::abs(delta_yaw) >= std::abs(goal_rotation_)) {
      RCLCPP_INFO(this->get_logger(), "Finished rotating.");
      move_cmd_.linear.x = 0.0;
      move_cmd_.angular.z = 0.0;
      rotating_ = false;
      pub_timer_->cancel();

      // Terminate program
      rclcpp::shutdown();
    }
  }
  cmd_vel_pub_->publish(move_cmd_);
}

float PreApproach::get_min_distance(const std::vector<float> &ranges,
                                    size_t start_idx, size_t end_idx) {
  auto it =
      std::min_element(ranges.begin() + start_idx, ranges.begin() + end_idx,
                       [](float a, float b) { return a > 0.0f && a < b; });

  return (it != ranges.begin() + end_idx)
             ? *it
             : std::numeric_limits<float>::infinity();
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)