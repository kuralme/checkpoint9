#include "my_components/approach_server_component.hpp"
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace my_components {

AttachServer::AttachServer(const rclcpp::NodeOptions &options)
    : Node("approach_shelf_server", options), is_moving_to_cart_(false),
      is_moving_to_goal_(false), is_service_finished_(false) {

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&AttachServer::scan_callback, this, std::placeholders::_1),
      sub_options);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(),
      std::bind(&AttachServer::odom_callback, this, std::placeholders::_1),
      sub_options);

  approach_server_ = this->create_service<attach_shelf_srv::srv::GoToLoading>(
      "/approach_shelf",
      std::bind(&AttachServer::approach_shelf_callback, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);

  tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(200),
                              std::bind(&AttachServer::listen_cart_tf, this));

  shutdown_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&AttachServer::shutdown_cb, this));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);
  attach_pub_ =
      this->create_publisher<std_msgs::msg::String>("/elevator_up", 1);

  RCLCPP_INFO(this->get_logger(), "Approach shelf service ready.");
}

void AttachServer::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (msg->ranges.empty() || msg->intensities.empty()) {
    RCLCPP_WARN(this->get_logger(), "Scan data missing.");
    return;
  }
  laser_ranges_ = msg->ranges;
  laser_intensities_ = msg->intensities;
  angle_increment_ = msg->angle_increment;
  angle_min_ = msg->angle_min;
}
void AttachServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_odom_ = *msg;
}
void AttachServer::listen_cart_tf(void) {
  if (!is_moving_to_cart_)
    return;

  try {
    // Lookup the transform from cart_frame to robot
    geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform("odom", "cart_frame", tf2::TimePointZero);

    // Final approach location
    goal_pose2d_.x = transform.transform.translation.x;
    goal_pose2d_.y = transform.transform.translation.y;

  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not find transform: %s", ex.what());
  }
}
void AttachServer::shutdown_cb(void) {
  if (is_service_finished_)
    rclcpp::shutdown();
}

void AttachServer::approach_shelf_callback(
    const std::shared_ptr<attach_shelf_srv::srv::GoToLoading::Request> request,
    std::shared_ptr<attach_shelf_srv::srv::GoToLoading::Response> response) {

  RCLCPP_INFO(this->get_logger(),
              "Received Service Request: Robot will %s final approach.",
              request->attach_to_shelf ? "do" : "not do");

  float angle_range_deg = 45.0;
  float angle_range_rad = angle_range_deg * (M_PI / 180.0);
  size_t front_idx = 540;
  size_t start_idx =
      front_idx - static_cast<size_t>(angle_range_rad / angle_increment_);
  size_t end_idx =
      front_idx + static_cast<size_t>(angle_range_rad / angle_increment_);
  float intensity_threshold = 8000.;

  auto cluster_centers = find_intensity_cluster_centers(
      laser_intensities_, start_idx, end_idx, intensity_threshold);

  if (cluster_centers.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Less than 2 reflectors found.");
    response->complete = false;
    is_service_finished_ = true;
    return;
  }

  size_t idx1 = cluster_centers[0];
  size_t idx2 = cluster_centers[1];

  float angle1 = angle_min_ + idx1 * angle_increment_;
  float angle2 = angle_min_ + idx2 * angle_increment_;
  float x1 = laser_ranges_[idx1] * std::cos(angle1);
  float y1 = laser_ranges_[idx1] * std::sin(angle1);
  float x2 = laser_ranges_[idx2] * std::cos(angle2);
  float y2 = laser_ranges_[idx2] * std::sin(angle2);
  std::pair<float, float> midpoint = {(x1 + x2) / 2.0f, (y1 + y2) / 2.0f};

  broadcast_cart_frame(midpoint);

  RCLCPP_INFO(this->get_logger(), "Starting to move toward cart...");
  is_moving_to_cart_ = true;
  move_to_cart();

  response->complete = true;
  RCLCPP_INFO(this->get_logger(), "Final approach completed.");
  is_service_finished_ = true;
}

std::vector<int> AttachServer::find_intensity_cluster_centers(
    const std::vector<float> &intensities, size_t start_idx, size_t end_idx,
    float threshold = 8000.0) {
  std::vector<int> centers;
  int cluster_start = -1;

  for (size_t i = start_idx; i <= end_idx; ++i) {
    if (intensities[i] >= threshold) {
      if (cluster_start == -1) {
        cluster_start = static_cast<int>(i);
      }
    } else {
      if (cluster_start != -1) {
        int cluster_end = static_cast<int>(i - 1);
        int center = (cluster_start + cluster_end) / 2;
        centers.push_back(center);
        cluster_start = -1; // reset
      }
    }
  }
  // Handle case where cluster is still open at the end
  if (cluster_start != -1) {
    int cluster_end = static_cast<int>(end_idx);
    int center = (cluster_start + cluster_end) / 2;
    centers.push_back(center);
    RCLCPP_INFO(rclcpp::get_logger("find_intensity_cluster_centers"),
                "center-i (from end): %d", center);
  }
  return centers;
}
void AttachServer::broadcast_cart_frame(std::pair<float, float> cart_frame) {

  tf2::Transform tf_laser_to_odom;
  try {
    // Lookup the transform from robot laser to odom frame
    geometry_msgs::msg::TransformStamped tf_laser2odom_ =
        tf_buffer_->lookupTransform("odom", "robot_front_laser_base_link",
                                    tf2::TimePointZero);

    tf2::Quaternion laser_q(tf_laser2odom_.transform.rotation.x,
                            tf_laser2odom_.transform.rotation.y,
                            tf_laser2odom_.transform.rotation.z,
                            tf_laser2odom_.transform.rotation.w);
    tf_laser_to_odom.setOrigin(
        tf2::Vector3(tf_laser2odom_.transform.translation.x,
                     tf_laser2odom_.transform.translation.y,
                     tf_laser2odom_.transform.translation.z));
    tf_laser_to_odom.setRotation(laser_q);

    // Cart frame in odom frame
    tf2::Vector3 cart_in_laser(cart_frame.first, cart_frame.second, 0.0);
    tf2::Vector3 cart_in_odom = tf_laser_to_odom * cart_in_laser;

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "cart_frame";
    t.transform.translation.x = cart_in_odom.x();
    t.transform.translation.y = cart_in_odom.y();
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = tf_laser2odom_.transform.rotation.x;
    t.transform.rotation.y = tf_laser2odom_.transform.rotation.y;
    t.transform.rotation.z = tf_laser2odom_.transform.rotation.z;
    t.transform.rotation.w = tf_laser2odom_.transform.rotation.w;

    tf_broadcaster_->sendTransform(t);
    RCLCPP_INFO(this->get_logger(),
                "Published static cart_frame at (%.2f, %.2f)",
                t.transform.translation.x, t.transform.translation.y);

  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(),
                "Could not transform robot_front_laser_base_link to odom: %s",
                ex.what());
  }
}
void AttachServer::move_to_cart(void) {
  rclcpp::Rate rate(10);

  while (rclcpp::ok()) {

    if (is_moving_to_cart_) {
      // Move robot to cart loading position
      double delta_x = goal_pose2d_.x - robot_odom_.pose.pose.position.x;
      double delta_y = goal_pose2d_.y - robot_odom_.pose.pose.position.y;
      double cart_dist = std::sqrt(delta_x * delta_x + delta_y * delta_y);

      if (cart_dist > 0.05) {
        // Moving to cart_frame
        tf2::Quaternion quat(robot_odom_.pose.pose.orientation.x,
                             robot_odom_.pose.pose.orientation.y,
                             robot_odom_.pose.pose.orientation.z,
                             robot_odom_.pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        double angle_to_cart = std::atan2(delta_y, delta_x);
        double delta_theta = angle_to_cart - yaw;

        if (delta_theta > M_PI) {
          delta_theta -= 2 * M_PI;
        } else if (delta_theta < -M_PI) {
          delta_theta += 2 * M_PI;
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.1;
        cmd.angular.z = 1.5 * delta_theta;
        cmd_vel_pub_->publish(cmd);

        RCLCPP_INFO(this->get_logger(), "Cart distance: %.2fm ", cart_dist);

      } else {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);

        // Set new goal: Loading point 30 cm ahead
        tf2::Quaternion quat(robot_odom_.pose.pose.orientation.x,
                             robot_odom_.pose.pose.orientation.y,
                             robot_odom_.pose.pose.orientation.z,
                             robot_odom_.pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        goal_pose2d_.x += 0.3 * std::cos(yaw);
        goal_pose2d_.y += 0.3 * std::sin(yaw);

        is_moving_to_cart_ = false;
        is_moving_to_goal_ = true;
        RCLCPP_INFO(this->get_logger(), "New goal set 30cm ahead: (%.2f, %.2f)",
                    goal_pose2d_.x, goal_pose2d_.y);
      }

    } else if (is_moving_to_goal_) {
      // Move to the new goal
      double delta_x_final = goal_pose2d_.x - robot_odom_.pose.pose.position.x;
      double delta_y_final = goal_pose2d_.y - robot_odom_.pose.pose.position.y;
      double distance_final = std::sqrt(delta_x_final * delta_x_final +
                                        delta_y_final * delta_y_final);

      RCLCPP_INFO(this->get_logger(), "Final distance: %.2fm ", distance_final);

      if (distance_final > 0.05) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.1;
        cmd_vel_pub_->publish(cmd);
      } else {
        // Stop the robot at the final goal
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_cmd);

        // Publish msg to attach cart to robot
        std_msgs::msg::String msg;
        msg.data = "";
        attach_pub_->publish(msg);
        return;
      }
    }

    rate.sleep();
  }
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)
