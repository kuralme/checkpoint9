#include "attach_shelf_srv/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

class ApproachShelfServer : public rclcpp::Node {
public:
  ApproachShelfServer()
      : Node("approach_shelf_server"), midpoint_(0.0, 0.0),
        is_moving_to_cart_(false), is_moving_to_goal_(false) {

    approach_server_ = this->create_service<attach_shelf_srv::srv::GoToLoading>(
        "/approach_shelf",
        std::bind(&ApproachShelfServer::approach_shelf_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&ApproachShelfServer::scan_callback, this,
                  std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(),
        std::bind(&ApproachShelfServer::odom_callback, this,
                  std::placeholders::_1));

    tf_broadcaster_ =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&ApproachShelfServer::listen_tf, this));
    pub_vel_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ApproachShelfServer::move_to_cart, this));
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 10);

    move_cmd_.linear.x = 0.0;
    move_cmd_.angular.z = 0.0;
    RCLCPP_INFO(this->get_logger(), "Approach shelf service ready.");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (msg->ranges.empty() || msg->intensities.empty()) {
      RCLCPP_WARN(this->get_logger(), "Scan data missing.");
      return;
    }
    laser_ranges_ = msg->ranges;
    laser_intensities_ = msg->intensities;
    angle_increment_ = msg->angle_increment;
    angle_min_ = msg->angle_min;
  }
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_odom_ = *msg;
  }
  void listen_tf() {
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
      RCLCPP_WARN(this->get_logger(), "Could not transform cart_frame: %s",
                  ex.what());
    }
  }

  void approach_shelf_callback(
      const std::shared_ptr<attach_shelf_srv::srv::GoToLoading::Request>
          request,
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
    midpoint_ = {(x1 + x2) / 2.0f, (y1 + y2) / 2.0f};

    RCLCPP_INFO(this->get_logger(), "idx1,2 (%d, %d)", idx1, idx2);
    RCLCPP_INFO(this->get_logger(), "range1: %d", laser_ranges_[idx1]);
    RCLCPP_INFO(this->get_logger(), "range2: %d", laser_ranges_[idx2]);
    RCLCPP_INFO(this->get_logger(), "cluster1 (%.2f, %.2f)", x1, y1);
    RCLCPP_INFO(this->get_logger(), "cluster2 (%.2f, %.2f)", x2, y2);
    RCLCPP_INFO(this->get_logger(), "cart_frame from laser (%.2f, %.2f)",
                midpoint_.first, midpoint_.second);

    broadcast_cart_frame(x1, x2, y1, y2);

    if (request->attach_to_shelf) {
      pending_response_ = response;
      is_moving_to_cart_ = true;

    } else {
      RCLCPP_INFO(this->get_logger(), "No approach has done.");
      response->complete = true;
    }
  }

  std::vector<int>
  find_intensity_cluster_centers(const std::vector<float> &intensities,
                                 size_t start_idx, size_t end_idx,
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

  void broadcast_cart_frame(float x1, float x2, float y1, float y2) {

    tf2::Transform tf_robot_to_odom;
    tf_robot_to_odom.setOrigin(tf2::Vector3(robot_odom_.pose.pose.position.x,
                                            robot_odom_.pose.pose.position.y,
                                            0.0));
    tf2::Quaternion robot_q(robot_odom_.pose.pose.orientation.x,
                            robot_odom_.pose.pose.orientation.y,
                            robot_odom_.pose.pose.orientation.z,
                            robot_odom_.pose.pose.orientation.w);
    tf_robot_to_odom.setRotation(robot_q);

    // NEED TO ADD MIDPOINTS FROM robot_front_laser_base_link NOT robot_odom

    tf2::Vector3 cart_in_robot(midpoint_.first, midpoint_.second, 0.0);
    tf2::Vector3 cart_in_odom = tf_robot_to_odom * cart_in_robot;

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "cart_frame";
    t.transform.translation.x = cart_in_odom.x();
    t.transform.translation.y = cart_in_odom.y();
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = robot_odom_.pose.pose.orientation.x;
    t.transform.rotation.y = robot_odom_.pose.pose.orientation.y;
    t.transform.rotation.z = robot_odom_.pose.pose.orientation.z;
    t.transform.rotation.w = robot_odom_.pose.pose.orientation.w;

    tf_broadcaster_->sendTransform(t);
    RCLCPP_INFO(this->get_logger(),
                "Published static cart_frame at (%.2f, %.2f)",
                t.transform.translation.x, t.transform.translation.y);

    // ------------------- Debug frames -------------------
    geometry_msgs::msg::TransformStamped t1;
    t1.header.stamp = this->now();
    t1.header.frame_id = "robot_front_laser_base_link";
    t1.child_frame_id = "cluster1_frame";
    t1.transform.translation.x = x1;
    t1.transform.translation.y = y1;
    t1.transform.translation.z = 0.0;
    tf2::Quaternion q1;
    q1.setRPY(0, 0, 0);
    t1.transform.rotation.x = q1.x();
    t1.transform.rotation.y = q1.y();
    t1.transform.rotation.z = q1.z();
    t1.transform.rotation.w = q1.w();
    tf_broadcaster_->sendTransform(t1);
    geometry_msgs::msg::TransformStamped t2;
    t2.header.stamp = this->now();
    t2.header.frame_id = "robot_front_laser_base_link";
    t2.child_frame_id = "cluster2_frame";
    t2.transform.translation.x = x2;
    t2.transform.translation.y = y2;
    t2.transform.translation.z = 0.0;
    t2.transform.rotation.x = q1.x();
    t2.transform.rotation.y = q1.y();
    t2.transform.rotation.z = q1.z();
    t2.transform.rotation.w = q1.w();
    tf_broadcaster_->sendTransform(t2);
    geometry_msgs::msg::TransformStamped t3;
    t3.header.stamp = this->now();
    t3.header.frame_id = "robot_front_laser_base_link";
    t3.child_frame_id = "cart_frame_from_laser";
    t3.transform.translation.x = midpoint_.first;
    t3.transform.translation.y = midpoint_.second;
    t3.transform.translation.z = 0.0;
    t3.transform.rotation.x = robot_odom_.pose.pose.orientation.x;
    t3.transform.rotation.y = robot_odom_.pose.pose.orientation.y;
    t3.transform.rotation.z = robot_odom_.pose.pose.orientation.z;
    t3.transform.rotation.w = robot_odom_.pose.pose.orientation.w;
    tf_broadcaster_->sendTransform(t3);
  }

  void move_to_cart() {
    if (!is_moving_to_cart_ && !is_moving_to_goal_) {
      return;
    }

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

        RCLCPP_INFO(this->get_logger(), "cart distance: %.2f ", cart_dist);

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
      // Move to the new goal point 30 cm ahead
      double delta_x_final = goal_pose2d_.x - robot_odom_.pose.pose.position.x;
      double delta_y_final = goal_pose2d_.y - robot_odom_.pose.pose.position.y;
      double distance_final = std::sqrt(delta_x_final * delta_x_final +
                                        delta_y_final * delta_y_final);

      RCLCPP_INFO(this->get_logger(), "final distance: %.2f ", distance_final);

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

        is_moving_to_goal_ = false;

        // Send the response
        if (pending_response_) {
          pending_response_->complete = true;
          RCLCPP_INFO(this->get_logger(),
                      "Final approach complete. Sending response.");
          pending_response_.reset();
        }
      }
    }
  }

  rclcpp::Service<attach_shelf_srv::srv::GoToLoading>::SharedPtr
      approach_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr pub_vel_timer_;
  nav_msgs::msg::Odometry robot_odom_;
  geometry_msgs::msg::Twist move_cmd_;
  geometry_msgs::msg::Pose2D goal_pose2d_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<attach_shelf_srv::srv::GoToLoading::Response>
      pending_response_;
  std::vector<float> laser_ranges_, laser_intensities_;
  std::pair<float, float> midpoint_;
  float angle_increment_, angle_min_;
  bool is_moving_to_cart_, is_moving_to_goal_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApproachShelfServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  return 0;
}
