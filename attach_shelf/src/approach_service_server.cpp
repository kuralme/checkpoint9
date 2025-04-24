#include "attach_shelf_srv/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

class ApproachShelfServer : public rclcpp::Node {
public:
  ApproachShelfServer() : Node("approach_shelf_server") {

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&ApproachShelfServer::scan_callback, this,
                  std::placeholders::_1));
    // pub_timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(100),
    //     std::bind(&ApproachShelfServer::final_approach_execute, this));
    approach_server_ = this->create_service<attach_shelf_srv::srv::GoToLoading>(
        "/approach_shelf",
        std::bind(&ApproachShelfServer::approach_shelf_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

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

  void approach_shelf_callback(
      const std::shared_ptr<attach_shelf_srv::srv::GoToLoading::Request>
          request,
      std::shared_ptr<attach_shelf_srv::srv::GoToLoading::Response> response) {

    RCLCPP_INFO(this->get_logger(),
                "Received Service Request: Robot will %s final approach.",
                request->attach_to_shelf ? "do" : "not do");

    float angle_range_deg = 45.0;
    float angle_range_rad = angle_range_deg * (M_PI / 180.0);
    size_t front_idx =
        static_cast<size_t>((0.0 - angle_min_) / angle_increment_);
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
    std::cout << "Index 1: " << idx1 << ", Index 2: " << idx2 << std::endl;

    // Convert index to angle
    float angle1 = angle_min_ + idx1 * angle_increment_;
    float angle2 = angle_min_ + idx2 * angle_increment_;

    // Convert to x, y
    float x1 = laser_ranges_[idx1] * std::cos(angle1);
    float y1 = laser_ranges_[idx1] * std::sin(angle1);
    float x2 = laser_ranges_[idx2] * std::cos(angle2);
    float y2 = laser_ranges_[idx2] * std::sin(angle2);

    // Get midpoint
    std::pair<float, float> midpoint = {(x1 + x2) / 2.0f, (y1 + y2) / 2.0f};

    broadcast_cart_frame(midpoint.first, midpoint.second);

    if (request->attach_to_shelf) {
      move_to_cart_loading(midpoint.first, midpoint.second);
      RCLCPP_INFO(this->get_logger(), "Final approach complete.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Transform published, no approach done.");
    }
    response->complete = true;
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

  void broadcast_cart_frame(float x, float y) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "robot_base_link";
    t.child_frame_id = "cart_frame";
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
    RCLCPP_INFO(this->get_logger(), "Broadcasted cart_frame at (%.2f, %.2f)", x,
                y);
  }

  void move_to_cart_loading(float x, float y) {
    // Move to cart frame location and move forward 30cm
    float distance = std::hypot(x, y) + 0.3;
    float moved = 0.0;

    rclcpp::Rate rate(10);
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.2;

    while (moved < distance && rclcpp::ok()) {
      cmd_vel_pub_->publish(cmd);
      rate.sleep();
      moved += cmd.linear.x * 0.1; // simulate 10Hz steps
    }
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
  }

  rclcpp::Service<attach_shelf_srv::srv::GoToLoading>::SharedPtr
      approach_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  geometry_msgs::msg::Twist move_cmd_;
  std::vector<float> laser_ranges_, laser_intensities_;
  float angle_increment_, angle_min_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachShelfServer>());
  rclcpp::shutdown();
  return 0;
}
