#include "attach_shelf_srv/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>

using namespace std::chrono_literals;

class PreApproach : public rclcpp::Node {
public:
  PreApproach(const rclcpp::NodeOptions &options)
      : Node("pre_approach_node", options), rotating_(false),
        total_rotation_(0.0), initial_yaw_(0.0), current_yaw_(0.0),
        _final_approach(false) {

    int degrees;
    this->get_parameter_or<float>("obstacle", _obstacle_dist, 0.3);
    this->get_parameter_or<int>("degrees", degrees, -90);
    _goal_rotation = degrees * M_PI / 180.0; // [rad]
    this->get_parameter_or<bool>("final_approach", _final_approach, false);

    RCLCPP_INFO(this->get_logger(),
                "Pre-approach initialized with parameters:");
    RCLCPP_INFO(this->get_logger(), "- obstacle       : %.1f", _obstacle_dist);
    RCLCPP_INFO(this->get_logger(), "- degrees        : %i", degrees);
    RCLCPP_INFO(this->get_logger(), "- final_approach : %s",
                _final_approach ? "true" : "false");

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
    approach_client_ = this->create_client<attach_shelf_srv::srv::GoToLoading>(
        "/approach_shelf");

    move_cmd_.linear.x = 0.0;
    move_cmd_.angular.z = 0.0;
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (msg->ranges.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty laser scan data.");
      return;
    }
    laser_ranges_ = msg->ranges;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
  }

  void preapproach_execute() {
    if (laser_ranges_.empty()) {
      RCLCPP_WARN(this->get_logger(), "Laser data not received yet.");
      return;
    }

    float laser_forward = get_min_distance(laser_ranges_, 500, 580);

    if (!rotating_ && laser_forward > _obstacle_dist) {
      // Move forward
      move_cmd_.linear.x = 0.3;
      move_cmd_.angular.z = 0.0;

    } else if (!rotating_ && laser_forward <= _obstacle_dist) {
      // Start rotating
      RCLCPP_INFO(this->get_logger(), "Reached wall, starting rotation.");
      move_cmd_.linear.x = 0.0;
      move_cmd_.angular.z = (_goal_rotation > 0 ? 0.4 : -0.4);
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
                  delta_yaw * 180.0 / M_PI, _goal_rotation * 180.0 / M_PI);

      if (std::abs(delta_yaw) >= std::abs(_goal_rotation)) {
        move_cmd_.linear.x = 0.0;
        move_cmd_.angular.z = 0.0;
        rotating_ = false;
        pub_timer_->cancel();

        call_approach_service();
      }
    }
    cmd_vel_pub_->publish(move_cmd_);
  }

  void call_approach_service() {
    while (!approach_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Waiting for approach_shelf service...");
    }
    auto request =
        std::make_shared<attach_shelf_srv::srv::GoToLoading::Request>();
    request->attach_to_shelf = _final_approach;

    auto result_future = approach_client_->async_send_request(
        request, std::bind(&PreApproach::response_callback, this,
                           std::placeholders::_1));

    // Now check for the response after a timeout
    auto status = result_future.wait_for(20s);
    if (status != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "Response not ready yet.");
    }
  }
  void response_callback(
      rclcpp::Client<attach_shelf_srv::srv::GoToLoading>::SharedFuture future) {
    auto response = future.get();

    RCLCPP_INFO(this->get_logger(), "Service Response - service %s.",
                response->complete ? "successful" : "failed");
    rclcpp::shutdown();
  }

  float get_min_distance(const std::vector<float> &ranges, size_t start_idx,
                         size_t end_idx) {
    auto it =
        std::min_element(ranges.begin() + start_idx, ranges.begin() + end_idx,
                         [](float a, float b) { return a > 0.0f && a < b; });

    return (it != ranges.begin() + end_idx)
               ? *it
               : std::numeric_limits<float>::infinity();
  }

  rclcpp::Client<attach_shelf_srv::srv::GoToLoading>::SharedPtr
      approach_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  geometry_msgs::msg::Twist move_cmd_;
  std::vector<float> laser_ranges_;
  float _goal_rotation;
  float _obstacle_dist;
  bool rotating_;
  float total_rotation_;
  float initial_yaw_;
  float current_yaw_;
  bool _final_approach;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<PreApproach>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}