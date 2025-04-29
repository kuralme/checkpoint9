#ifndef COMPOSITION__APPROACH_SERVER_COMPONENT_HPP_
#define COMPOSITION__APPROACH_SERVER_COMPONENT_HPP_
#include "attach_shelf_srv/srv/go_to_loading.hpp"
#include "my_components/visibility_control.h"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace my_components {

class AttachServer : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachServer(const rclcpp::NodeOptions &options);

protected:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void listen_cart_tf(void);
  void shutdown_cb(void);

  void check_and_respond_to_service();

  void approach_shelf_callback(
      const std::shared_ptr<attach_shelf_srv::srv::GoToLoading::Request>
          request,
      std::shared_ptr<attach_shelf_srv::srv::GoToLoading::Response> response);
  std::vector<int>
  find_intensity_cluster_centers(const std::vector<float> &intensities,
                                 size_t start_idx, size_t end_idx,
                                 float threshold);
  void broadcast_cart_frame(std::pair<float, float> cart_frame);
  void move_to_cart(void);

private:
  rclcpp::Service<attach_shelf_srv::srv::GoToLoading>::SharedPtr
      approach_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr attach_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
  rclcpp::TimerBase::SharedPtr move_timer_;
  geometry_msgs::msg::Pose2D goal_pose2d_;
  nav_msgs::msg::Odometry robot_odom_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  std::vector<float> laser_ranges_, laser_intensities_;
  float angle_increment_, angle_min_;
  bool is_moving_to_cart_, is_moving_to_goal_, is_service_finished_;
};

} // namespace my_components

#endif // COMPOSITION__APPROACH_SERVER_COMPONENT_HPP_