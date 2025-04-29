#ifndef COMPOSITION__APPROACH_CLIENT_COMPONENT_HPP_
#define COMPOSITION__APPROACH_CLIENT_COMPONENT_HPP_
#include "attach_shelf_srv/srv/go_to_loading.hpp"
#include "my_components/visibility_control.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace my_components {

class AttachClient : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachClient(const rclcpp::NodeOptions &options);

protected:
  void call_approach_service(void);
  void response_callback(
      rclcpp::Client<attach_shelf_srv::srv::GoToLoading>::SharedFuture future);
  void status_callback(const std_msgs::msg::String::SharedPtr msg);

private:
  rclcpp::Client<attach_shelf_srv::srv::GoToLoading>::SharedPtr
      approach_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  bool service_requested_;
};

} // namespace my_components

#endif // COMPOSITION__APPROACH_CLIENT_COMPONENT_HPP_