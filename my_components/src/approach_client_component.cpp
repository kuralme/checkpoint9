#include "my_components/approach_client_component.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace my_components {

AttachClient::AttachClient(const rclcpp::NodeOptions &options)
    : Node("approach_shelf_client", options), service_requested_(false) {

  approach_client_ = this->create_client<attach_shelf_srv::srv::GoToLoading>(
      "/approach_shelf");
  status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/approach_status", 10,
      std::bind(&AttachClient::status_callback, this, std::placeholders::_1));

  call_approach_service();
}

void AttachClient::call_approach_service(void) {
  while (!approach_client_->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN(this->get_logger(), "Waiting for approach_shelf service...");
  }

  auto request =
      std::make_shared<attach_shelf_srv::srv::GoToLoading::Request>();
  request->attach_to_shelf = true;
  service_requested_ = true;

  auto result_future = approach_client_->async_send_request(
      request,
      std::bind(&AttachClient::response_callback, this, std::placeholders::_1));

  // Wait for service response
  //   auto status = result_future.wait_for(20s);
  //   if (status != std::future_status::ready) {
  //     RCLCPP_WARN(this->get_logger(), "Response not ready yet.");
  //   }
}

void AttachClient::response_callback(
    rclcpp::Client<attach_shelf_srv::srv::GoToLoading>::SharedFuture future) {
  auto response = future.get();

  if (response->complete) {
    RCLCPP_INFO(this->get_logger(),
                "Service Response - Request accepted, awaiting completion...");
  } else {
    RCLCPP_WARN(this->get_logger(), "Service Response - Failed.");
  }
}

void AttachClient::status_callback(const std_msgs::msg::String::SharedPtr msg) {
  if (service_requested_) {
    if (msg->data == "done") {
      RCLCPP_INFO(this->get_logger(), "Final approach completed.");
      rclcpp::shutdown();
    } else if (msg->data == "in_progress") {
      //   RCLCPP_INFO(this->get_logger(), "Approach task is still in
      //   progress...");
    } else {
      RCLCPP_WARN(this->get_logger(), "Approach task failed.");
      rclcpp::shutdown();
    }
  }
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)