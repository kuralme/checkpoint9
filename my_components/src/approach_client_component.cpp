#include "my_components/approach_client_component.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace my_components {

AttachClient::AttachClient(const rclcpp::NodeOptions &options)
    : Node("approach_shelf_client", options) {

  approach_client_ = this->create_client<attach_shelf_srv::srv::GoToLoading>(
      "/approach_shelf");

  call_approach_service();
}

void AttachClient::call_approach_service(void) {
  while (!approach_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Waiting for approach_shelf service...");
  }
  auto request =
      std::make_shared<attach_shelf_srv::srv::GoToLoading::Request>();
  request->attach_to_shelf = true;

  auto result_future = approach_client_->async_send_request(
      request,
      std::bind(&AttachClient::response_callback, this, std::placeholders::_1));

  // Now check for the response after a timeout
  auto status = result_future.wait_for(20s);
  if (status != std::future_status::ready) {
    RCLCPP_WARN(this->get_logger(), "Response not ready yet.");
  }
}

void AttachClient::response_callback(
    rclcpp::Client<attach_shelf_srv::srv::GoToLoading>::SharedFuture future) {
  auto response = future.get();

  RCLCPP_INFO(this->get_logger(), "Service Response - service %s.",
              response->complete ? "successful" : "failed");
  rclcpp::shutdown();
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)