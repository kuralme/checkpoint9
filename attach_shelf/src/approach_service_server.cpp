#include "attach_shelf_srv/srv/go_to_loading.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>

class ApproachShelfServer : public rclcpp::Node {
public:
  ApproachShelfServer() : Node("approach_shelf_server") {
    approach_server_ = this->create_service<attach_shelf_srv::srv::GoToLoading>(
        "/approach_shelf",
        std::bind(&ApproachShelfServer::approach_shelf_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Approach shelf service ready.");
  }

private:
  void approach_shelf_callback(
      const std::shared_ptr<attach_shelf_srv::srv::GoToLoading::Request>
          request,
      std::shared_ptr<attach_shelf_srv::srv::GoToLoading::Response> response) {

    if (request->attach_to_shelf) {
      RCLCPP_INFO(this->get_logger(),
                  "Received Service Request: Robot will do final approach.");
    } else {
      RCLCPP_INFO(
          this->get_logger(),
          "Received Service Request: Robot will not do final approach.");
    }

    // need to do final approach

    response->complete = true;
    RCLCPP_INFO(this->get_logger(), "Service Completed.");
  }

  rclcpp::Service<attach_shelf_srv::srv::GoToLoading>::SharedPtr
      approach_server_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachShelfServer>());
  rclcpp::shutdown();
  return 0;
}
