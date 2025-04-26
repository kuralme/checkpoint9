#include "my_components/approach_server_component.hpp"
#include <memory>

int main(int argc, char *argv[]) {
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::NodeOptions options;

  auto approach_srv = std::make_shared<my_components::AttachServer>(options);
  executor.add_node(approach_srv);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}