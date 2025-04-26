cmake_minimum_required(VERSION 3.8)
project(my_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(attach_shelf_srv REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

# Pre approach composition node
add_library(pre_approach_component SHARED src/pre_approach_component.cpp)
target_compile_definitions(pre_approach_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(pre_approach_component rclcpp rclcpp_components geometry_msgs sensor_msgs nav_msgs tf2)
rclcpp_components_register_nodes(pre_approach_component "my_components::PreApproach")
set(node_plugins "${node_plugins}my_components::PreApproach;$<TARGET_FILE:pre_approach_component>\n")

# Server composition nodes: Client (Runtime), Server (manual)
add_library(approach_client_component SHARED src/approach_client_component.cpp)
target_compile_definitions(approach_client_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(approach_client_component rclcpp rclcpp_components geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros attach_shelf_srv)
rclcpp_components_register_nodes(approach_client_component "my_components::AttachClient")
set(node_plugins "${node_plugins}my_components::AttachClient;$<TARGET_FILE:approach_client_component>\n")
add_library(approach_server_component SHARED src/approach_server_component.cpp)
target_compile_definitions(approach_server_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(approach_server_component rclcpp rclcpp_components geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros attach_shelf_srv)
rclcpp_components_register_nodes(approach_server_component "my_components::AttachServer")
set(node_plugins "${node_plugins}my_components::AttachServer;$<TARGET_FILE:approach_server_component>\n")

# Manual composition
add_executable(server_manual_composition src/server_manual_composition.cpp)
target_link_libraries(server_manual_composition approach_server_component)
ament_target_dependencies(server_manual_composition rclcpp)
install(TARGETS
  server_manual_composition
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  pre_approach_component
  approach_client_component
  approach_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
