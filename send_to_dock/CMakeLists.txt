cmake_minimum_required(VERSION 3.11)
project(send_to_dock)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

find_package(rclcpp REQUIRED)

# Add node send_to_dock_node
add_executable(send_to_dock_node src/main.cpp src/send_to_dock_node.cpp)

target_include_directories(
  send_to_dock_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

# ament_target_dependencies(send_to_dock_node "rclcpp")
ament_target_dependencies(send_to_dock_node rclcpp rclcpp_action nav2_msgs)

install(TARGETS send_to_dock_node DESTINATION lib/${PROJECT_NAME})

ament_package()
