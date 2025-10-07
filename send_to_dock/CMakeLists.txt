cmake_minimum_required(VERSION 3.11)
project(send_to_dock)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test_send_to_dock_node
                  test/test_send_to_dock_node.cpp src/send_to_dock_node.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_send_to_dock_node
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_send_to_dock_node rclcpp
                            rclcpp_action nav2_msgs std_srvs)
endif()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

find_package(rclcpp REQUIRED)

add_executable(send_to_dock_node src/main.cpp src/send_to_dock_node.cpp)

target_include_directories(
  send_to_dock_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(send_to_dock_node rclcpp rclcpp_action nav2_msgs
                          std_srvs)

install(TARGETS send_to_dock_node DESTINATION lib/${PROJECT_NAME})

ament_package()
