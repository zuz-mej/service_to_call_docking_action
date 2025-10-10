cmake_minimum_required(VERSION 3.11)
project(send_to_dock)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES ament_cmake rclcpp rclcpp_action nav2_msgs std_srvs)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

add_executable(send_to_dock_node src/main.cpp src/send_to_dock_node.cpp)

target_include_directories(
  send_to_dock_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(send_to_dock_node ${PACKAGE_DEPENDENCIES})

install(TARGETS send_to_dock_node DESTINATION lib/${PROJECT_NAME})

ament_package()
