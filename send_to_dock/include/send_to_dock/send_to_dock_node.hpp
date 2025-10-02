// Copyright 2024 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SERVICE_TO_CALL_DOCKING_ACTION_SEND_TO_DOCK_NODE_HPP_
#define SERVICE_TO_CALL_DOCKING_ACTION_SEND_TO_DOCK_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <nav2_msgs/action/dock_robot.hpp>

class SendToDockNode : public rclcpp::Node {
public:
  using DockRobot = nav2_msgs::action::DockRobot;
  // using GoalHandleDockRobot = rclcpp_action::ClientGoalHandle<DockRobot>;

  SendToDockNode() : Node("send_to_dock_node") {

    // create action client
    dock_action_client_ =
        rclcpp_action::create_client<DockRobot>(this, "/panther/dock_robot");

    // wait for action server
    while (
        !dock_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_INFO(get_logger(), "Waiting for action server DockRobot");
    }

    handle_service();

    this->declare_parameter<std::string>("example_param", "default_value");
    std::string example_param =
        this->get_parameter("example_param").as_string();
    RCLCPP_INFO(this->get_logger(),
                "Declared parameter 'example_param'. Value: %s",
                example_param.c_str());

    RCLCPP_INFO(this->get_logger(), "Hello world from the C++ node %s",
                "send_to_dock_node");
  }

private:
  rclcpp_action::Client<DockRobot>::SharedPtr dock_action_client_;
  void handle_service();
};

#endif // SERVICE_TO_CALL_DOCKING_ACTION_SEND_TO_DOCK_NODE_HPP_
