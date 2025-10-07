// Copyright 2025 Husarion sp. z o.o.
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

#ifndef SEND_TO_DOCK_SEND_TO_DOCK_SEND_TO_DOCK_NODE_HPP_
#define SEND_TO_DOCK_SEND_TO_DOCK_SEND_TO_DOCK_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/dock_robot.hpp>
#include <std_srvs/srv/set_bool.hpp>

class SendToDockNode : public rclcpp::Node {
public:
  using DockRobot = nav2_msgs::action::DockRobot;
  using GoalHandleDockRobot = rclcpp_action::ClientGoalHandle<DockRobot>;

  SendToDockNode() : Node("send_to_dock_node") {

    // create action client
    dock_action_client_ =
        rclcpp_action::create_client<DockRobot>(this, "/panther/dock_robot");

    // wait for action server
    while (
        !dock_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_INFO(get_logger(), "Waiting for action server DockRobot");
    }

    // create service server SetBool
    service_ = this->create_service<std_srvs::srv::SetBool>(
        "send_robot_to_dock",
        std::bind(&SendToDockNode::handle_service, this, std::placeholders::_1,
                  std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
                "Service server 'send_robot_to_dock' ready");
  }

private:
  rclcpp_action::Client<DockRobot>::SharedPtr dock_action_client_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

  void
  handle_service(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                 std::shared_ptr<std_srvs::srv::SetBool::Response> response);
};

#endif // SEND_TO_DOCK_SEND_TO_DOCK_SEND_TO_DOCK_NODE_HPP_
