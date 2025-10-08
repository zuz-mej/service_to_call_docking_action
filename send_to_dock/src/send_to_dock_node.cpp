// Copyright 2025 Husarion Sp. z o.o.
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

#include "send_to_dock/send_to_dock_node.hpp"

namespace send_to_dock {

SendToDockNode::SendToDockNode() : rclcpp::Node("send_to_dock_node") {
  dock_action_client_ =
      rclcpp_action::create_client<DockRobot>(this, "/panther/dock_robot");

  while (
      !dock_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_INFO(get_logger(), "Waiting for action server DockRobot");
  }

  service_ = this->create_service<SetBoolSrv>(
      "send_robot_to_dock",
      std::bind(&SendToDockNode::handle_service, this, std::placeholders::_1,
                std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Service server 'send_robot_to_dock' ready");
}

void SendToDockNode::feedback_callback(
    GoalHandleDockRobot::SharedPtr,
    const std::shared_ptr<const DockRobot::Feedback> feedback) {
  RCLCPP_INFO(this->get_logger(), "Feedback received: Robot state = %d",
              feedback->state);
}

void SendToDockNode::result_callback(
    const GoalHandleDockRobot::WrappedResult &result) {
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(this->get_logger(), "Docking succeeded! Robot is charging.");
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_WARN(this->get_logger(), "Docking aborted!");
    break;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_WARN(this->get_logger(), "Docking canceled by user.");
    break;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
    break;
  }
  active_goal_.reset();
}

void SendToDockNode::handle_service(
    const std::shared_ptr<SetBoolSrv::Request> request,
    std::shared_ptr<SetBoolSrv::Response> response) {

  if (request->data) {
    auto goal_msg = DockRobot::Goal();
    goal_msg.dock_type = "charging_dock";
    goal_msg.navigate_to_staging_pose = true;
    goal_msg.dock_id = "main";

    auto goal_options = rclcpp_action::Client<DockRobot>::SendGoalOptions();
    goal_options.feedback_callback =
        std::bind(&SendToDockNode::feedback_callback, this,
                  std::placeholders::_1, std::placeholders::_2);

    goal_options.result_callback = std::bind(&SendToDockNode::result_callback,
                                             this, std::placeholders::_1);

    auto future_goal_handle =
        dock_action_client_->async_send_goal(goal_msg, goal_options);

    // active_goal_ = future_goal_handle.get();   // blocking

    response->success = true;
    response->message = "Docking request sent.";
  } else {
    if (1) { // active_goal_     // segmentation fault
      dock_action_client_->async_cancel_goal(active_goal_);
      response->success = true;
      response->message = "Docking canceled.";
      RCLCPP_INFO(this->get_logger(), "Sent cancel request to action server.");
    } else {
      response->success = false;
      response->message = "No active docking goal to cancel.";
      RCLCPP_WARN(this->get_logger(), "No active goal to cancel.");
    }
  }
}
} // namespace send_to_dock
