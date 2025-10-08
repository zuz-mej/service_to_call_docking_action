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

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/dock_robot.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace send_to_dock {
using SetBoolSrv = std_srvs::srv::SetBool;

class SendToDockNode : public rclcpp::Node {
public:
  SendToDockNode();
  using DockRobot = nav2_msgs::action::DockRobot;
  using GoalHandleDockRobot = rclcpp_action::ClientGoalHandle<DockRobot>;

private:
  rclcpp_action::Client<DockRobot>::SharedPtr dock_action_client_;
  rclcpp::Service<SetBoolSrv>::SharedPtr service_;
  GoalHandleDockRobot::SharedPtr active_goal_;

  void
  feedback_callback(GoalHandleDockRobot::SharedPtr,
                    const std::shared_ptr<const DockRobot::Feedback> feedback);

  void result_callback(const GoalHandleDockRobot::WrappedResult &result);

  void handle_service(const std::shared_ptr<SetBoolSrv::Request> request,
                      std::shared_ptr<SetBoolSrv::Response> response);
};
} // namespace send_to_dock
#endif // SEND_TO_DOCK_SEND_TO_DOCK_SEND_TO_DOCK_NODE_HPP_
