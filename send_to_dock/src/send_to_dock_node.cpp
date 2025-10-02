// Copyright (c) 2024 Husarion Sp. z o.o.
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

void SendToDockNode::handle_service() {
  std::cout << "Jesteśmy w metodzie handle service\n";

  // Prepare goal (parameters to action)
  auto goal_msg = DockRobot::Goal();
  goal_msg.dock_type = "charging_dock";
  goal_msg.navigate_to_staging_pose = true;
  goal_msg.dock_id = "main";

  // Send goal
  auto goal_options = rclcpp_action::Client<DockRobot>::SendGoalOptions();
  // goal_options.result_callback = [this, response](const
  // GoalHandleDockRobot::WrappedResult & result) {
  //   if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
  //     response->success = true;
  //     response->message = "Dokowanie zakończone sukcesem.";
  //     RCLCPP_INFO(this->get_logger(), "Akcja zakończona sukcesem.");
  //   } else {
  //     response->success = false;
  //     response->message = "Błąd dokowania.";
  //     RCLCPP_WARN(this->get_logger(), "Akcja zakończona błędem.");
  //   }
  // };

  dock_action_client_->async_send_goal(goal_msg, goal_options);
}
