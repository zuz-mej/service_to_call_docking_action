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

#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/dock_robot.hpp>

#include "send_to_dock/send_to_dock_node.hpp"

using DockRobot = nav2_msgs::action::DockRobot;
using ClientGoalHandleDockRobot = rclcpp_action::ClientGoalHandle<DockRobot>;
using ServerGoalHandleDockRobot = rclcpp_action::ServerGoalHandle<DockRobot>;
using SetBoolSrv = std_srvs::srv::SetBool;

class SendToDockNodeWrapper : public send_to_dock::SendToDockNode {
public:
  SendToDockNodeWrapper() {};

  void HandleService(const SetBoolSrv::Request::SharedPtr request,
                     SetBoolSrv::Response::SharedPtr response) {
    return send_to_dock::SendToDockNode::HandleService(request, response);
  }

  void SetActiveGoal() {
    send_to_dock::SendToDockNode::active_goal_ =
        ClientGoalHandleDockRobot::SharedPtr(
            reinterpret_cast<ClientGoalHandleDockRobot *>(0x1), [](auto *) {});
  }
};

class TestSendToDockNode : public testing::Test {
protected:
  TestSendToDockNode();
  rclcpp_action::Server<DockRobot>::SharedPtr CreateDockServer();
  std::shared_ptr<SendToDockNodeWrapper> node_;
  SetBoolSrv::Request::SharedPtr request_;
  SetBoolSrv::Response::SharedPtr response_;
  rclcpp_action::Server<DockRobot>::SharedPtr action_server_;
};

TestSendToDockNode::TestSendToDockNode() {
  node_ = std::make_shared<SendToDockNodeWrapper>();
  request_ = std::make_shared<SetBoolSrv::Request>();
  response_ = std::make_shared<SetBoolSrv::Response>();
  action_server_ = nullptr;
}

rclcpp_action::Server<DockRobot>::SharedPtr
TestSendToDockNode::CreateDockServer() {
  return rclcpp_action::create_server<DockRobot>(
      node_, "dock_robot",
      // handle_goal
      [](const rclcpp_action::GoalUUID &, DockRobot::Goal::ConstSharedPtr) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      // handle_cancel
      [](const std::shared_ptr<ServerGoalHandleDockRobot>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      // handle_accepted
      [](const std::shared_ptr<ServerGoalHandleDockRobot> goal_handle) {
        auto result = std::make_shared<DockRobot::Result>();
        goal_handle->succeed(result);
      });
}

TEST_F(TestSendToDockNode, DockServerIsNotCreated) {
  node_->HandleService(request_, response_);
  ASSERT_FALSE(response_->success);
  EXPECT_EQ(response_->message, "Docking action server is not available");
}

TEST_F(TestSendToDockNode, DockServerCreatedAndTrueRequest) {
  action_server_ = CreateDockServer();
  request_->data = true;
  node_->HandleService(request_, response_);
  ASSERT_TRUE(response_->success);
  EXPECT_EQ(response_->message, "New docking goal request sent.");
}

TEST_F(TestSendToDockNode, DockServerCreatedAndFalseRequest) {
  action_server_ = CreateDockServer();
  request_->data = false;
  node_->HandleService(request_, response_);
  ASSERT_TRUE(response_->success);
  EXPECT_EQ(response_->message, "No active docking goal to cancel.");
}

TEST_F(TestSendToDockNode, DockRobotTrueRequestAndExistingGoal) {
  action_server_ = CreateDockServer();
  node_->SetActiveGoal();
  request_->data = true;
  node_->HandleService(request_, response_);
  ASSERT_TRUE(response_->success);
  EXPECT_EQ(response_->message, "Docking goal already exists.");
}

TEST_F(TestSendToDockNode, DockRobotFalseRequestAndExistingGoal) {
  action_server_ = CreateDockServer();
  node_->SetActiveGoal();
  request_->data = false;
  node_->HandleService(request_, response_);
  ASSERT_TRUE(response_->success);
  EXPECT_EQ(response_->message, "Docking canceled.");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
