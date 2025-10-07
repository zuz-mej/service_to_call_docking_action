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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "send_to_dock/send_to_dock_node.hpp"

class TestSendToDockNode : public testing::Test {
protected:
  TestSendToDockNode() { node_ = std::make_shared<SendToDockNode>(); }

  std::shared_ptr<SendToDockNode> node_;
};

TEST_F(TestSendToDockNode, NodeIsCreated) {
  ASSERT_NE(node_, nullptr); // node not equal to nullptr == success
  SUCCEED();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
