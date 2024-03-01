// Copyright 2024 StrayedCats.
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

#include "core1_bt_libs/sub_bool.hpp"

namespace core1_bt_libs
{

SubBool::SubBool(
  const std::string & name, const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::RosTopicSubNode<std_msgs::msg::Bool>(name, conf, params)
{}

BT::PortsList SubBool::providedPorts()
{
  return {};
}

BT::NodeStatus SubBool::onTick(const std::shared_ptr<std_msgs::msg::Bool> & last_msg)
{
  if (last_msg) {
    if (last_msg->data) {
      RCLCPP_INFO(logger(), "[True] new message: %d", last_msg->data);
    } else {
      RCLCPP_INFO(logger(), "[False] new message: %d", last_msg->data);
    }
  } else {
    RCLCPP_INFO(logger(), "[%s] no message", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  return BT::NodeStatus::SUCCESS;
}
}  // namespace core1_bt_libs
