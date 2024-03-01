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

#include "core1_bt_libs/sub_empty.hpp"

namespace core1_bt_libs
{

SubEmpty::SubEmpty(
  const std::string & name, const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::RosTopicSubNode<std_msgs::msg::Empty>(name, conf, params)
{}

BT::PortsList SubEmpty::providedPorts()
{
  return {};
}

BT::NodeStatus SubEmpty::onTick(const std::shared_ptr<std_msgs::msg::Empty> & last_msg)
{
  if (last_msg) {
    RCLCPP_INFO(logger(), "get empty message");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
}  // namespace core1_bt_libs
