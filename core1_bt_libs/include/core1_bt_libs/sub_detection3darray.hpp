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

#pragma once

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

namespace core1_bt_libs
{

class SubDetection3dArray : public BT::RosTopicSubNode<vision_msgs::msg::Detection3DArray>
{
public:
  SubDetection3dArray(const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);
  static BT::PortsList providedPorts();
  BT::NodeStatus onTick(const std::shared_ptr<vision_msgs::msg::Detection3DArray> & last_msg) override;
};
}  // namespace core1_bt_libs
