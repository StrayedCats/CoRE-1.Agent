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
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace core1_bt_libs
{

class SubPoseArray : public BT::RosTopicSubNode<geometry_msgs::msg::PoseArray>
{
public:
  SubPoseArray(const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);
  static BT::PortsList providedPorts();
  BT::NodeStatus onTick(const std::shared_ptr<geometry_msgs::msg::PoseArray> & last_msg) override;
};
}  // namespace core1_bt_libs
