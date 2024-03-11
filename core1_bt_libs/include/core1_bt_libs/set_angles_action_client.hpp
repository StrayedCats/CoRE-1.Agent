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

// Ref: https://github.com/BehaviorTree/BehaviorTree.ROS2 (Apache-2.0 license)

#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_ros2/bt_action_node.hpp>
#include "auto_driver_msgs/action/move_to_target_deg.hpp"

namespace core1_bt_libs
{

using namespace BT;

class SetAnglesActionClient : public BT::RosActionNode<auto_driver_msgs::action::MoveToTargetDeg>
{
public:
  SetAnglesActionClient(
    const std::string & name, const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);
  static BT::PortsList providedPorts();

  bool setGoal(Goal & goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override;

  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};

}  // namespace core1_bt_libs
