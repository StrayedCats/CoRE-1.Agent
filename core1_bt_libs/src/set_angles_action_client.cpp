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

#include "core1_bt_libs/set_angles_action_client.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace core1_bt_libs
{

SetAnglesActionClient::SetAnglesActionClient(
  const std::string & name, const NodeConfig & conf,
  const RosNodeParams & params)
: RosActionNode<auto_driver_msgs::action::MoveToTargetDeg>(name, conf, params)
{}

BT::PortsList SetAnglesActionClient::providedPorts()
{
  return providedBasicPorts({
      InputPort<unsigned>("yaw"),
      InputPort<unsigned>("pitch"),
      InputPort<unsigned>("yaw_gap"),
      InputPort<unsigned>("pitch_gap"),
    });
}

bool SetAnglesActionClient::setGoal(RosActionNode::Goal & goal)
{
  RCLCPP_INFO(node_->get_logger(), "SetAnglesActionClient: setGoal");
  auto yaw = getInput<unsigned>("yaw");
  auto pitch = getInput<unsigned>("pitch");
  auto yaw_gap = getInput<unsigned>("yaw_gap");
  auto pitch_gap = getInput<unsigned>("pitch_gap");

  // goal.yaw_deg = yaw + yaw_gap;
  // goal.pitch_deg = pitch + pitch_gap;
  goal.yaw_deg = yaw.value() + yaw_gap.value();
  goal.pitch_deg = pitch.value() + pitch_gap.value();
  goal.msec = 1000;
  RCLCPP_INFO(node_->get_logger(), "SetAnglesActionClient: setGoal: yaw: %d, pitch: %d", goal.yaw_deg, goal.pitch_deg);
  return true;
}

NodeStatus SetAnglesActionClient::onResultReceived(const RosActionNode::WrappedResult & wr)
{
  RCLCPP_INFO(node_->get_logger(), "SetAnglesActionClient: onResultReceived");

  return wr.result->succeed ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus SetAnglesActionClient::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void SetAnglesActionClient::onHalt()
{
  RCLCPP_INFO(node_->get_logger(), "%s: onHalt", name().c_str() );
}

}  // namespace core1_bt_libs


CreateRosNodePlugin(core1_bt_libs::SetAnglesActionClient, "SetAnglesActionClient");