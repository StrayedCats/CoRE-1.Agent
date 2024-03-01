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

#include "core1_bt_libs/example_action_client.hpp"

namespace core1_bt_libs
{

ExampleActionClient::ExampleActionClient(
  const std::string & name, const NodeConfig & conf,
  const RosNodeParams & params)
: RosActionNode<core1_agent_msgs::action::ExampleTimer>(name, conf, params)
{}

BT::PortsList ExampleActionClient::providedPorts()
{
  return providedBasicPorts({InputPort<unsigned>("msec")});
}

bool ExampleActionClient::setGoal(RosActionNode::Goal & goal)
{
  auto timeout = getInput<unsigned>("msec");
  goal.msec_timeout = timeout.value();
  return true;
}

NodeStatus ExampleActionClient::onResultReceived(const RosActionNode::WrappedResult & wr)
{
  RCLCPP_INFO(
    node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(),
    wr.result->done ? "true" : "false");

  return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus ExampleActionClient::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void ExampleActionClient::onHalt()
{
  RCLCPP_INFO(node_->get_logger(), "%s: onHalt", name().c_str() );
}

}  // namespace core1_bt_libs
