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

#include "core1_bt_libs/get_angles_action_client.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace core1_bt_libs
{

GetAnglesActionClient::GetAnglesActionClient(
  const std::string & name, const NodeConfig & conf,
  const RosNodeParams & params)
: RosActionNode<auto_driver_msgs::action::GetAnglesFromTf>(name, conf, params)
{}

BT::PortsList GetAnglesActionClient::providedPorts()
{
  return {
      InputPort<std::string>("root_tf"),
      InputPort<std::string>("camera_tf"),
      InputPort<std::string>("target_tf"),
      OutputPort<int32_t>("yaw"),
      OutputPort<int32_t>("pitch"),
      OutputPort<int32_t>("yaw_gap"),
      OutputPort<int32_t>("pitch_gap"),
    };
}

bool GetAnglesActionClient::setGoal(RosActionNode::Goal & goal)
{
  auto root_tf = getInput<std::string>("root_tf");
  auto camera_tf = getInput<std::string>("camera_tf");
  auto target_tf = getInput<std::string>("target_tf");

  if (!root_tf || !camera_tf || !target_tf) {
    return false;
  }

  goal.root_tf_frame_id = root_tf.value();
  goal.camera_tf_frame_id = camera_tf.value();
  goal.target_tf_frame_id = target_tf.value();
  RCLCPP_INFO(node_->get_logger(), "root_tf: %s, camera_tf: %s, target_tf: %s",
    goal.root_tf_frame_id.c_str(), goal.camera_tf_frame_id.c_str(), goal.target_tf_frame_id.c_str());

  return true;
}

NodeStatus GetAnglesActionClient::onResultReceived(const RosActionNode::WrappedResult & wr)
{
  RCLCPP_INFO(node_->get_logger(), "yaw: %d, pitch: %d, yaw_gap: %d, pitch_gap: %d",
    wr.result->yaw_deg, wr.result->pitch_deg, wr.result->yaw_gap_deg, wr.result->pitch_gap_deg);

  setOutput("yaw", wr.result->yaw_deg);
  setOutput("pitch", wr.result->pitch_deg);
  setOutput("yaw_gap", wr.result->yaw_gap_deg);
  setOutput("pitch_gap", wr.result->pitch_gap_deg);

  return wr.result->succeed ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus GetAnglesActionClient::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void GetAnglesActionClient::onHalt()
{
  RCLCPP_INFO(node_->get_logger(), "%s: onHalt", name().c_str() );
}

}  // namespace core1_bt_libs
