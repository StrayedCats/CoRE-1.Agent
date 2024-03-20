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

#include "core1_bt_libs/get_enemy_position_client.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace core1_bt_libs
{

GetEnemyPositionClient::GetEnemyPositionClient(
  const std::string & name, const NodeConfig & conf,
  const RosNodeParams & params)
: RosActionNode<core1_bt_action::action::TargetPose>(name, conf, params)
{}

BT::PortsList GetEnemyPositionClient::providedPorts()
{
  return {
      InputPort<vision_msgs::msg::Detection3DArray>("detection3d_array"),
      InputPort<std::string>("allies_label"),
      InputPort<std::string>("enemies_label"),
      OutputPort<geometry_msgs::msg::Pose>("pose"),
    };
}

bool GetEnemyPositionClient::setGoal(RosActionNode::Goal & goal)
{
  auto detection3d_array = getInput<vision_msgs::msg::Detection3DArray>("detection3d_array");
  auto allies_label = getInput<std::string>("allies_label");
  auto enemies_label = getInput<std::string>("enemies_label");

  if (!detection3d_array) {
    RCLCPP_ERROR(node_->get_logger(), "%s: detection3d_array is missing", name().c_str());
    return false;
  }

  auto allies_poses = geometry_msgs::msg::PoseArray();
  auto enemies_poses = geometry_msgs::msg::PoseArray();
  for (auto detection : detection3d_array.value().detections) {
    if (detection.results[0].hypothesis.class_id == allies_label.value()) {
      allies_poses.poses.push_back(detection.bbox.center);
    } else if (detection.results[0].hypothesis.class_id == enemies_label.value()) {
      enemies_poses.poses.push_back(detection.bbox.center);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Unknown label: %s", detection.results[0].hypothesis.class_id.c_str());
    }
  }

  RCLCPP_INFO(node_->get_logger(), "allies: %d, enemies: %d", allies_poses.poses.size(), enemies_poses.poses.size());

  goal.allies = allies_poses;
  goal.enemies = enemies_poses;

  return true;
}

NodeStatus GetEnemyPositionClient::onResultReceived(const RosActionNode::WrappedResult & wr)
{
  RCLCPP_INFO(node_->get_logger(), "%s: onResultReceived", name().c_str() );
  setOutput("pose", wr.result->target_pose);

  // if target pose is 0,0,0, return FAILURE
  if (wr.result->target_pose.position.x == 0 && wr.result->target_pose.position.y == 0 && wr.result->target_pose.position.z == 0) {
    return NodeStatus::FAILURE;
  }
  return NodeStatus::SUCCESS;
}

NodeStatus GetEnemyPositionClient::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void GetEnemyPositionClient::onHalt()
{
  RCLCPP_INFO(node_->get_logger(), "%s: onHalt", name().c_str() );
}

}  // namespace core1_bt_libs
