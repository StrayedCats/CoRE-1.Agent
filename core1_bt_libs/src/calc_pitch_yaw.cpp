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

#include "core1_bt_libs/calc_pitch_yaw.hpp"

namespace core1_bt_libs
{
CalcPitchYaw::CalcPitchYaw(
  const std::string& name, const BT::NodeConfig& config,
  const BT::RosNodeParams & params)
: BT::RosTopicSubNode<std_msgs::msg::Empty>(name, config, params)
{}

BT::PortsList CalcPitchYaw::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::Pose>("pose"),
    BT::InputPort<float>("current_pitch"),
    BT::InputPort<float>("current_yaw"),
    BT::OutputPort<int32_t>("pitch"),
    BT::OutputPort<int32_t>("yaw")
  };
}

BT::NodeStatus CalcPitchYaw::onTick(const std::shared_ptr<std_msgs::msg::Empty> & last_msg)
{

  auto pose = getInput<geometry_msgs::msg::Pose>("pose").value();
  // use only x, y, z
  auto sqrt_x_y = std::sqrt(pose.position.x * pose.position.x + pose.position.y * pose.position.y);
  auto pitch = std::atan2(pose.position.z, sqrt_x_y) * 180 / M_PI;
  auto yaw = std::atan2(pose.position.y, pose.position.x) * 180 / M_PI;;

  setOutput("pitch", int32_t(pitch));
  setOutput("yaw", -int32_t(yaw));

  RCLCPP_INFO(node_->get_logger(), "CalcPitchYaw: pitch: %d, yaw: %d", int32_t(pitch), int32_t(yaw));


  return BT::NodeStatus::SUCCESS;
}

}  // namespace core1_bt_libs
