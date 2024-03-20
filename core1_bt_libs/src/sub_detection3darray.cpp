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

#include "core1_bt_libs/sub_detection3darray.hpp"

namespace core1_bt_libs
{

SubDetection3dArray::SubDetection3dArray(
  const std::string & name, const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::RosTopicSubNode<vision_msgs::msg::Detection3DArray>(name, conf, params)
{}

BT::PortsList SubDetection3dArray::providedPorts()
{
  return {
      BT::OutputPort<vision_msgs::msg::Detection3DArray>("detection3d_array"),
    };
}

BT::NodeStatus SubDetection3dArray::onTick(const std::shared_ptr<vision_msgs::msg::Detection3DArray> & last_msg)
{
  if (last_msg) {
    setOutput("detection3d_array", *last_msg);
  } else {
    RCLCPP_INFO(logger(), "[%s] no message", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}
}  // namespace core1_bt_libs
