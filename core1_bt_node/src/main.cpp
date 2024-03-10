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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <core1_bt_libs/core1_bt_libs.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("core1_bt_node");
  std::string xml_filepath = ament_index_cpp::get_package_share_directory("core1_bt_node") +
    "/trees/test.xml";
  RCLCPP_INFO(nh->get_logger(), "Loading XML file from: %s", xml_filepath.c_str());

  BT::BehaviorTreeFactory factory;
  BT::RosNodeParams params;

  params.nh = nh;
  params.default_port_value = "result";
  factory.registerNodeType<core1_bt_libs::SubString>("ReceiveString", params);

  params.default_port_value = "btcpp_int";
  factory.registerNodeType<core1_bt_libs::SubInt>("ReceiveInt", params);

  params.default_port_value = "restart_trigger";
  factory.registerNodeType<core1_bt_libs::SubEmpty>("RestartTrigger", params);

  params.default_port_value = "sleep_service";
  factory.registerNodeType<core1_bt_libs::ExampleActionClient>("ExampleActionClient", params);

  params.default_port_value = "tf_to_position";
  factory.registerNodeType<core1_bt_libs::GetAnglesActionClient>("GetAnglesActionClient", params);

  auto tree = factory.createTreeFromFile(xml_filepath);

  while (rclcpp::ok()) {
    tree.tickWhileRunning();
  }

  return 0;
}
