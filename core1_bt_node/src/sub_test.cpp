#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace BT;

class ReceiveString: public RosTopicSubNode<std_msgs::msg::String>
{
public:
  ReceiveString(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
    : RosTopicSubNode<std_msgs::msg::String>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  NodeStatus onTick(const std::shared_ptr<std_msgs::msg::String>& last_msg) override
  {
    if(last_msg)
    {
      RCLCPP_INFO(logger(), "[%s] new message: %s", name().c_str(), last_msg->data.c_str());
    }
    else
    {
      RCLCPP_INFO(logger(), "[%s] no message", name().c_str());
      return NodeStatus::FAILURE;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return NodeStatus::SUCCESS;
  }
};

class ReceiveInt: public RosTopicSubNode<std_msgs::msg::Int32>
{
public:
  ReceiveInt(const std::string& name,
             const NodeConfig& conf,
             const RosNodeParams& params)
    : RosTopicSubNode<std_msgs::msg::Int32>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Int32>& last_msg) override
  {
    if(last_msg)
    {
      RCLCPP_INFO(logger(), "[%s] new message: %d", name().c_str(), last_msg->data);
    }
    else
    {
      RCLCPP_INFO(logger(), "[%s] no message", name().c_str());
      return NodeStatus::FAILURE;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return NodeStatus::SUCCESS;
  }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("subscriber_test");

  std::string pkgpath = ament_index_cpp::get_package_share_directory("core1_bt");
  std::string xml_filepath = pkgpath + "/trees/test.xml";
  RCLCPP_INFO(nh->get_logger(), "Loading XML file from: %s", xml_filepath.c_str());

  BehaviorTreeFactory factory;

  RosNodeParams params;
  params.nh = nh;
  params.default_port_value = "btcpp_string";
  factory.registerNodeType<ReceiveString>("ReceiveString", params);

  params.default_port_value = "btcpp_int";
  factory.registerNodeType<ReceiveInt>("ReceiveInt", params);

  auto tree = factory.createTreeFromFile(xml_filepath);

  while(rclcpp::ok())
  {
    tree.tickWhileRunning();
  }

  return 0;
}
