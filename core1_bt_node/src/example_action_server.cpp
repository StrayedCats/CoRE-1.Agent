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

#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <core1_agent_msgs/action/example_timer.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>

class ExampleTimerActionServer : public rclcpp::Node
{
public:
  using ExampleTimer = core1_agent_msgs::action::ExampleTimer;
  using GoalHandleExampleTimer = rclcpp_action::ServerGoalHandle<ExampleTimer>;

  explicit ExampleTimerActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("sleep_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<ExampleTimer>(
      this,
      "sleep_service",
      std::bind(&ExampleTimerActionServer::handle_goal, this, _1, _2),
      std::bind(&ExampleTimerActionServer::handle_cancel, this, _1),
      std::bind(&ExampleTimerActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<ExampleTimer>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExampleTimer::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with sleep time %d", goal->msec_timeout);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExampleTimer> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleExampleTimer> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&ExampleTimerActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleExampleTimer> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(5);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ExampleTimer::Feedback>();
    auto result = std::make_shared<ExampleTimer::Result>();

    rclcpp::Time deadline = get_clock()->now() +
      rclcpp::Duration::from_seconds(double(goal->msec_timeout) / 1000);
    int cycle = 0;

    while (get_clock()->now() < deadline) {
      if (goal_handle->is_canceling()) {
        result->done = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      feedback->cycle = cycle++;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->done = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class ExampleTimerActionServer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleTimerActionServer>();

  rclcpp::spin(node);

  return 0;
}
