// ROS 2 action demo -- server side.
//
// Actions are the "long-running task" mechanism: unlike a service, the call is
// not a single blocking round-trip. A client sends a goal, the server accepts
// (or rejects) it, streams feedback while working, and finally returns a
// result. The goal can be cancelled at any time in between.
//
// This node computes the first `order` Fibonacci numbers, publishing the
// partial sequence as feedback twice per second.

#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_use_example_interfaces/action/example_action.hpp"

using ExampleAction = ros2_use_example_interfaces::action::ExampleAction;

class ActionServer : public rclcpp::Node
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExampleAction>;

  ActionServer()
  : Node("action_server")
  {
    using namespace std::placeholders;
    // The three callbacks correspond to the three phases of an action:
    //   handle_goal      : accept or reject a new goal
    //   handle_cancel    : accept or reject a cancel request
    //   handle_accepted  : start executing an accepted goal
    action_server_ = rclcpp_action::create_server<ExampleAction>(
      this, "example_action",
      std::bind(&ActionServer::handle_goal, this, _1, _2),
      std::bind(&ActionServer::handle_cancel, this, _1),
      std::bind(&ActionServer::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "Serving /example_action (Fibonacci)");
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExampleAction::Goal> goal)
  {
    (void)uuid;
    if (goal->order < 1 || goal->order > 30) {
      RCLCPP_WARN(
        this->get_logger(), "Rejecting goal: order=%d (must be in [1, 30])",
        goal->order);
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Accepting goal: order=%d", goal->order);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Cancel request accepted");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // execute() sleeps between feedback messages, so run it on its own thread:
    // the executor stays free to service cancel requests and other goals.
    std::thread{std::bind(&ActionServer::execute, this, std::placeholders::_1), goal_handle}
    .detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto feedback = std::make_shared<ExampleAction::Feedback>();
    auto result = std::make_shared<ExampleAction::Result>();

    const int32_t order = goal_handle->get_goal()->order;
    auto & sequence = feedback->partial_sequence;
    sequence.push_back(0);
    if (order > 1) {
      sequence.push_back(1);
    }

    rclcpp::Rate loop_rate(2.0);
    while (sequence.size() < static_cast<size_t>(order)) {
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled at %zu/%d", sequence.size(), order);
        return;
      }

      const size_t n = sequence.size();
      sequence.push_back(sequence[n - 1] + sequence[n - 2]);

      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(
        this->get_logger(), "Feedback: %zu/%d numbers", sequence.size(), order);
      loop_rate.sleep();
    }

    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  rclcpp_action::Server<ExampleAction>::SharedPtr action_server_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActionServer>());
  rclcpp::shutdown();
  return 0;
}
