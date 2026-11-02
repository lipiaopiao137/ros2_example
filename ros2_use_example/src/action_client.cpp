// ROS 2 action demo -- client side.
//
// Sends a Fibonacci goal, prints streamed feedback, and prints the final
// result. Usage:
//
//     ros2 run ros2_use_example action_client [order] [cancel_after_seconds]
//
// Examples:
//     ros2 run ros2_use_example action_client 10      # compute 10 numbers
//     ros2 run ros2_use_example action_client 20 2.5  # cancel after 2.5 s
//
// An action client has three separate channels instead of one request/response
// pair: goal response, feedback, and result.

#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_use_example_interfaces/action/example_action.hpp"

using namespace std::chrono_literals;
using ExampleAction = ros2_use_example_interfaces::action::ExampleAction;

class ActionClient : public rclcpp::Node
{
public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExampleAction>;

  ActionClient()
  : Node("action_client")
  {
    client_ = rclcpp_action::create_client<ExampleAction>(this, "example_action");
  }

  bool wait_for_server()
  {
    return client_->wait_for_action_server(5s);
  }

  std::shared_future<std::shared_ptr<GoalHandle>> send_goal(int32_t order)
  {
    ExampleAction::Goal goal;
    goal.order = order;

    auto options = rclcpp_action::Client<ExampleAction>::SendGoalOptions();
    options.feedback_callback =
      std::bind(&ActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

    return client_->async_send_goal(goal, options);
  }

  // Register a one-shot timer that cancels the goal after `seconds`.
  void request_cancel_after(double seconds, const std::shared_ptr<GoalHandle> & goal_handle)
  {
    cancel_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(seconds),
      [this, goal_handle]() {
        RCLCPP_INFO(this->get_logger(), "Requesting cancellation");
        client_->async_cancel_goal(goal_handle);
        cancel_timer_->cancel();
      });
  }

  std::shared_future<GoalHandle::WrappedResult> get_result(
    const std::shared_ptr<GoalHandle> & goal_handle)
  {
    return client_->async_get_result(goal_handle);
  }

private:
  void feedback_callback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const ExampleAction::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(), "Feedback: %zu numbers, last=%d",
      feedback->partial_sequence.size(), feedback->partial_sequence.back());
  }

  rclcpp_action::Client<ExampleAction>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const int32_t order = argc > 1 ? std::atoi(argv[1]) : 10;
  const double cancel_after = argc > 2 ? std::atof(argv[2]) : 0.0;

  auto node = std::make_shared<ActionClient>();

  if (!node->wait_for_server()) {
    RCLCPP_ERROR(node->get_logger(), "Action server /example_action not available");
    rclcpp::shutdown();
    return 1;
  }

  // Phase 1: send the goal and wait for the server to accept or reject it.
  auto goal_future = node->send_goal(order);
  if (rclcpp::spin_until_future_complete(node, goal_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Sending the goal failed");
    rclcpp::shutdown();
    return 1;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by the server");
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Goal accepted: order=%d", order);

  // Optional: demonstrate cancellation while the action is running.
  if (cancel_after > 0.0) {
    node->request_cancel_after(cancel_after, goal_handle);
  }

  // Phase 2: wait for the result. Feedback callbacks fire while we spin here.
  auto result_future = node->get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Waiting for the result failed");
    rclcpp::shutdown();
    return 1;
  }

  auto wrapped_result = result_future.get();

  std::ostringstream sequence;
  for (size_t i = 0; i < wrapped_result.result->sequence.size(); ++i) {
    sequence << (i ? ", " : "") << wrapped_result.result->sequence[i];
  }

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node->get_logger(), "Result: [%s]", sequence.str().c_str());
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(
        node->get_logger(), "Result: canceled, partial sequence [%s]",
        sequence.str().c_str());
      break;
    default:
      RCLCPP_ERROR(node->get_logger(), "Result: aborted");
      break;
  }

  rclcpp::shutdown();
  return 0;
}
