// Gimbal move demo -- action client side.
//
// Sends a yaw-rotation goal, prints the streamed feedback and the final
// result. Usage:
//
//     ros2 run gimbal_use_example gimbal_move_client [delta_yaw_deg] [speed_dps] [cancel_after_s]
//
// Examples:
//     ros2 run gimbal_use_example gimbal_move_client 30        # +30 deg at default speed
//     ros2 run gimbal_use_example gimbal_move_client 30 60     # +30 deg at 60 deg/s
//     ros2 run gimbal_use_example gimbal_move_client 90 30 1.0 # cancel after 1 s

#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>

#include "gimbal_use_example_interfaces/action/gimbal_move.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using GimbalMove = gimbal_use_example_interfaces::action::GimbalMove;

class GimbalMoveClient : public rclcpp::Node
{
public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<GimbalMove>;

  GimbalMoveClient()
  : Node("gimbal_move_client")
  {
    client_ = rclcpp_action::create_client<GimbalMove>(this, "gimbal_move");
  }

  bool wait_for_server()
  {
    return client_->wait_for_action_server(5s);
  }

  std::shared_future<std::shared_ptr<GoalHandle>> send_goal(
    double delta_yaw_deg, double speed_dps)
  {
    GimbalMove::Goal goal;
    goal.delta_yaw_deg = delta_yaw_deg;
    goal.speed_dps = speed_dps;

    auto options = rclcpp_action::Client<GimbalMove>::SendGoalOptions();
    options.feedback_callback =
      std::bind(
        &GimbalMoveClient::feedback_callback, this,
        std::placeholders::_1, std::placeholders::_2);

    RCLCPP_INFO(
      this->get_logger(), "Sending goal: delta_yaw=%.1f deg, speed=%.1f deg/s",
      delta_yaw_deg, speed_dps);
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
    const std::shared_ptr<const GimbalMove::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(), "Feedback: yaw=%.1f deg progress=%.0f%%",
      feedback->current_yaw_deg, feedback->progress * 100.0);
  }

  rclcpp_action::Client<GimbalMove>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const double delta_yaw_deg = argc > 1 ? std::atof(argv[1]) : 30.0;
  const double speed_dps = argc > 2 ? std::atof(argv[2]) : 0.0;
  const double cancel_after = argc > 3 ? std::atof(argv[3]) : 0.0;

  auto node = std::make_shared<GimbalMoveClient>();

  if (!node->wait_for_server()) {
    RCLCPP_ERROR(node->get_logger(), "Action server /gimbal_move not available");
    rclcpp::shutdown();
    return 1;
  }

  // Phase 1: send the goal and wait for the server to accept or reject it.
  auto goal_future = node->send_goal(delta_yaw_deg, speed_dps);
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
  RCLCPP_INFO(node->get_logger(), "Goal accepted");

  if (cancel_after > 0.0) {
    node->request_cancel_after(cancel_after, goal_handle);
  }

  // Phase 2: wait for the result; feedback callbacks fire while spinning here.
  auto result_future = node->get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Waiting for the result failed");
    rclcpp::shutdown();
    return 1;
  }

  const auto wrapped_result = result_future.get();
  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(
        node->get_logger(), "Result: success, final yaw=%.1f deg (%s)",
        wrapped_result.result->final_yaw_deg, wrapped_result.result->message.c_str());
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(
        node->get_logger(), "Result: canceled at yaw=%.1f deg (%s)",
        wrapped_result.result->final_yaw_deg, wrapped_result.result->message.c_str());
      break;
    default:
      RCLCPP_ERROR(node->get_logger(), "Result: aborted");
      break;
  }

  rclcpp::shutdown();
  return wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED ? 0 : 2;
}
