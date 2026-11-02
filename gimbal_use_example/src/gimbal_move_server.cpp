// Gimbal move demo -- action server side.
//
// Serves /gimbal_move: the client asks "rotate yaw by +30 deg", the server
// accepts the goal, streams the current yaw while the gimbal moves, and
// finally reports success. The motion can be cancelled halfway.
//
// The gimbal state is simulated: yaw moves at a constant speed, one goal at a
// time, inside the configured soft limits.
//
// Parameters:
//   initial_yaw_deg    starting yaw                (default 0.0)
//   default_speed_dps  used when goal.speed_dps<=0 (default 30.0)
//   yaw_min_deg        lower soft limit            (default -170.0)
//   yaw_max_deg        upper soft limit            (default 170.0)
//   feedback_hz        feedback publish rate       (default 10.0)

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "gimbal_use_example_interfaces/action/gimbal_move.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using GimbalMove = gimbal_use_example_interfaces::action::GimbalMove;

class GimbalMoveServer : public rclcpp::Node
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<GimbalMove>;

  GimbalMoveServer()
  : Node("gimbal_move_server")
  {
    initial_yaw_deg_ = this->declare_parameter<double>("initial_yaw_deg", 0.0);
    default_speed_dps_ = this->declare_parameter<double>("default_speed_dps", 30.0);
    yaw_min_deg_ = this->declare_parameter<double>("yaw_min_deg", -170.0);
    yaw_max_deg_ = this->declare_parameter<double>("yaw_max_deg", 170.0);
    feedback_hz_ = this->declare_parameter<double>("feedback_hz", 10.0);
    yaw_.store(initial_yaw_deg_);

    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<GimbalMove>(
      this, "gimbal_move",
      std::bind(&GimbalMoveServer::handle_goal, this, _1, _2),
      std::bind(&GimbalMoveServer::handle_cancel, this, _1),
      std::bind(&GimbalMoveServer::handle_accepted, this, _1));

    RCLCPP_INFO(
      this->get_logger(),
      "Serving /gimbal_move (yaw range [%.0f, %.0f] deg, default speed %.0f deg/s)",
      yaw_min_deg_, yaw_max_deg_, default_speed_dps_);
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GimbalMove::Goal> goal)
  {
    (void)uuid;

    // Only one motion at a time: reject a new goal while the gimbal is moving.
    bool expected = false;
    if (!moving_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal: gimbal is already moving");
      return rclcpp_action::GoalResponse::REJECT;
    }

    const double target = yaw_.load() + goal->delta_yaw_deg;
    if (target < yaw_min_deg_ || target > yaw_max_deg_) {
      moving_ = false;
      RCLCPP_WARN(
        this->get_logger(), "Rejecting goal: target %.1f deg outside [%.0f, %.0f]",
        target, yaw_min_deg_, yaw_max_deg_);
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(
      this->get_logger(), "Accepting goal: yaw %.1f -> %.1f deg (delta %.1f)",
      yaw_.load(), target, goal->delta_yaw_deg);
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
    // Moving blocks the thread while it sleeps between feedback messages, so
    // run it on its own thread and keep the executor free for cancel requests.
    std::thread{std::bind(&GimbalMoveServer::execute, this, std::placeholders::_1), goal_handle}
    .detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GimbalMove::Feedback>();
    auto result = std::make_shared<GimbalMove::Result>();

    const double start_yaw = yaw_.load();
    const double target_yaw = start_yaw + goal->delta_yaw_deg;
    const double speed = goal->speed_dps > 0.0 ? goal->speed_dps : default_speed_dps_;
    const double duration_s = std::abs(target_yaw - start_yaw) / speed;

    rclcpp::Rate rate(feedback_hz_);
    const auto start_time = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time).count();
      const double progress =
        duration_s > 0.0 ? std::min(elapsed / duration_s, 1.0) : 1.0;
      const double current_yaw = start_yaw + (target_yaw - start_yaw) * progress;
      yaw_.store(current_yaw);

      feedback->current_yaw_deg = current_yaw;
      feedback->progress = progress;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(
        this->get_logger(), "Feedback: yaw=%.1f deg progress=%.0f%%",
        current_yaw, progress * 100.0);

      if (progress >= 1.0) {
        result->success = true;
        result->final_yaw_deg = target_yaw;
        std::ostringstream message;
        message << "yaw moved to " << target_yaw << " deg";
        result->message = message.str();
        moving_ = false;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded: %s", result->message.c_str());
        return;
      }

      if (goal_handle->is_canceling()) {
        result->success = false;
        result->final_yaw_deg = current_yaw;
        result->message = "canceled while moving";
        moving_ = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(
          this->get_logger(), "Goal canceled at yaw=%.1f deg", current_yaw);
        return;
      }

      rate.sleep();
    }

    // Shut down mid-motion: release the gimbal so a restarted node can move.
    moving_ = false;
  }

  double initial_yaw_deg_;
  double default_speed_dps_;
  double yaw_min_deg_;
  double yaw_max_deg_;
  double feedback_hz_;
  std::atomic<double> yaw_;
  std::atomic<bool> moving_{false};
  rclcpp_action::Server<GimbalMove>::SharedPtr action_server_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalMoveServer>());
  rclcpp::shutdown();
  return 0;
}
