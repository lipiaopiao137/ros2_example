// CLI client for the auto-flight action.
//
// Usage:
//     ros2 run auto_flight_example auto_flight_client [altitude] [radius] [cancel_after]
//
// Default mission: take off to 3 m, fly a square of side `radius` (default
// 5 m) at the server's default speed, then land. A positive cancel_after
// cancels the mission after that many seconds, and the planner lands on the
// spot.
//
// Examples:
//     ros2 run auto_flight_example auto_flight_client
//     ros2 run auto_flight_example auto_flight_client 5 8
//     ros2 run auto_flight_example auto_flight_client 3 5 2.0

#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>

#include "auto_flight_interfaces/action/auto_flight.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using AutoFlight = auto_flight_interfaces::action::AutoFlight;

class AutoFlightClient : public rclcpp::Node
{
public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<AutoFlight>;

  AutoFlightClient()
  : Node("auto_flight_client")
  {
    client_ = rclcpp_action::create_client<AutoFlight>(this, "auto_flight/execute");
  }

  bool wait_for_server()
  {
    // Discovery can take a few seconds when the whole stack starts together.
    return client_->wait_for_action_server(15s);
  }

  std::shared_future<std::shared_ptr<GoalHandle>> send_mission(
    double takeoff_altitude_m, double radius_m)
  {
    AutoFlight::Goal goal;
    goal.takeoff_altitude_m = takeoff_altitude_m;
    goal.max_speed_mps = 0.0;  // let the server use its default speed

    // Square: (r,0) -> (r,r) -> (0,r) -> (0,0), then land at the origin.
    const double xs[] = {radius_m, radius_m, 0.0, 0.0};
    const double ys[] = {0.0, radius_m, radius_m, 0.0};
    for (size_t i = 0; i < 4; ++i) {
      geometry_msgs::msg::Pose2D waypoint;
      waypoint.x = xs[i];
      waypoint.y = ys[i];
      waypoint.theta = 0.0;
      goal.waypoints.push_back(waypoint);
    }

    auto options = rclcpp_action::Client<AutoFlight>::SendGoalOptions();
    options.feedback_callback =
      std::bind(&AutoFlightClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

    RCLCPP_INFO(
      this->get_logger(), "Sending mission: %.1f m altitude, %.1f m square",
      takeoff_altitude_m, radius_m);
    return client_->async_send_goal(goal, options);
  }

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
    const std::shared_ptr<const AutoFlight::Feedback> feedback)
  {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Feedback: %s wp %u/%u remaining %.1f m",
      feedback->state.c_str(), feedback->current_waypoint,
      feedback->total_waypoints, feedback->distance_remaining_m);
  }

  rclcpp_action::Client<AutoFlight>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const double altitude = argc > 1 ? std::atof(argv[1]) : 3.0;
  const double radius = argc > 2 ? std::atof(argv[2]) : 5.0;
  const double cancel_after = argc > 3 ? std::atof(argv[3]) : 0.0;

  auto node = std::make_shared<AutoFlightClient>();
  if (!node->wait_for_server()) {
    RCLCPP_ERROR(node->get_logger(), "Action server /auto_flight/execute not available");
    rclcpp::shutdown();
    return 1;
  }

  auto goal_future = node->send_mission(altitude, radius);
  if (rclcpp::spin_until_future_complete(node, goal_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Sending the mission failed");
    rclcpp::shutdown();
    return 1;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Mission rejected by the server");
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Mission accepted");

  if (cancel_after > 0.0) {
    node->request_cancel_after(cancel_after, goal_handle);
  }

  auto result_future = node->get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Waiting for the mission result failed");
    rclcpp::shutdown();
    return 1;
  }

  const auto wrapped = result_future.get();
  switch (wrapped.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(
        node->get_logger(),
        "Mission succeeded: %u/%u waypoints, %.1f m flown in %.1f s (%s)",
        wrapped.result->waypoints_completed, 4u, wrapped.result->distance_flown_m,
        wrapped.result->duration_s, wrapped.result->message.c_str());
      rclcpp::shutdown();
      return 0;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(
        node->get_logger(), "Mission canceled: %u waypoints completed (%s)",
        wrapped.result->waypoints_completed, wrapped.result->message.c_str());
      rclcpp::shutdown();
      return 2;
    default:
      RCLCPP_ERROR(
        node->get_logger(), "Mission aborted: %s", wrapped.result->message.c_str());
      rclcpp::shutdown();
      return 1;
  }
}
