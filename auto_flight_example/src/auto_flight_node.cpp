// Auto-flight mission executor -- the integration point of the project.
//
//   subscribes  /drone/pose, /drone/imu, /drone/odom     (topics)
//   publishes   /drone/cmd_vel, /auto_flight/status      (topics)
//   publishes   /auto_flight/waypoints                   (MarkerArray for RViz)
//   calls       /gimbal/get_info                          (service)
//   serves      /auto_flight/execute                      (action)
//               /auto_flight/preflight                    (service)
//
// A mission runs PREFLIGHT -> TAKEOFF -> CRUISE (per waypoint) -> LAND.
// Preflight refuses to launch when sensor data is stale, the vehicle is not
// level, or the gimbal firmware is older than required. Cancelling a mission
// lands the vehicle at its current position and reports CANCELED.
//
// The gimbal service is polled at 2 Hz into a cache, so the mission thread and
// the preflight service never block the executor waiting for a response.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "auto_flight_interfaces/action/auto_flight.hpp"
#include "auto_flight_interfaces/msg/flight_status.hpp"
#include "auto_flight_interfaces/srv/get_gimbal_info.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;
using AutoFlight = auto_flight_interfaces::action::AutoFlight;
using FlightStatus = auto_flight_interfaces::msg::FlightStatus;
using GetGimbalInfo = auto_flight_interfaces::srv::GetGimbalInfo;
using Trigger = std_srvs::srv::Trigger;

namespace {
enum class Step {
  Done,
  Canceled,
  Failed,
};

double elapsed_s(const std::chrono::steady_clock::time_point& start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}
}  // namespace

class AutoFlightNode : public rclcpp::Node {
 public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<AutoFlight>;

  AutoFlightNode() : Node("auto_flight_node") {
    default_speed_mps_ = this->declare_parameter<double>("default_speed_mps", 3.0);
    max_speed_mps_ = this->declare_parameter<double>("max_speed_mps", 5.0);
    takeoff_speed_mps_ = this->declare_parameter<double>("takeoff_speed_mps", 1.0);
    land_speed_mps_ = this->declare_parameter<double>("land_speed_mps", 0.8);
    waypoint_tolerance_m_ = this->declare_parameter<double>("waypoint_tolerance_m", 0.3);
    sensor_timeout_s_ = this->declare_parameter<double>("sensor_timeout_s", 0.5);
    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 20.0);
    max_altitude_m_ = this->declare_parameter<double>("max_altitude_m", 30.0);
    geofence_radius_m_ = this->declare_parameter<double>("geofence_radius_m", 50.0);
    max_tilt_deg_ = this->declare_parameter<double>("max_tilt_deg", 30.0);
    this->declare_parameter<bool>("require_gimbal_up_to_date", true);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "drone/pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          pose_ = *msg;
          pose_time_ = std::chrono::steady_clock::now();
          have_pose_ = true;
        });
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("drone/imu", 10,
                                                                [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                                                                  std::lock_guard<std::mutex> lock(data_mutex_);
                                                                  imu_ = *msg;
                                                                  imu_time_ = std::chrono::steady_clock::now();
                                                                  have_imu_ = true;
                                                                });
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "drone/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          odom_ = *msg;
          odom_time_ = std::chrono::steady_clock::now();
          have_odom_ = true;
        });

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("drone/cmd_vel", 10);
    status_pub_ = this->create_publisher<FlightStatus>("auto_flight/status", 10);

    // Transient local so an RViz started after the goal still receives the plan.
    rclcpp::QoS waypoint_qos(1);
    waypoint_qos.transient_local();
    waypoint_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("auto_flight/waypoints", waypoint_qos);

    gimbal_client_ = this->create_client<GetGimbalInfo>("gimbal/get_info");
    gimbal_poll_timer_ = this->create_wall_timer(2s, [this]() { poll_gimbal_info(); });
    poll_gimbal_info();

    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<AutoFlight>(
        this, "auto_flight/execute", std::bind(&AutoFlightNode::handle_goal, this, _1, _2),
        std::bind(&AutoFlightNode::handle_cancel, this, _1), std::bind(&AutoFlightNode::handle_accepted, this, _1));

    preflight_service_ = this->create_service<Trigger>(
        "auto_flight/preflight",
        [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
          std::string reason;
          response->success = preflight_ok(reason);
          response->message = reason;
          RCLCPP_INFO(this->get_logger(), "Preflight query: %s (%s)", response->success ? "ready" : "NOT ready",
                      reason.c_str());
        });

    status_timer_ = this->create_wall_timer(500ms, [this]() { publish_status(); });

    RCLCPP_INFO(this->get_logger(), "Auto-flight ready: action /auto_flight/execute, preflight /auto_flight/preflight");
  }

 private:
  // ---------------------------------------------------------------- topics

  void publish_velocity(double vx, double vy, double vz, double wz) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.linear.z = vz;
    cmd.angular.z = wz;
    cmd_pub_->publish(cmd);
  }

  bool sensors_ok() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!have_pose_ || !have_imu_ || !have_odom_) {
      return false;
    }
    const auto now = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::duration<double>(sensor_timeout_s_);
    return now - pose_time_ <= timeout && now - imu_time_ <= timeout && now - odom_time_ <= timeout;
  }

  bool latest_position(double& x, double& y, double& z) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!have_pose_ ||
        std::chrono::steady_clock::now() - pose_time_ > std::chrono::duration<double>(sensor_timeout_s_)) {
      return false;
    }
    x = pose_.pose.position.x;
    y = pose_.pose.position.y;
    z = pose_.pose.position.z;
    return true;
  }

  bool attitude_ok() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!have_imu_ || std::chrono::steady_clock::now() - imu_time_ > std::chrono::duration<double>(sensor_timeout_s_)) {
      return false;
    }

    const auto& q = imu_.orientation;
    const double roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    const double sin_pitch = std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0);
    const double pitch = std::asin(sin_pitch);
    constexpr double kPi = 3.14159265358979323846;
    const double limit = max_tilt_deg_ * kPi / 180.0;
    return std::abs(roll) <= limit && std::abs(pitch) <= limit;
  }

  // ------------------------------------------------------------- gimbal srv

  void poll_gimbal_info() {
    if (!gimbal_client_->service_is_ready()) {
      return;
    }
    if (gimbal_poll_inflight_.exchange(true)) {
      return;
    }

    auto request = std::make_shared<GetGimbalInfo::Request>();
    gimbal_client_->async_send_request(request, [this](rclcpp::Client<GetGimbalInfo>::SharedFuture future) {
      auto response = future.get();
      {
        std::lock_guard<std::mutex> lock(gimbal_mutex_);
        gimbal_info_ = *response;
        gimbal_info_time_ = std::chrono::steady_clock::now();
        have_gimbal_info_ = true;
      }
      gimbal_poll_inflight_ = false;

      if (response->upgrade_required) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Gimbal firmware upgrade required: %s",
                             response->reason.c_str());
      }
    });
  }

  bool cached_gimbal_info(GetGimbalInfo::Response& out, double max_age_s) {
    std::lock_guard<std::mutex> lock(gimbal_mutex_);
    if (!have_gimbal_info_ ||
        std::chrono::steady_clock::now() - gimbal_info_time_ > std::chrono::duration<double>(max_age_s)) {
      return false;
    }
    out = gimbal_info_;
    return true;
  }

  // Used by the mission thread only: the executor keeps running, so polling
  // the cache lets the service response arrive without spinning here.
  bool wait_for_gimbal_info(double timeout_s, GetGimbalInfo::Response& out) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
      if (cached_gimbal_info(out, 5.0)) {
        return true;
      }
      std::this_thread::sleep_for(50ms);
    }
    return false;
  }

  bool preflight_ok(std::string& reason) {
    if (!sensors_ok()) {
      reason = "pose/imu/odom data is stale or missing";
      return false;
    }
    if (!attitude_ok()) {
      reason = "vehicle is not level";
      return false;
    }

    GetGimbalInfo::Response info;
    if (!cached_gimbal_info(info, 5.0)) {
      reason = "gimbal info unavailable";
      return false;
    }
    if (info.upgrade_required && this->get_parameter("require_gimbal_up_to_date").as_bool()) {
      reason = "gimbal firmware upgrade required: " + info.reason;
      return false;
    }

    reason = "ready: " + info.model + " firmware " + info.firmware_version;
    return true;
  }

  // ----------------------------------------------------------------- action

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const AutoFlight::Goal> goal) {
    (void)uuid;

    if (goal->takeoff_altitude_m < 0.5 || goal->takeoff_altitude_m > max_altitude_m_) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal: takeoff altitude %.1f m outside [0.5, %.0f]",
                  goal->takeoff_altitude_m, max_altitude_m_);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->waypoints.empty() || goal->waypoints.size() > 50) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal: %zu waypoints (must be 1..50)", goal->waypoints.size());
      return rclcpp_action::GoalResponse::REJECT;
    }
    for (const auto& waypoint : goal->waypoints) {
      if (std::hypot(waypoint.x, waypoint.y) > geofence_radius_m_) {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal: waypoint (%.1f, %.1f) outside the %.0f m geofence", waypoint.x,
                    waypoint.y, geofence_radius_m_);
        return rclcpp_action::GoalResponse::REJECT;
      }
    }

    bool expected = false;
    if (!mission_active_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal: a mission is already running");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "Accepting mission: %.1f m altitude, %zu waypoints", goal->takeoff_altitude_m,
                goal->waypoints.size());
    publish_waypoint_markers(*goal);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Cancel request accepted; landing at current position");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread{std::bind(&AutoFlightNode::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  // Draws the planned route at cruise altitude plus numbered waypoint markers.
  void publish_waypoint_markers(const AutoFlight::Goal& goal) {
    visualization_msgs::msg::MarkerArray markers;
    const auto stamp = this->get_clock()->now();

    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    visualization_msgs::msg::Marker route;
    route.header.stamp = stamp;
    route.header.frame_id = "map";
    route.ns = "waypoint_route";
    route.type = visualization_msgs::msg::Marker::LINE_STRIP;
    route.action = visualization_msgs::msg::Marker::ADD;
    route.pose.orientation.w = 1.0;
    route.scale.x = 0.05;
    route.color.r = 0.1f;
    route.color.g = 0.8f;
    route.color.b = 1.0f;
    route.color.a = 1.0f;

    geometry_msgs::msg::Point start;
    start.z = goal.takeoff_altitude_m;
    route.points.push_back(start);
    for (const auto& waypoint : goal.waypoints) {
      geometry_msgs::msg::Point point;
      point.x = waypoint.x;
      point.y = waypoint.y;
      point.z = goal.takeoff_altitude_m;
      route.points.push_back(point);
    }
    markers.markers.push_back(route);

    int32_t id = 1;
    for (size_t i = 0; i < goal.waypoints.size(); ++i) {
      const auto& waypoint = goal.waypoints[i];

      visualization_msgs::msg::Marker sphere;
      sphere.header.stamp = stamp;
      sphere.header.frame_id = "map";
      sphere.ns = "waypoints";
      sphere.id = id++;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position.x = waypoint.x;
      sphere.pose.position.y = waypoint.y;
      sphere.pose.position.z = goal.takeoff_altitude_m;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = 0.4;
      sphere.scale.y = 0.4;
      sphere.scale.z = 0.4;
      sphere.color.r = 1.0f;
      sphere.color.g = 0.6f;
      sphere.color.b = 0.0f;
      sphere.color.a = 1.0f;
      markers.markers.push_back(sphere);

      visualization_msgs::msg::Marker label;
      label.header.stamp = stamp;
      label.header.frame_id = "map";
      label.ns = "waypoint_labels";
      label.id = id++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = waypoint.x;
      label.pose.position.y = waypoint.y;
      label.pose.position.z = goal.takeoff_altitude_m + 0.8;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.5;
      label.color.r = 1.0f;
      label.color.g = 1.0f;
      label.color.b = 1.0f;
      label.color.a = 1.0f;
      label.text = "P" + std::to_string(i + 1);
      markers.markers.push_back(label);
    }

    waypoint_pub_->publish(markers);
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    // Release the mission slot on every return path.
    struct MissionGuard {
      std::atomic<bool>& flag;
      ~MissionGuard() { flag = false; }
    } guard{mission_active_};

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<AutoFlight::Feedback>();
    auto result = std::make_shared<AutoFlight::Result>();
    const auto total = static_cast<uint32_t>(goal->waypoints.size());
    feedback->total_waypoints = total;

    const auto mission_start = std::chrono::steady_clock::now();
    uint32_t completed = 0;

    auto finish = [&](bool success, bool canceled, const std::string& message) {
      result->success = success;
      result->message = message;
      result->waypoints_completed = completed;
      result->duration_s = elapsed_s(mission_start);
      publish_velocity(0, 0, 0, 0);
      set_status("IDLE", completed, total);
      if (!rclcpp::ok()) {
        return;
      }
      if (canceled) {
        goal_handle->canceled(result);
      } else if (success) {
        goal_handle->succeed(result);
      } else {
        goal_handle->abort(result);
      }
    };

    // ---- PREFLIGHT -----------------------------------------------------
    set_status("PREFLIGHT", 0, total);
    publish_velocity(0, 0, 0, 0);

    GetGimbalInfo::Response gimbal;
    if (!wait_for_gimbal_info(5.0, gimbal)) {
      finish(false, false, "preflight failed: gimbal info service unavailable");
      return;
    }
    if (goal_handle->is_canceling()) {
      finish(false, true, "canceled during preflight");
      return;
    }
    if (gimbal.upgrade_required && this->get_parameter("require_gimbal_up_to_date").as_bool()) {
      finish(false, false, "preflight failed: " + gimbal.reason + "; upgrade the gimbal before flying");
      return;
    }
    if (!sensors_ok()) {
      finish(false, false, "preflight failed: pose/imu/odom data is stale");
      return;
    }
    if (!attitude_ok()) {
      finish(false, false, "preflight failed: vehicle is not level");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Preflight passed: gimbal %s %s, firmware %s", gimbal.model.c_str(),
                gimbal.serial.c_str(), gimbal.firmware_version.c_str());

    // ---- TAKEOFF -------------------------------------------------------
    set_status("TAKEOFF", 0, total);
    Step step = climb_to(goal_handle, goal->takeoff_altitude_m, feedback);
    if (step != Step::Done) {
      land(goal_handle, feedback, total);
      finish(false, step == Step::Canceled,
             step == Step::Canceled ? "canceled during takeoff; landed" : "takeoff failed: lost sensor data; landed");
      return;
    }

    // ---- CRUISE --------------------------------------------------------
    const double speed = goal->max_speed_mps > 0.0 ? std::min(goal->max_speed_mps, max_speed_mps_) : default_speed_mps_;

    for (size_t i = 0; i < goal->waypoints.size(); ++i) {
      set_status("CRUISE", completed, total);
      step = fly_to(goal_handle, i, goal->waypoints, speed, feedback, result);
      if (step != Step::Done) {
        land(goal_handle, feedback, total);
        finish(false, step == Step::Canceled,
               step == Step::Canceled ? "canceled during cruise; landed" : "cruise failed: lost sensor data; landed");
        return;
      }
      completed = static_cast<uint32_t>(i + 1);
    }

    // ---- LAND ----------------------------------------------------------
    land(goal_handle, feedback, total);

    if (!rclcpp::ok()) {
      return;
    }
    finish(true, false, "mission complete: " + std::to_string(completed) + "/" + std::to_string(total) + " waypoints");
  }

  Step climb_to(const std::shared_ptr<GoalHandle>& goal_handle, double target_z,
                const std::shared_ptr<AutoFlight::Feedback>& feedback) {
    rclcpp::Rate rate(control_rate_hz_);
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        return Step::Canceled;
      }

      double x, y, z;
      if (!latest_position(x, y, z) || !sensors_ok()) {
        return Step::Failed;
      }

      const double error = target_z - z;
      if (std::abs(error) < waypoint_tolerance_m_) {
        publish_velocity(0, 0, 0, 0);
        return Step::Done;
      }

      publish_velocity(0, 0, std::clamp(error, -takeoff_speed_mps_, takeoff_speed_mps_), 0);
      publish_feedback(goal_handle, feedback, "TAKEOFF", 0, 0.0, x, y, z);
      rate.sleep();
    }
    return Step::Failed;
  }

  Step fly_to(const std::shared_ptr<GoalHandle>& goal_handle, size_t index,
              const std::vector<geometry_msgs::msg::Pose2D>& waypoints, double speed,
              const std::shared_ptr<AutoFlight::Feedback>& feedback,
              const std::shared_ptr<AutoFlight::Result>& result) {
    rclcpp::Rate rate(control_rate_hz_);
    double prev_x = 0.0;
    double prev_y = 0.0;
    bool have_prev = false;

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        return Step::Canceled;
      }

      double x, y, z;
      if (!latest_position(x, y, z) || !sensors_ok()) {
        return Step::Failed;
      }

      const double dx = waypoints[index].x - x;
      const double dy = waypoints[index].y - y;
      const double distance = std::hypot(dx, dy);
      if (distance < waypoint_tolerance_m_) {
        publish_velocity(0, 0, 0, 0);
        return Step::Done;
      }

      if (have_prev) {
        result->distance_flown_m += std::hypot(x - prev_x, y - prev_y);
      }
      prev_x = x;
      prev_y = y;
      have_prev = true;

      // Slow down near the target so arrivals are smooth.
      const double command = std::min(speed, std::max(0.2, distance));
      publish_velocity(dx / distance * command, dy / distance * command, 0, 0);
      publish_feedback(goal_handle, feedback, "CRUISE", static_cast<uint32_t>(index + 1),
                       remaining_distance(x, y, index, waypoints), x, y, z);
      rate.sleep();
    }
    return Step::Failed;
  }

  void land(const std::shared_ptr<GoalHandle>& goal_handle, const std::shared_ptr<AutoFlight::Feedback>& feedback,
            uint32_t total) {
    set_status("LAND", feedback->current_waypoint, total);
    rclcpp::Rate rate(control_rate_hz_);
    const auto deadline = std::chrono::steady_clock::now() + 10s;

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      double x, y, z;
      if (!latest_position(x, y, z)) {
        break;
      }
      if (z <= 0.05) {
        break;
      }
      publish_velocity(0, 0, -land_speed_mps_, 0);
      publish_feedback(goal_handle, feedback, "LAND", feedback->current_waypoint, 0.0, x, y, z);
      rate.sleep();
    }
    publish_velocity(0, 0, 0, 0);
    if (rclcpp::ok()) {
      RCLCPP_INFO(this->get_logger(), "Landed");
    }
  }

  void publish_feedback(const std::shared_ptr<GoalHandle>& goal_handle,
                        const std::shared_ptr<AutoFlight::Feedback>& feedback, const std::string& state,
                        uint32_t current_waypoint, double distance_remaining, double x, double y, double z) {
    feedback->state = state;
    feedback->current_waypoint = current_waypoint;
    feedback->distance_remaining_m = distance_remaining;
    feedback->current_pose.header.stamp = this->get_clock()->now();
    feedback->current_pose.header.frame_id = "map";
    feedback->current_pose.pose.position.x = x;
    feedback->current_pose.pose.position.y = y;
    feedback->current_pose.pose.position.z = z;
    goal_handle->publish_feedback(feedback);
    set_status(state, current_waypoint, feedback->total_waypoints);
  }

  double remaining_distance(double x, double y, size_t index,
                            const std::vector<geometry_msgs::msg::Pose2D>& waypoints) const {
    double distance = std::hypot(waypoints[index].x - x, waypoints[index].y - y);
    for (size_t i = index + 1; i < waypoints.size(); ++i) {
      distance += std::hypot(waypoints[i].x - waypoints[i - 1].x, waypoints[i].y - waypoints[i - 1].y);
    }
    return distance;
  }

  // ---------------------------------------------------------------- status

  void set_status(const std::string& state, uint32_t current, uint32_t total) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    state_ = state;
    current_waypoint_ = current;
    total_waypoints_ = total;
  }

  void publish_status() {
    FlightStatus status;
    status.header.stamp = this->get_clock()->now();
    status.header.frame_id = "map";

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status.state = state_;
      status.current_waypoint = current_waypoint_;
      status.total_waypoints = total_waypoints_;
    }
    {
      std::lock_guard<std::mutex> lock(gimbal_mutex_);
      if (have_gimbal_info_) {
        status.gimbal_firmware_version = gimbal_info_.firmware_version;
        status.gimbal_upgrade_required = gimbal_info_.upgrade_required;
      }
    }
    status.sensors_ok = sensors_ok();

    status_pub_->publish(status);
  }

  // ---------------------------------------------------------------- members

  double default_speed_mps_;
  double max_speed_mps_;
  double takeoff_speed_mps_;
  double land_speed_mps_;
  double waypoint_tolerance_m_;
  double sensor_timeout_s_;
  double control_rate_hz_;
  double max_altitude_m_;
  double geofence_radius_m_;
  double max_tilt_deg_;

  std::mutex data_mutex_;
  geometry_msgs::msg::PoseStamped pose_;
  sensor_msgs::msg::Imu imu_;
  nav_msgs::msg::Odometry odom_;
  bool have_pose_{false};
  bool have_imu_{false};
  bool have_odom_{false};
  std::chrono::steady_clock::time_point pose_time_;
  std::chrono::steady_clock::time_point imu_time_;
  std::chrono::steady_clock::time_point odom_time_;

  std::mutex gimbal_mutex_;
  GetGimbalInfo::Response gimbal_info_;
  bool have_gimbal_info_{false};
  std::chrono::steady_clock::time_point gimbal_info_time_;
  std::atomic<bool> gimbal_poll_inflight_{false};

  std::mutex status_mutex_;
  std::string state_{"IDLE"};
  uint32_t current_waypoint_{0};
  uint32_t total_waypoints_{0};

  std::atomic<bool> mission_active_{false};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<FlightStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_pub_;
  rclcpp::Client<GetGimbalInfo>::SharedPtr gimbal_client_;
  rclcpp::TimerBase::SharedPtr gimbal_poll_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::Service<Trigger>::SharedPtr preflight_service_;
  rclcpp_action::Server<AutoFlight>::SharedPtr action_server_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoFlightNode>());
  rclcpp::shutdown();
  return 0;
}
