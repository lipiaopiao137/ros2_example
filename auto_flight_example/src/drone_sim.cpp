// Self-contained vehicle simulator for the auto-flight demo.
//
// Integrates world-frame velocity commands from /drone/cmd_vel into a simple
// point-mass model with yaw, and publishes the three streams the planner
// consumes:
//
//     /drone/pose   geometry_msgs/PoseStamped
//     /drone/imu    sensor_msgs/Imu
//     /drone/odom   nav_msgs/Odometry
//     /drone/trail  nav_msgs/Path           (accumulated flight path for RViz)
//     /tf           map -> odom -> base_link
//
// Frames: "map" is ENU with z up, unlike PX4's NED. cmd_vel linear components
// are world-frame m/s and angular.z is the yaw rate in rad/s. There is no
// aerodynamic model and no gravity in the IMU; this is a kinematic stand-in
// for a real flight controller. odom coincides with map in this simulator, so
// map -> odom is published as identity.
//
// Parameters:
//   publish_rate_hz  sensor publish rate     (default 50.0)
//   initial_x/y/z    starting position       (default 0.0)
//   max_speed_mps    command clamp           (default 10.0)
//   trail_rate_hz    Path publish rate       (default 2.0)
//   trail_max_points accumulated poses kept  (default 20000)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class DroneSim : public rclcpp::Node {
 public:
  DroneSim() : Node("drone_sim") {
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
    max_speed_mps_ = this->declare_parameter<double>("max_speed_mps", 10.0);
    trail_rate_hz_ = this->declare_parameter<double>("trail_rate_hz", 2.0);
    trail_max_points_ = this->declare_parameter<int>("trail_max_points", 20000);
    x_ = this->declare_parameter<double>("initial_x", 0.0);
    y_ = this->declare_parameter<double>("initial_y", 0.0);
    z_ = this->declare_parameter<double>("initial_z", 0.0);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "drone/cmd_vel", 10, std::bind(&DroneSim::cmd_callback, this, std::placeholders::_1));

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("drone/pose", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("drone/odom", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("drone/imu", 10);
    trail_pub_ = this->create_publisher<nav_msgs::msg::Path>("drone/trail", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    trail_.header.frame_id = "map";

    trail_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                               std::chrono::duration<double>(1.0 / std::max(0.1, trail_rate_hz_))),
                                           std::bind(&DroneSim::publish_trail, this));

    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_hz_)),
        std::bind(&DroneSim::update, this));
    last_update_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "Vehicle sim up: /drone/{pose,imu,odom} at %.0f Hz, cmd on /drone/cmd_vel",
                publish_rate_hz_);
  }

 private:
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_vx_ = std::clamp(msg->linear.x, -max_speed_mps_, max_speed_mps_);
    cmd_vy_ = std::clamp(msg->linear.y, -max_speed_mps_, max_speed_mps_);
    cmd_vz_ = std::clamp(msg->linear.z, -max_speed_mps_, max_speed_mps_);
    cmd_wz_ = msg->angular.z;
  }

  void update() {
    const auto now_time = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now_time - last_update_).count();
    last_update_ = now_time;
    if (dt <= 0.0) {
      return;
    }

    double vx, vy, vz, wz;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      vx = cmd_vx_;
      vy = cmd_vy_;
      vz = cmd_vz_;
      wz = cmd_wz_;
    }

    x_ += vx * dt;
    y_ += vy * dt;
    z_ += vz * dt;
    yaw_ += wz * dt;

    // Ground contact: the vehicle cannot sink below the takeoff plane.
    if (z_ < 0.0) {
      z_ = 0.0;
      vz = 0.0;
    }
    yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));

    // World-frame acceleration for the IMU, rotated into the body frame.
    const double ax = dt > 0.0 ? (vx - prev_vx_) / dt : 0.0;
    const double ay = dt > 0.0 ? (vy - prev_vy_) / dt : 0.0;
    const double az = dt > 0.0 ? (vz - prev_vz_) / dt : 0.0;
    prev_vx_ = vx;
    prev_vy_ = vy;
    prev_vz_ = vz;

    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);

    const auto stamp = this->get_clock()->now();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "map";
    pose.pose.position.x = x_;
    pose.pose.position.y = y_;
    pose.pose.position.z = z_;
    pose.pose.orientation.z = std::sin(yaw_ * 0.5);
    pose.pose.orientation.w = std::cos(yaw_ * 0.5);
    pose_pub_->publish(pose);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose = pose.pose;
    odom.twist.twist.linear.x = c * vx + s * vy;
    odom.twist.twist.linear.y = -s * vx + c * vy;
    odom.twist.twist.linear.z = vz;
    odom.twist.twist.angular.z = wz;
    odom_pub_->publish(odom);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "base_link";
    imu.orientation = pose.pose.orientation;
    imu.angular_velocity.z = wz;
    imu.linear_acceleration.x = c * ax + s * ay;
    imu.linear_acceleration.y = -s * ax + c * ay;
    imu.linear_acceleration.z = az;
    imu_pub_->publish(imu);

    broadcast_tf(stamp);
    append_trail(pose);
  }

  void broadcast_tf(const rclcpp::Time& stamp) {
    // odom and map coincide in this simulator, so the first transform is identity.
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = stamp;
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = "odom";
    map_to_odom.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(map_to_odom);

    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header.stamp = stamp;
    odom_to_base.header.frame_id = "odom";
    odom_to_base.child_frame_id = "base_link";
    odom_to_base.transform.translation.x = x_;
    odom_to_base.transform.translation.y = y_;
    odom_to_base.transform.translation.z = z_;
    odom_to_base.transform.rotation.z = std::sin(yaw_ * 0.5);
    odom_to_base.transform.rotation.w = std::cos(yaw_ * 0.5);
    tf_broadcaster_->sendTransform(odom_to_base);
  }

  void append_trail(const geometry_msgs::msg::PoseStamped& pose) {
    std::lock_guard<std::mutex> lock(trail_mutex_);
    if (!trail_.poses.empty()) {
      const auto& last = trail_.poses.back().pose.position;
      const auto& current = pose.pose.position;
      // Skip samples taken while hovering so the path stays compact.
      if (std::hypot(current.x - last.x, current.y - last.y) < 0.02 && std::abs(current.z - last.z) < 0.02) {
        return;
      }
    }
    trail_.poses.push_back(pose);
    if (static_cast<int>(trail_.poses.size()) > trail_max_points_) {
      trail_.poses.erase(trail_.poses.begin(), trail_.poses.begin() + (trail_.poses.size() - trail_max_points_));
    }
  }

  void publish_trail() {
    std::lock_guard<std::mutex> lock(trail_mutex_);
    if (trail_.poses.empty()) {
      return;
    }
    trail_.header.stamp = this->get_clock()->now();
    trail_pub_->publish(trail_);
  }

  double publish_rate_hz_;
  double max_speed_mps_;
  double trail_rate_hz_;
  int trail_max_points_;
  double x_;
  double y_;
  double z_;
  double yaw_{0.0};
  double prev_vx_{0.0};
  double prev_vy_{0.0};
  double prev_vz_{0.0};

  std::mutex cmd_mutex_;
  double cmd_vx_{0.0};
  double cmd_vy_{0.0};
  double cmd_vz_{0.0};
  double cmd_wz_{0.0};

  std::mutex trail_mutex_;
  nav_msgs::msg::Path trail_;

  std::chrono::steady_clock::time_point last_update_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trail_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr trail_timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneSim>());
  rclcpp::shutdown();
  return 0;
}
