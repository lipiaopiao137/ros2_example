// Gimbal pose demo -- publisher process.
//
// Simulates a gimbal sweeping its yaw/pitch axes and publishes GimbalPose on
// /gimbal_pose at 20 Hz. Run it in one terminal and gimbal_pose_subscriber in
// another: two independent processes communicating only through the topic.
//
// Parameters:
//   rate_hz            publish frequency (default 20.0)
//   yaw_amplitude_deg  yaw sweep amplitude (default 30.0)

#include <chrono>
#include <cmath>
#include <memory>

#include "gimbal_use_example_interfaces/msg/gimbal_pose.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using GimbalPose = gimbal_use_example_interfaces::msg::GimbalPose;

class GimbalPosePublisher : public rclcpp::Node
{
public:
  GimbalPosePublisher()
  : Node("gimbal_pose_publisher")
  {
    rate_hz_ = this->declare_parameter<double>("rate_hz", 20.0);
    yaw_amplitude_deg_ = this->declare_parameter<double>("yaw_amplitude_deg", 30.0);

    publisher_ = this->create_publisher<GimbalPose>("gimbal_pose", 10);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz_)),
      std::bind(&GimbalPosePublisher::publish_pose, this));
    start_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      this->get_logger(), "Publishing /gimbal_pose at %.1f Hz (yaw +-%.0f deg)",
      rate_hz_, yaw_amplitude_deg_);
  }

private:
  void publish_pose()
  {
    const double t = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time_).count();

    GimbalPose msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "gimbal_base";
    msg.yaw = yaw_amplitude_deg_ * std::sin(0.5 * t);
    msg.pitch = 10.0 + 5.0 * std::sin(0.3 * t);
    msg.roll = 2.0 * std::sin(0.2 * t);
    msg.yaw_rate = yaw_amplitude_deg_ * 0.5 * std::cos(0.5 * t);
    msg.pitch_rate = 5.0 * 0.3 * std::cos(0.3 * t);
    msg.roll_rate = 2.0 * 0.2 * std::cos(0.2 * t);

    publisher_->publish(msg);
  }

  double rate_hz_;
  double yaw_amplitude_deg_;
  rclcpp::Publisher<GimbalPose>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point start_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
