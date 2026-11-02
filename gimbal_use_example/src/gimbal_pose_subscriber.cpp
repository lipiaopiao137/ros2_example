// Gimbal pose demo -- subscriber process.
//
// Receives GimbalPose and prints it. The log is throttled to twice per second
// so a 20 Hz stream stays readable; the last received pose is what a control
// loop would actually consume.

#include <memory>

#include "gimbal_use_example_interfaces/msg/gimbal_pose.hpp"
#include "rclcpp/rclcpp.hpp"

using GimbalPose = gimbal_use_example_interfaces::msg::GimbalPose;

class GimbalPoseSubscriber : public rclcpp::Node
{
public:
  GimbalPoseSubscriber()
  : Node("gimbal_pose_subscriber")
  {
    subscription_ = this->create_subscription<GimbalPose>(
      "gimbal_pose", 10,
      std::bind(&GimbalPoseSubscriber::pose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Listening on /gimbal_pose");
  }

private:
  void pose_callback(const GimbalPose::SharedPtr msg)
  {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "Pose: yaw=%.2f pitch=%.2f roll=%.2f deg (yaw_rate=%.2f deg/s)",
      msg->yaw, msg->pitch, msg->roll, msg->yaw_rate);
  }

  rclcpp::Subscription<GimbalPose>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalPoseSubscriber>());
  rclcpp::shutdown();
  return 0;
}
