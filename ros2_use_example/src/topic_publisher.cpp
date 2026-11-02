// ROS 2 topic demo -- publisher side.
//
// Topics are the "stream" mechanism: one publisher sends messages on a named
// channel and any number of subscribers receive them. Delivery is one-way and
// decoupled -- the publisher does not know (or wait for) its subscribers, and
// publishing with zero subscribers is not an error.
//
// This node publishes ross2_use_example_interfaces/msg/ExampleMessage on
// /example_topic once per second.

#include <chrono>
#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_use_example_interfaces/msg/example_message.hpp"

using namespace std::chrono_literals;

class TopicPublisher : public rclcpp::Node
{
public:
  TopicPublisher()
  : Node("topic_publisher"), count_(0)
  {
    // Depth 10: keep up to 10 unread messages in the publisher's outgoing
    // queue. The subscriber must use a compatible QoS to connect.
    publisher_ = this->create_publisher<ros2_use_example_interfaces::msg::ExampleMessage>(
      "example_topic", 10);
    timer_ = this->create_wall_timer(
      1s, std::bind(&TopicPublisher::publish_message, this));

    RCLCPP_INFO(this->get_logger(), "Publishing on /example_topic every second");
  }

private:
  void publish_message()
  {
    auto message = ros2_use_example_interfaces::msg::ExampleMessage();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "map";
    message.content = "hello from topic_publisher";
    message.count = count_++;

    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published: count=%d", message.count);
  }

  rclcpp::Publisher<ros2_use_example_interfaces::msg::ExampleMessage>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int32_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicPublisher>());
  rclcpp::shutdown();
  return 0;
}
