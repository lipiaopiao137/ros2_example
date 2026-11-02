// ROS 2 topic demo -- subscriber side.
//
// Subscribes to /example_topic and prints every message. Subscribers are
// passive: the callback only runs when a message arrives, and the node keeps
// spinning otherwise.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_use_example_interfaces/msg/example_message.hpp"

using ExampleMessage = ros2_use_example_interfaces::msg::ExampleMessage;

class TopicSubscriber : public rclcpp::Node
{
public:
  TopicSubscriber()
  : Node("topic_subscriber")
  {
    // The subscription QoS depth must be compatible with the publisher's.
    subscription_ = this->create_subscription<ExampleMessage>(
      "example_topic", 10,
      std::bind(&TopicSubscriber::message_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Listening on /example_topic");
  }

private:
  void message_callback(const ExampleMessage::SharedPtr msg)
  {
    RCLCPP_INFO(
      this->get_logger(), "Received: count=%d frame_id='%s' content='%s'",
      msg->count, msg->header.frame_id.c_str(), msg->content.c_str());
  }

  rclcpp::Subscription<ExampleMessage>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicSubscriber>());
  rclcpp::shutdown();
  return 0;
}
