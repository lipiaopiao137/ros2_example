// ROS 2 service demo -- client side.
//
// Sends one request and waits for the reply, then exits. Usage:
//
//     ros2 run ros2_use_example service_client [a] [b]
//
// The two steps below are the whole service client pattern:
//   1. wait_for_service() until the server is up
//   2. async_send_request() + spin_until_future_complete() to get the response

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_use_example_interfaces/srv/example_service.hpp"

using namespace std::chrono_literals;
using ExampleService = ros2_use_example_interfaces::srv::ExampleService;

class ServiceClient : public rclcpp::Node
{
public:
  ServiceClient()
  : Node("service_client")
  {
    client_ = this->create_client<ExampleService>("example_service");
  }

  std::shared_future<std::shared_ptr<ExampleService::Response>> send_request(
    int64_t a, int64_t b)
  {
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service");
        return {};
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for /example_service ...");
    }

    auto request = std::make_shared<ExampleService::Request>();
    request->a = a;
    request->b = b;

    RCLCPP_INFO(this->get_logger(), "Sending request: %ld + %ld", a, b);
    return client_->async_send_request(request).future.share();
  }

private:
  rclcpp::Client<ExampleService>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  int64_t a = argc > 1 ? std::atoll(argv[1]) : 2;
  int64_t b = argc > 2 ? std::atoll(argv[2]) : 3;

  auto node = std::make_shared<ServiceClient>();
  auto future = node->send_request(a, b);

  if (rclcpp::spin_until_future_complete(node, future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Response: sum=%ld", future.get()->sum);

  rclcpp::shutdown();
  return 0;
}
