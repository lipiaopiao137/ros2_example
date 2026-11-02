// ROS 2 service demo -- server side.
//
// Services are the "request/response" mechanism: a client blocks (or waits on
// a future) until the server replies. Unlike a topic, exactly one server owns
// a service name and every request gets exactly one response.
//
// This node serves /example_service with ExampleService (a + b -> sum).

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_use_example_interfaces/srv/example_service.hpp"

using ExampleService = ros2_use_example_interfaces::srv::ExampleService;

class ServiceServer : public rclcpp::Node
{
public:
  ServiceServer()
  : Node("service_server")
  {
    service_ = this->create_service<ExampleService>(
      "example_service",
      std::bind(
        &ServiceServer::handle_request, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Serving /example_service (a + b)");
  }

private:
  void handle_request(
    const std::shared_ptr<ExampleService::Request> request,
    std::shared_ptr<ExampleService::Response> response)
  {
    response->sum = request->a + request->b;
    RCLCPP_INFO(
      this->get_logger(), "Request: %ld + %ld = %ld",
      request->a, request->b, response->sum);
  }

  rclcpp::Service<ExampleService>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServiceServer>());
  rclcpp::shutdown();
  return 0;
}
