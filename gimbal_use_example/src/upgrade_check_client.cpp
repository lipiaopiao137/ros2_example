// Gimbal upgrade demo -- client side.
//
// Asks the server whether an upgrade to target_version is allowed, prints the
// answer and exits. Usage:
//
//     ros2 run gimbal_use_example upgrade_check_client [current] [target] [--force]
//
// Exit codes: 0 = upgrade approved, 2 = upgrade refused, 1 = call failed.

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include "gimbal_use_example_interfaces/srv/upgrade_check.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using UpgradeCheck = gimbal_use_example_interfaces::srv::UpgradeCheck;

class UpgradeCheckClient : public rclcpp::Node
{
public:
  UpgradeCheckClient()
  : Node("upgrade_check_client")
  {
    client_ = this->create_client<UpgradeCheck>("gimbal_upgrade_check");
  }

  std::shared_future<std::shared_ptr<UpgradeCheck::Response>> send_request(
    const std::string & current_version,
    const std::string & target_version,
    bool force)
  {
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service");
        return {};
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for /gimbal_upgrade_check ...");
    }

    auto request = std::make_shared<UpgradeCheck::Request>();
    request->current_version = current_version;
    request->target_version = target_version;
    request->force = force;

    RCLCPP_INFO(
      this->get_logger(), "Requesting check: %s -> %s (force=%s)",
      current_version.c_str(), target_version.c_str(), force ? "true" : "false");

    return client_->async_send_request(request).future.share();
  }

private:
  rclcpp::Client<UpgradeCheck>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const std::string current_version = argc > 1 ? argv[1] : "1.0.0";
  const std::string target_version = argc > 2 ? argv[2] : "1.1.0";
  const bool force = argc > 3 && std::string(argv[3]) == "--force";

  auto node = std::make_shared<UpgradeCheckClient>();
  auto future = node->send_request(current_version, target_version, force);

  if (rclcpp::spin_until_future_complete(node, future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Upgrade check call failed");
    rclcpp::shutdown();
    return 1;
  }

  const auto response = future.get();
  if (response->can_upgrade) {
    RCLCPP_INFO(
      node->get_logger(), "Upgrade approved: %s (~%d s)",
      response->reason.c_str(), response->estimated_duration_sec);
    rclcpp::shutdown();
    return 0;
  }

  RCLCPP_WARN(node->get_logger(), "Upgrade refused: %s", response->reason.c_str());
  rclcpp::shutdown();
  return 2;
}
