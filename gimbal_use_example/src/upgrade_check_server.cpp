// Gimbal upgrade demo -- server side (question/answer).
//
// Serves /gimbal_upgrade_check. The operator asks "can this gimbal be
// upgraded to version X?", the server runs its readiness checks and answers
// once with can_upgrade + reason.
//
// The gimbal's simulated health state is exposed as parameters so every path
// can be exercised without hardware:
//
//   ros2 param set /upgrade_check_server battery_percent 20.0
//   ros2 param set /upgrade_check_server gimbal_idle false
//
// Parameters:
//   battery_percent      current battery level   (default 80.0)
//   temperature_c        gimbal temperature      (default 45.0)
//   gimbal_idle          is the gimbal parked?   (default true)
//   min_battery_percent  check threshold         (default 30.0)
//   max_temperature_c    check threshold         (default 70.0)
//   upgrade_duration_sec estimated install time  (default 30)

#include <memory>
#include <sstream>
#include <string>

#include "gimbal_use_example_interfaces/srv/upgrade_check.hpp"
#include "rclcpp/rclcpp.hpp"

using UpgradeCheck = gimbal_use_example_interfaces::srv::UpgradeCheck;

class UpgradeCheckServer : public rclcpp::Node
{
public:
  UpgradeCheckServer()
  : Node("upgrade_check_server")
  {
    this->declare_parameter<double>("battery_percent", 80.0);
    this->declare_parameter<double>("temperature_c", 45.0);
    this->declare_parameter<bool>("gimbal_idle", true);
    this->declare_parameter<double>("min_battery_percent", 30.0);
    this->declare_parameter<double>("max_temperature_c", 70.0);
    this->declare_parameter<int>("upgrade_duration_sec", 30);

    service_ = this->create_service<UpgradeCheck>(
      "gimbal_upgrade_check",
      std::bind(
        &UpgradeCheckServer::handle_check, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Serving /gimbal_upgrade_check");
  }

private:
  void handle_check(
    const std::shared_ptr<UpgradeCheck::Request> request,
    std::shared_ptr<UpgradeCheck::Response> response)
  {
    // Read parameters on every call so `ros2 param set` takes effect live.
    const double battery = this->get_parameter("battery_percent").as_double();
    const double temperature = this->get_parameter("temperature_c").as_double();
    const bool idle = this->get_parameter("gimbal_idle").as_bool();
    const double min_battery = this->get_parameter("min_battery_percent").as_double();
    const double max_temperature = this->get_parameter("max_temperature_c").as_double();

    RCLCPP_INFO(
      this->get_logger(),
      "Check: %s -> %s (force=%s, battery=%.0f%%, temp=%.0f C, idle=%s)",
      request->current_version.c_str(), request->target_version.c_str(),
      request->force ? "true" : "false", battery, temperature, idle ? "true" : "false");

    response->can_upgrade = false;
    response->estimated_duration_sec = 0;

    if (request->target_version.empty()) {
      response->reason = "target_version is empty";
    } else if (request->current_version == request->target_version) {
      response->reason = "gimbal is already on " + request->target_version;
    } else if (!request->force && battery < min_battery) {
      std::ostringstream reason;
      reason << "battery too low: " << static_cast<int>(battery)
             << "% < " << static_cast<int>(min_battery) << "%";
      response->reason = reason.str();
    } else if (!request->force && temperature > max_temperature) {
      std::ostringstream reason;
      reason << "temperature too high: " << static_cast<int>(temperature)
             << " C > " << static_cast<int>(max_temperature) << " C";
      response->reason = reason.str();
    } else if (!request->force && !idle) {
      response->reason = "gimbal is moving; park it before upgrading";
    } else {
      response->can_upgrade = true;
      response->estimated_duration_sec =
        this->get_parameter("upgrade_duration_sec").as_int();
      response->reason = "checks passed, ready to upgrade to " + request->target_version;
    }

    RCLCPP_INFO(
      this->get_logger(), "Reply: %s (%s)",
      response->can_upgrade ? "CAN upgrade" : "CANNOT upgrade", response->reason.c_str());
  }

  rclcpp::Service<UpgradeCheck>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UpgradeCheckServer>());
  rclcpp::shutdown();
  return 0;
}
