// Gimbal manager for the auto-flight demo: answers gimbal identity and
// firmware questions over a service.
//
// The upgrade decision is part of the answer, not the caller's job: the
// gimbal compares its installed firmware with the version the aircraft
// requires and reports upgrade_required + reason.
//
// Parameters (all live-editable with `ros2 param set`):
//   model             gimbal model           (default "Z6")
//   serial            serial number          (default "Z6-0001")
//   firmware_version  installed firmware     (default "1.2.0")
//   required_version  version the aircraft   (default "1.1.0")
//                     expects

#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>

#include "auto_flight_interfaces/srv/get_gimbal_info.hpp"
#include "rclcpp/rclcpp.hpp"

using GetGimbalInfo = auto_flight_interfaces::srv::GetGimbalInfo;

namespace
{
// Compare dotted numeric versions. Returns <0, 0, >0 like strcmp.
int compare_versions(const std::string & a, const std::string & b)
{
  std::istringstream sa(a);
  std::istringstream sb(b);
  while (true) {
    std::string pa;
    std::string pb;
    const bool has_a = static_cast<bool>(std::getline(sa, pa, '.'));
    const bool has_b = static_cast<bool>(std::getline(sb, pb, '.'));
    if (!has_a && !has_b) {
      return 0;
    }
    const long va = has_a ? std::atol(pa.c_str()) : 0;
    const long vb = has_b ? std::atol(pb.c_str()) : 0;
    if (va != vb) {
      return va < vb ? -1 : 1;
    }
  }
}
}  // namespace

class GimbalManager : public rclcpp::Node
{
public:
  GimbalManager()
  : Node("gimbal_manager")
  {
    this->declare_parameter<std::string>("model", "Z6");
    this->declare_parameter<std::string>("serial", "Z6-0001");
    this->declare_parameter<std::string>("firmware_version", "1.2.0");
    this->declare_parameter<std::string>("required_version", "1.1.0");

    service_ = this->create_service<GetGimbalInfo>(
      "gimbal/get_info",
      std::bind(
        &GimbalManager::handle_info, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      this->get_logger(),
      "Serving /gimbal/get_info (firmware %s, required %s)",
      this->get_parameter("firmware_version").as_string().c_str(),
      this->get_parameter("required_version").as_string().c_str());
  }

private:
  void handle_info(
    const std::shared_ptr<GetGimbalInfo::Request> request,
    std::shared_ptr<GetGimbalInfo::Response> response)
  {
    (void)request;

    // Read parameters per call so `ros2 param set` changes the answer live.
    const std::string firmware = this->get_parameter("firmware_version").as_string();
    const std::string required = this->get_parameter("required_version").as_string();

    response->model = this->get_parameter("model").as_string();
    response->serial = this->get_parameter("serial").as_string();
    response->firmware_version = firmware;
    response->required_version = required;
    response->upgrade_required = compare_versions(firmware, required) < 0;

    if (response->upgrade_required) {
      response->reason = "firmware " + firmware + " is older than required " + required;
    } else {
      response->reason = "firmware " + firmware + " is up to date";
    }

    RCLCPP_INFO(
      this->get_logger(), "Info request: %s %s, firmware %s, upgrade_required=%s (%s)",
      response->model.c_str(), response->serial.c_str(), firmware.c_str(),
      response->upgrade_required ? "true" : "false", response->reason.c_str());
  }

  rclcpp::Service<GetGimbalInfo>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalManager>());
  rclcpp::shutdown();
  return 0;
}
