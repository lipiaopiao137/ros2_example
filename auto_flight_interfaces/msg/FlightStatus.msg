# High-level flight status broadcast for operators and logs.
std_msgs/Header header
# IDLE / PREFLIGHT / TAKEOFF / CRUISE / LAND
string state
string gimbal_firmware_version
bool gimbal_upgrade_required
# True when pose, imu and odom are all fresh.
bool sensors_ok
uint32 current_waypoint
uint32 total_waypoints
