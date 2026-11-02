cmake_minimum_required(VERSION 3.8)
project(auto_flight_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(auto_flight_interfaces REQUIRED)

set(demo_nodes
  drone_sim
  gimbal_manager
  auto_flight_node
  auto_flight_client
)

foreach(node ${demo_nodes})
  add_executable(${node} src/${node}.cpp)
  ament_target_dependencies(${node}
    rclcpp
    rclcpp_action
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    std_srvs
    auto_flight_interfaces
  )
endforeach()

install(TARGETS ${demo_nodes}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
