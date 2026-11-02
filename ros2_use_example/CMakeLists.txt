cmake_minimum_required(VERSION 3.8)
project(ros2_use_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_use_example_interfaces REQUIRED)

set(demo_nodes
  topic_publisher
  topic_subscriber
  service_server
  service_client
  action_server
  action_client
)

foreach(node ${demo_nodes})
  add_executable(${node} src/${node}.cpp)
  ament_target_dependencies(${node}
    rclcpp
    rclcpp_action
    std_msgs
    ros2_use_example_interfaces
  )
endforeach()

install(TARGETS ${demo_nodes}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
