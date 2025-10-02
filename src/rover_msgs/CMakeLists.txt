cmake_minimum_required(VERSION 3.8)
project(rover_msgs)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Params.msg"
  "msg/Trajectory.msg"
  "action/Fibonacci.action"
  "action/PendulumTrajectory.action"
  DEPENDENCIES std_msgs unique_identifier_msgs action_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
