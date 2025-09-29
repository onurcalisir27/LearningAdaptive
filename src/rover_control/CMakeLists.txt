cmake_minimum_required(VERSION 3.8)
project(rover_control)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_action)
find_package(rclcpp_components)
include_directories(include)

# Action Server Example
add_library(action_server SHARED
  src/fibonacci_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(action_server
  rover_msgs
  rclcpp
  rclcpp_action
  rclcpp_components)
rclcpp_components_register_node(action_server PLUGIN "rover_action::FibonacciActionServer" EXECUTABLE fibonacci_action_server)

install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Action Client Example
add_library(action_client SHARED
  src/fibonacci_action_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(action_client
  rover_msgs
  rclcpp
  rclcpp_action
  rclcpp_components)
rclcpp_components_register_node(action_client PLUGIN "rover_action::FibonacciActionClient" EXECUTABLE fibonacci_action_client)

install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Inverted Pendulum Control using Adaptive Controllers
add_executable(pendulum_control_node src/pendulum_control_node.cpp src/self_tuning_regulator.cpp)
ament_target_dependencies(pendulum_control_node
  rclcpp
  sensor_msgs
  rover_msgs)
target_include_directories(pendulum_control_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(pendulum_control_node Eigen3::Eigen)

# A* Planner Node
add_executable(planner_node src/planner_node.cpp src/astar.cpp)
ament_target_dependencies(planner_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)
target_include_directories(planner_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Include Controller Node
add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node
  rclcpp
  nav_msgs
  geometry_msgs
)
target_include_directories(control_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(control_node Eigen3::Eigen)

install(
  DIRECTORY description launch config worlds src rviz
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(TARGETS
    pendulum_control_node
    planner_node
    control_node
    DESTINATION lib/${PROJECT_NAME}
)
ament_package()
