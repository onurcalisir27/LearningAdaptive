cmake_minimum_required(VERSION 3.8)
project(pendulum_control)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(rover_utils REQUIRED)
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

##########################################################################
####################### Libraries ########################################
# Pendulum Action Server
add_library(pendulum_server SHARED
  src/pendulum_action_server.cpp)
target_include_directories(pendulum_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(pendulum_server
  rover_msgs
  rover_utils
  rclcpp
  sensor_msgs
  std_msgs
  rclcpp_action
  rclcpp_components)
rclcpp_components_register_node(pendulum_server PLUGIN "pendulum_action::PendulumActionServer" EXECUTABLE pendulum_action_server)
target_link_libraries(pendulum_server Eigen3::Eigen)

# Pendulum Action Client
add_library(pendulum_client SHARED
  src/pendulum_action_client.cpp)

target_include_directories(pendulum_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(pendulum_client
  rover_msgs
  rclcpp
  rclcpp_action
  rclcpp_components)
rclcpp_components_register_node(pendulum_client PLUGIN "pendulum_action::PendulumActionClient" EXECUTABLE pendulum_action_client)

# Pendulum Action Server controller
add_library(pendulum_controller SHARED
  src/pendulum_control_node.cpp)
target_include_directories(pendulum_controller PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(pendulum_controller
  rover_msgs
  rover_utils
  rclcpp
  sensor_msgs
  std_msgs
  rclcpp_components)
rclcpp_components_register_node(pendulum_controller PLUGIN "pendulum_action::PendulumControlNode" EXECUTABLE pendulum_control_node)
target_link_libraries(pendulum_controller Eigen3::Eigen)

install(TARGETS
  pendulum_server
  pendulum_client
  pendulum_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

##########################################################################
####################### Executables ######################################
# Pendulum Service Handler Node
add_executable(pendulum_service_handle src/pendulum_service_handle.cpp)
ament_target_dependencies(pendulum_service_handle
  rclcpp
  std_srvs
  ros_gz_interfaces
  geometry_msgs
  sensor_msgs
  )

# Inverted Pendulum Control for 2 Links using Adaptive Controllers
add_executable(two_link_control_node src/two_link_control_node.cpp)
ament_target_dependencies(two_link_control_node
  rclcpp
  rover_utils
  sensor_msgs
  rclcpp_components
  rover_msgs)
target_include_directories(two_link_control_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(
  DIRECTORY description launch config worlds src
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(TARGETS
    two_link_control_node
    pendulum_service_handle
    DESTINATION lib/${PROJECT_NAME}
)
ament_package()
