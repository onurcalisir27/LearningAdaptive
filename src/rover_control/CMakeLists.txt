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
add_library(action_server SHARED
  src/pendulum_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(action_server
  rover_msgs
  rover_utils
  rclcpp
  sensor_msgs
  std_msgs
  rclcpp_action
  rclcpp_components)
rclcpp_components_register_node(action_server PLUGIN "pendulum_action::PendulumActionServer" EXECUTABLE pendulum_action_server)
target_link_libraries(action_server Eigen3::Eigen)

# Pendulum Action Client
add_library(action_client SHARED
  src/pendulum_action_client.cpp)

target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(action_client
  rover_msgs
  rclcpp
  rclcpp_action
  rclcpp_components)
rclcpp_components_register_node(action_client PLUGIN "pendulum_action::PendulumActionClient" EXECUTABLE pendulum_action_client)

# Pendulum Action Server controller
add_library(controller SHARED
  src/pendulum_control_node.cpp)
target_include_directories(controller PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(controller
  rover_msgs
  rover_utils
  rclcpp
  sensor_msgs
  std_msgs
  rclcpp_action
  rclcpp_components)
rclcpp_components_register_node(controller PLUGIN "pendulum_action::PendulumControlNode" EXECUTABLE pendulum_control_node)
target_link_libraries(controller Eigen3::Eigen)

install(TARGETS
  action_server
  action_client
  controller
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

# Inverted Pendulum Control using Adaptive Controllers
# add_executable(pendulum_control_node src/pendulum_control_node.cpp src/self_tuning_regulator.cpp)
# ament_target_dependencies(pendulum_control_node
#   rclcpp
#   sensor_msgs
#   rclcpp_components
#   rover_msgs)
# target_include_directories(pendulum_control_node
#   PRIVATE
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
# )
# target_link_libraries(pendulum_control_node Eigen3::Eigen)

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

# A* Planner Node
add_executable(planner_node src/planner_node.cpp)
ament_target_dependencies(planner_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  rover_utils
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
    planner_node
    control_node
    two_link_control_node
    pendulum_service_handle
    DESTINATION lib/${PROJECT_NAME}
)
ament_package()
