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


##########################################################################
####################### Executables ######################################

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
    DESTINATION lib/${PROJECT_NAME}
)
ament_package()
