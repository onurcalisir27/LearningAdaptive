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
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(pendulum_control_node src/pendulum_control_node.cpp src/self_tuning_regulator.cpp)

ament_target_dependencies(pendulum_control_node
  rclcpp
  sensor_msgs)

target_include_directories(pendulum_control_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(pendulum_control_node Eigen3::Eigen)

add_executable(pendulum_node src/pendulum_node.cpp )

ament_target_dependencies(pendulum_node
  rclcpp
  sensor_msgs)

target_include_directories(pendulum_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(pendulum_node Eigen3::Eigen)


install(
  DIRECTORY description launch config worlds
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(TARGETS
    pendulum_control_node
    pendulum_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
