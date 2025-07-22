cmake_minimum_required(VERSION 3.8)
project(autonomous_rover)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(depthai REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(depthai_ros_msgs REQUIRED)

# Find pigpio library
set(PIGPIO_LIBRARY "/usr/local/lib/libpigpiod_if2.so")
if(NOT EXISTS ${PIGPIO_LIBRARY})
  message(FATAL_ERROR "pigpio library not found at: ${PIGPIO_LIBRARY}")
else()
  message(STATUS "Found pigpio library: ${PIGPIO_LIBRARY}")
endif()

add_library(
  autonomous_rover
  SHARED
  hardware/DiffDrivePi.cpp
  hardware/hardware.cpp
)

target_compile_features(autonomous_rover PUBLIC cxx_std_17)
target_include_directories(autonomous_rover PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/local/include
)

ament_target_dependencies(
  autonomous_rover PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(autonomous_rover PUBLIC ${PIGPIO_LIBRARY})

pluginlib_export_plugin_description_file(hardware_interface diffdrivepi_plugin.xml)

add_executable(encoder_calibration_node 
  src/encoder_calibration_node.cpp
)

ament_target_dependencies(encoder_calibration_node 
  rclcpp 
  sensor_msgs
)

add_executable(feedback_node 
  src/feedback_node.cpp
)
ament_target_dependencies(feedback_node 
  rclcpp 
  nav_msgs
  sensor_msgs
)
target_link_libraries(feedback_node Eigen3::Eigen)

add_executable(imu_filter_node src/imu_filter_node.cpp)

ament_target_dependencies(imu_filter_node 
  rclcpp 
  sensor_msgs
)

add_executable(camera_driver_node hardware/camera_driver_node.cpp)
ament_target_dependencies(camera_driver_node
  rclcpp
  sensor_msgs
  camera_info_manager
  depthai
  depthai_bridge
  depthai_ros_msgs
)
target_link_libraries(camera_driver_node
  depthai::core
)

install(TARGETS autonomous_rover
  DESTINATION lib
)

install(TARGETS 
  autonomous_rover
  encoder_calibration_node
  feedback_node
  camera_driver_node
  imu_filter_node
  
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
  OPTIONAL
)

install(FILES diffdrivepi_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
