#ifndef HARDWARE_HPP
#define HARDWARE_HPP
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "DiffDrivePi.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

namespace autonomous_rover
{
class AutonomousRoverHardware : public hardware_interface::SystemInterface
{
    public:

    RCLCPP_SHARED_PTR_DEFINITIONS(AutonomousRoverHardware);

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;
    
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
    
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
    // ROS2 Control state variables
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_commands_;

    double wheel_radius_ = 0.0325;

    // Encoder state tracking
    std::vector<int32_t> prev_encoder_counts_;
    double prev_left_cmd_ = 0.0;
    double prev_right_cmd_ = 0.0;

    // DiffDrivePi instance
    std::unique_ptr<DiffDrivePi> controller_;
    DiffDrivePi::RobotConfig robot_config_;

    // Publisher for pid performance logging
    rclcpp::Node::SharedPtr debug_node_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pid_debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_pulses_pub_;

};
} // namespace autonomous_rover

#endif // HARDWARE_HPP
