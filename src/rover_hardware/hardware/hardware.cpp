#include "rover_hardware/hardware.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// String parser helper function
std::vector<std::string> param_parser(const std::string& str, char delimiter){

    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, delimiter)){
        tokens.push_back(token);
    }
    return tokens;
}

namespace autonomous_rover
{
hardware_interface::CallbackReturn AutonomousRoverHardware::on_init(const hardware_interface::HardwareInfo & info)
{
    // Return ERROR if cant access the info
    if(
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize hardware data
    hw_positions_.resize(info_.joints.size(), 0.0);
    hw_velocities_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);

    prev_encoder_counts_.resize(info_.joints.size(), 0);

    // Parse motor parameters
    auto left_motor_param = param_parser(info_.hardware_parameters.at("left_motor"), ',');        
    auto right_motor_param = param_parser(info_.hardware_parameters.at("right_motor"), ',');

    if (left_motor_param.size() != 4 || right_motor_param.size() != 4 ){
        RCLCPP_FATAL(rclcpp::get_logger("AutonomousRoverHardware"), "Motor parameters must have 4 values");
        return hardware_interface::CallbackReturn::ERROR;
    }

    auto left_front_encoder = param_parser(info_.hardware_parameters.at("left_front_encoder"), ',');
    auto left_back_encoder = param_parser(info_.hardware_parameters.at("left_back_encoder"), ',');
    auto right_front_encoder = param_parser(info_.hardware_parameters.at("right_front_encoder"), ',');
    auto right_back_encoder = param_parser(info_.hardware_parameters.at("right_back_encoder"), ',');

    if (left_front_encoder.size() != 4 || left_back_encoder.size() != 4 || right_front_encoder.size() != 4 || right_back_encoder.size() != 4){
        RCLCPP_FATAL(rclcpp::get_logger("AutonomousRoverHardware"), "Encoder parameters must have 4 values");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Parse controller parameters
    auto left_controller_param = param_parser(info_.hardware_parameters.at("left_controller"), ',');        
    auto right_controller_param = param_parser(info_.hardware_parameters.at("right_controller"), ',');

    if (left_controller_param.size() != 5 || right_controller_param.size() != 5 ){
        RCLCPP_FATAL(rclcpp::get_logger("AutonomousRoverHardware"), "Controller parameters must have 5 values");
        return hardware_interface::CallbackReturn::ERROR;
    }

    double min_velocity = std::stod(info_.hardware_parameters.at("min_velocity"));
    double max_velocity = std::stod(info_.hardware_parameters.at("max_velocity"));
    
    // Left Motor Config
    robot_config_.motors[0] = {
        static_cast<uint8_t>(std::stoi(left_motor_param[0])),       // forward pin
        static_cast<uint8_t>(std::stoi(left_motor_param[1])),       // reverse pin
        static_cast<uint8_t>(std::stoi(left_motor_param[2])),       // enable pin
        static_cast<uint16_t>(std::stoi(left_motor_param[3]))       // pwm freq
    };
    // Right Motor Config
    robot_config_.motors[1] = {
        static_cast<uint8_t>(std::stoi(right_motor_param[0])),       // forward pin
        static_cast<uint8_t>(std::stoi(right_motor_param[1])),       // reverse pin
        static_cast<uint8_t>(std::stoi(right_motor_param[2])),       // enable pin
        static_cast<uint16_t>(std::stoi(right_motor_param[3]))       // pwm freq
    };

    // Left Front Encoder
    robot_config_.encoders[0]= {
        static_cast<uint8_t>(std::stoi(left_front_encoder[0])),     // pin A
        static_cast<uint8_t>(std::stoi(left_front_encoder[1])),     // pin B
        std::stoi(left_front_encoder[2]),     // CPR    
        static_cast<double>(std::stod(left_front_encoder[3]))       // Wheel Circumference 
    };
    // Left Back Encoder
    robot_config_.encoders[1]= {
        static_cast<uint8_t>(std::stoi(left_back_encoder[0])),     // pin A
        static_cast<uint8_t>(std::stoi(left_back_encoder[1])),     // pin B
        std::stoi(left_back_encoder[2]),     // CPR    
        static_cast<double>(std::stod(left_back_encoder[3]))       // Wheel Circumference 
    };
    // Right Front Encoder
    robot_config_.encoders[2]= {
        static_cast<uint8_t>(std::stoi(right_front_encoder[0])),     // pin A
        static_cast<uint8_t>(std::stoi(right_front_encoder[1])),     // pin B
        std::stoi(right_front_encoder[2]),     // CPR    
        static_cast<double>(std::stod(right_front_encoder[3]))       // Wheel Circumference 
    };
    // Right Back Encoder
    robot_config_.encoders[3]= {
        static_cast<uint8_t>(std::stoi(right_back_encoder[0])),     // pin A
        static_cast<uint8_t>(std::stoi(right_back_encoder[1])),     // pin B
        std::stoi(right_back_encoder[2]),     // CPR    
        static_cast<double>(std::stod(right_back_encoder[3]))       // Wheel Circumference 
    };

    robot_config_.controllers[0] = {
        static_cast<bool>(std::stoi(left_controller_param[0])),     // enable
        static_cast<double>(std::stod(left_controller_param[1])),   // kp
        static_cast<double>(std::stod(left_controller_param[2])),   // ki
        static_cast<double>(std::stod(left_controller_param[3])),   // kd
        static_cast<double>(std::stod(left_controller_param[4]))    // integral limits
    };

    robot_config_.controllers[1] = {
        static_cast<bool>(std::stoi(right_controller_param[0])),     // enable
        static_cast<double>(std::stod(right_controller_param[1])),   // kp
        static_cast<double>(std::stod(right_controller_param[2])),   // ki
        static_cast<double>(std::stod(right_controller_param[3])),   // kd
        static_cast<double>(std::stod(right_controller_param[4]))    // integral limits
    };

    robot_config_.min_velocity = static_cast<float>(min_velocity);
    robot_config_.max_velocity = static_cast<float>(max_velocity);

    RCLCPP_INFO(rclcpp::get_logger("AutonomousRoverHardware"), "Successfully initialized!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AutonomousRoverHardware::on_configure(const rclcpp_lifecycle::State & previous_state) {
    
    RCLCPP_INFO(rclcpp::get_logger("AutonomousRoverHardware"), "Configuring hardware..");
    try {
        controller_ = std::make_unique<DiffDrivePi>();

        debug_node_ = std::make_shared<rclcpp::Node>("pid_debug_node");
        pid_debug_pub_ = debug_node_->create_publisher<std_msgs::msg::Float64MultiArray>("pid_debug", 50);
        encoder_pulses_pub_ = debug_node_->create_publisher<std_msgs::msg::Int32MultiArray>("encoder_pulses", 50);

        // Run Robot initialization
        if(!controller_->initialize(robot_config_)){
            RCLCPP_FATAL(rclcpp::get_logger("AutonomousRoverHardware"), "Failed to initialize hardware..");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("AutonomousRoverHardware"), "Hardware configuration succesful..");
        return hardware_interface::CallbackReturn::SUCCESS;

    } catch(std::exception& e){
            RCLCPP_FATAL(rclcpp::get_logger("AutonomousRoverHardware"), "Exception during configuration %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
    }
}

std::vector<hardware_interface::StateInterface> AutonomousRoverHardware::export_state_interfaces(){
    
    std::vector<hardware_interface::StateInterface> state_interfaces;
    // Create state interface for each joint
    for(size_t i=0; i < info_.joints.size(); ++i){
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }

    RCLCPP_INFO(rclcpp::get_logger("AutonomousRoverHardware"), "Exported %zu state interfaces", state_interfaces.size());
    return state_interfaces;
    
}

std::vector<hardware_interface::CommandInterface> AutonomousRoverHardware::export_command_interfaces(){
   
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    // Create command interface for each joint
    for(size_t i=0; i < info_.joints.size(); ++i){
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));

    }

    RCLCPP_INFO(rclcpp::get_logger("AutonomousRoverHardware"), "Exported %zu command interfaces", command_interfaces.size());
    return command_interfaces;
}

hardware_interface::CallbackReturn AutonomousRoverHardware::on_activate(const rclcpp_lifecycle::State & previous_state){

    RCLCPP_INFO(rclcpp::get_logger("AutonomousRoverHardware"), "Activating..");

    // Use DiffDrivePi recover() to reinitialize if needed
    if(!controller_ || !controller_->recover()){
        RCLCPP_FATAL(rclcpp::get_logger("AutonomousRoverHardware"),"Error activating controller.." );
        return hardware_interface::CallbackReturn::ERROR;
    }

    std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    std::fill(prev_encoder_counts_.begin(), prev_encoder_counts_.end(), 0);

    RCLCPP_INFO(rclcpp::get_logger("AutonomousRoverHardware"), "Hardware Activated succesfully..");
    return hardware_interface::CallbackReturn::SUCCESS;
}
    
hardware_interface::CallbackReturn AutonomousRoverHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(rclcpp::get_logger("AutonomousRoverHardware"), "Deactivating..");

    // Call DiffDrivePi shutdown method for safe controller deactivation
    if(controller_){
        controller_->shutdown();
    }
    RCLCPP_INFO(rclcpp::get_logger("AutonomousRoverHardware"), "Hardware Deactivated..");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AutonomousRoverHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period){

    if(!controller_){
        return hardware_interface::return_type::ERROR;
    }
    // Read from the encoders
    std::chrono::duration<double> dt_duration(period.seconds());

    // Left Front Encoder
    hw_positions_[0] = controller_->getDistanceRad(DiffDrivePi::Encoder::LEFT_FRONT);
    hw_velocities_[0] = controller_->getVelocityRad(DiffDrivePi::Encoder::LEFT_FRONT, prev_encoder_counts_[0], dt_duration);
    prev_encoder_counts_[0] = controller_->getEncoderCounts(DiffDrivePi::Encoder::LEFT_FRONT);

    // Left Back Encoder
    hw_positions_[1] = controller_->getDistanceRad(DiffDrivePi::Encoder::LEFT_BACK);
    hw_velocities_[1] = controller_->getVelocityRad(DiffDrivePi::Encoder::LEFT_BACK, prev_encoder_counts_[1], dt_duration);
    prev_encoder_counts_[1] = controller_->getEncoderCounts(DiffDrivePi::Encoder::LEFT_BACK);

    // Right Front Encoder
    hw_positions_[2] = controller_->getDistanceRad(DiffDrivePi::Encoder::RIGHT_FRONT);
    hw_velocities_[2] = controller_->getVelocityRad(DiffDrivePi::Encoder::RIGHT_FRONT, prev_encoder_counts_[2], dt_duration);
    prev_encoder_counts_[2] = controller_->getEncoderCounts(DiffDrivePi::Encoder::RIGHT_FRONT);

     // Right Back Encoder
    hw_positions_[3] = controller_->getDistanceRad(DiffDrivePi::Encoder::RIGHT_BACK);
    hw_velocities_[3] = controller_->getVelocityRad(DiffDrivePi::Encoder::RIGHT_BACK, prev_encoder_counts_[3], dt_duration);
    prev_encoder_counts_[3] = controller_->getEncoderCounts(DiffDrivePi::Encoder::RIGHT_BACK);

    int32_t myvar = prev_encoder_counts_[0];
    std_msgs::msg::Int32MultiArray encoder_msg;
    encoder_msg.data = {prev_encoder_counts_[0], prev_encoder_counts_[1], prev_encoder_counts_[2], prev_encoder_counts_[3]};
    encoder_pulses_pub_->publish(encoder_msg);

    return hardware_interface::return_type::OK;
}
    
hardware_interface::return_type AutonomousRoverHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period){

    if(!controller_){
        return hardware_interface::return_type::ERROR;
    }
    
    std::chrono::duration<double> dt_duration(period.seconds());

    // Apply PID Controller Correction to velocities
    double left_cmd = (hw_commands_[0] + hw_commands_[1]) / 2.0 * wheel_radius_;
    double right_cmd = (hw_commands_[2] + hw_commands_[3]) / 2.0 * wheel_radius_;

    double left_measured = (hw_velocities_[0] + hw_velocities_[1]) / 2.0 * wheel_radius_;
    double right_measured = (hw_velocities_[2] + hw_velocities_[3]) / 2.0 * wheel_radius_;

    if (std::abs(left_cmd) < 0.01 && std::abs(right_cmd) < 0.01) {
        if(!controller_->DiffDriveControl(0.0, 0.0)){
            RCLCPP_FATAL(rclcpp::get_logger("AutonomousRoverHardware"), "Failed to send motor commands");
            return hardware_interface::return_type::ERROR;
        }
    } else {
        if (std::abs(left_cmd - prev_left_cmd_) > 0.05 || std::abs(right_cmd - prev_right_cmd_) > 0.05) {
            controller_->resetControllers();
        }
        double left_corrected = controller_->PIDController(DiffDrivePi::Controller::LEFT, left_cmd, left_measured, dt_duration);
        double right_corrected = controller_->PIDController(DiffDrivePi::Controller::RIGHT, right_cmd, right_measured, dt_duration);
        
        if(!controller_->DiffDriveControl(left_corrected, right_corrected)){
            RCLCPP_FATAL(rclcpp::get_logger("AutonomousRoverHardware"), "Failed to send motor commands");
            return hardware_interface::return_type::ERROR;
        }
    }
    // Publish debug message
    std_msgs::msg::Float64MultiArray debug_msg;
    debug_msg.data = {left_cmd, left_measured,
                      right_cmd, right_measured};
    pid_debug_pub_->publish(debug_msg);
       
    prev_left_cmd_ = left_cmd;
    prev_right_cmd_ = right_cmd;

    return hardware_interface::return_type::OK;
}

}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(autonomous_rover::AutonomousRoverHardware, hardware_interface::SystemInterface)