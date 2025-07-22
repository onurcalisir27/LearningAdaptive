#include "autonomous_rover/DiffDrivePi.hpp"
#include <cmath>
#include <algorithm>

/* DiffDrivePi Differential Drive Robot Control using a Raspberry Pi 4*/

// if DiffDrivePi is used, it defaults to allowing verbose debugging 
DiffDrivePi::DiffDrivePi() 
	:   config_(getDefaultConfig()),
	status_(SysStatus::SLEEP),
        pi_(0),
        pigpio_status_(false),
        verbose_debug_(false),
        reset_counter_(0)
{
    // Initialize all motors with default values
    for (size_t i=0; i < static_cast<size_t>(Motor::TOTAL); i++){
        motor_status_[i].forward = true;
        motor_status_[i].enabled = false;
        motor_status_[i].last_cmd_vel = 0.0f;
        motor_status_[i].cmd_pwm = 0;
    }
    for (size_t i=0; i < static_cast<size_t>(Encoder::TOTAL); i++){
        encoders_state_[i].count.store(0);
        encoders_state_[i].filtered.store(0);
        
        encoders_state_[i].last_state = 0;
        encoders_state_[i].init = false;
        encoders_state_[i].callback_a_id = -1;
        encoders_state_[i].callback_b_id = -1;
    }
    
    for (size_t i=0; i < static_cast<size_t>(Controller::TOTAL); i++){
        controllers_state_[i].prev_error = 0;
        controllers_state_[i].integral = 0;
    }

    if(verbose_debug_)
        std::cout << "DiffDrivePi: System in SLEEP mode" << std::endl;

}
// Alternatively, the verbosity can be adjusted in the constructor
DiffDrivePi::DiffDrivePi(bool allow_debug)
    :   DiffDrivePi()
{
    verbose_debug_ = allow_debug;
}

DiffDrivePi::~DiffDrivePi(){
    // Make sure to explicitly call shutdown before 
    shutdown();
}

DiffDrivePi::RobotConfig DiffDrivePi::getDefaultConfig(){

    DiffDrivePi::RobotConfig default_config;
    
    /* Read below to get an idea on how to set your config for your robot's requirements

    Raspberry Pi 4 and 5 have 4 PWM enabled pins: GPIO PWM0(12, 18) , PWM1(13, 19)
    With Differential Drive Cars, a neat trick can be used to make use of hardware abstraction. We do not explicitly need to tell the
    contoller we are controlling 4 motors, if we just plug the same side motors into the same terminals of our h-bridge, both motors will
    see the same commands to execute, thus we can reduce our control logic to RIGHT and LEFT motors only, and allowing our smart wiring to
    handle the front and rear motors

    Motor configuration follows the following format: {Forward Direction Pin, Reverse Direction Pin, Enable Pin, PWM Frequency}
    
    Choose your minimum velocity required to overcome static friction effects
    Choose your maximum velocity you want your robot to achieve, this will correspond to maximum PWM

    By default verbose debugging is ON, if you are comfortable with your setup, you can turn it off by setting the debug_override key to FALSE

    Please do not touch the getDefaultConfig function. You can pass the controller your own configuration using 'initialize(const RobotConfig& config)
    'initialize' is the only correct method to pass your config to the hardware interface class, this function is here to serve as a reference.
    Again, initialize will override the following config, make sure you can create your own config before you use the motor control commands.
    */

    // Example motor pin configuration using Raspberry pi hardware pins and nearby available GPIO pins
    
    // Direction Pin Forward, Pin Reverse, Power Pin Enable, PWM Frequency
    default_config.motors[0] = {16, 20, 12, 1000};     // Left Motor
    default_config.motors[1] = {5, 6, 13, 1000};       // Right Motor

    // Pin A, Pin B, Counts per Revolution, Wheel Diameter
    default_config.encoders[0] = {23, 24, 1920, 0.205};     // Left Front Encoder
    default_config.encoders[1] = {25, 8, 1920, 0.205};      // Left Back Encoder
    default_config.encoders[2] = {17, 4, 1920, 0.205};      // Right Front Encoder
    default_config.encoders[3] = {22, 27, 1920, 0.205};     // Right Back Encoder

    // If encoder is reading (-) values in forward direction, switch pinA and pinB assignments
    // Can switch in your config or in real life... 

    // Enable Controller, kp, ki, kd, limits-->
    default_config.controllers[0] = {0, 0.0, 0.0, 0.0, 0.0};    // Left Controller
    default_config.controllers[1] = {0, 0.0, 0.0, 0.0, 0.0};    // Right Controller

    default_config.min_velocity = 0.1f;
    default_config.max_velocity = 1.0f; // max velocity will map to max PWM

    return default_config;
}

bool DiffDrivePi::initialize(const RobotConfig& config){

    // Store the user defined Robot config as the current configuration and as the default in case we need to reset
    config_ = config;

    if(verbose_debug_)
        std::cout << "Starting Robot initialization sequence" << std::endl;

    // Try to initialize pigpio library support
    if (!initpigpio()){
        if(verbose_debug_)
            std::cerr << "DiffDrivePi: Something went wrong trying to initialize pigpio" << std::endl;
        status_ = SysStatus::ERROR;
        return false;
    }

    // Start all pins at LOW and 0 PWM
    if (!Stop()){
        if(verbose_debug_)
            std::cerr << "DiffDrivePi: Something went wrong trying to stop, check wiring" << std::endl;
        status_ = SysStatus::ERROR;
        return false;
    }

    if(!initializeEncoders()){
        if(verbose_debug_){
            std::cerr << "DiffDrivePi: Encoder configuration failed!" << std::endl;
        }
        status_ = SysStatus::ERROR;
        return false;
    }

    // Make sure controllers start at with 0 accumulated parameters
    resetControllers();

    // If no errors, get the robot ready for velocity commands
    status_ = SysStatus::READY;

    if(verbose_debug_){
        std::cout << "DiffDrivePi: Robot interface initialized with no errors" << std::endl;
        std::cout << "DiffDrivePi: Configured " << static_cast<int>(Motor::TOTAL) << " motors" << std::endl;
    }

    return true;
}

void DiffDrivePi::shutdown(){
    if(verbose_debug_)
        std::cout << "DiffDrivePi: Starting Robot shutdown sequence" << std::endl;

    if (pigpio_status_){
        // Make sure all motors are shut down
        if(!Stop()){
            if(verbose_debug_)
                std::cout << "DiffDrivePi: Error attempting Stop" << std::endl;
            status_ = SysStatus::ERROR;
        }

        // Also shutdown all the encoders
        shutdownEncoders();

        for (size_t i = 0; i < static_cast<size_t>(Motor::TOTAL); i++){
            // Create a const reference to the motor pins defined in Robot Config
            const MotorConfig& motor_config = config_.motors[i];

            // Release Motor Pins
            set_mode(pi_, motor_config.pinForward, PI_INPUT);
            set_mode(pi_, motor_config.pinReverse, PI_INPUT);

            set_mode(pi_, motor_config.pinEnable, PI_INPUT);
        }

        if(verbose_debug_){
            std::cout << "DiffDrivePi: All pins are set to LOW and released" << std::endl;
        }

        // Terminate connection to pigpio daemon
        pigpio_stop(pi_);
    }

    // Update System status
    status_ = SysStatus::SLEEP;
    pigpio_status_ = false;
    verbose_debug_ = true;

    if(verbose_debug_)
        std::cout << "DiffDrivePi: Shutdown completed" << std::endl;
}

bool DiffDrivePi::reset(){
    if (verbose_debug_) {std::cout << "DiffDrivePi: Entering reset sequence!" << std::endl;}
    // Shutdown current controller, and restart
    shutdown();
    // Add reset counter to recommend a new configuration if system cant recover
    reset_counter_++;

    return initialize(config_);
}

bool DiffDrivePi::recover() {
    if (verbose_debug_) {
        std::cout << "DiffDrivePi: Recovery attempt from state: " 
                  << static_cast<int>(status_) << std::endl;
    }
    
    switch(status_) {
        case SysStatus::STOP:
            // If no issues with wire connection, keep going
            if (pigpio_status_) {
                status_ = SysStatus::READY;
                return true;
            }
            break;
            
        case SysStatus::ERROR:
            // Try to reinitialize
            if (!pigpio_status_) {
                if (reset_counter_ < 3) {
                    return reset();
                } else {
                    std::cerr << "DiffDrivePi: Too many reset attempts" << std::endl;
                    return false;
                }
            }
            break;
            
        case SysStatus::READY:
        case SysStatus::RUNNING:
            // Already good
            return true;
            
        case SysStatus::SLEEP:
            // Need full initialization
            return initialize(config_);
    }
    
    return false;
}

bool DiffDrivePi::setMotorVel(Motor motor, double velocity){

    // Return false if commanded motor is invalid
    if (!isValidMotor(motor)) {return false;}

    if (!pigpio_status_){
        if (verbose_debug_) {std::cerr << "DiffDrivePi: pigpio is not initialized." << std::endl;}
        return false;       
    }

    // Poll for information before starting to set up
    MotorStatus& motor_status = getMotorStatus(motor);
    const MotorConfig& motor_config = getMotorConfig(motor);

    uint32_t pwm = velocityToPWM(velocity);

    // If the velocity is positive, set the motor in forward motion
    if (velocity > 0) {

        motor_status.forward = true;
        gpio_write(pi_, motor_config.pinReverse, 0);
        gpio_write(pi_, motor_config.pinForward, 1);
        motor_status.enabled = true;
        hardware_PWM(pi_, motor_config.pinEnable, motor_config.pwm_freq, pwm);

        motor_status.cmd_pwm = pwm;

    } else if (velocity < 0) {
    
        // Else, the velocity is negative, motor in reverse motion
        motor_status.forward = false;
        gpio_write(pi_, motor_config.pinForward, 0);
        gpio_write(pi_, motor_config.pinReverse, 1);

        motor_status.enabled = true;
        hardware_PWM(pi_, motor_config.pinEnable, motor_config.pwm_freq, pwm);

        motor_status.cmd_pwm = pwm;

    } else { // velocity = 0
        motor_status.forward = true;
        gpio_write(pi_, motor_config.pinForward, 0);
        gpio_write(pi_, motor_config.pinReverse, 0);

        motor_status.enabled = false;
        hardware_PWM(pi_, motor_config.pinEnable, motor_config.pwm_freq, pwm);

        motor_status.cmd_pwm = 0;
    }
    
    motor_status.last_cmd_vel = velocity;

    if (verbose_debug_) {
        std::cerr << "DiffDrivePi: Velocity Command being executed!" << std::endl;
        std::cout << "Motor " << static_cast<int>(motor) 
            << " - Velocity: " << velocity 
            << " - PWM: " << pwm 
            << " - Direction: " << (velocity > 0 ? "Forward" : "Reverse") 
            << std::endl;
    }

    return true;
}

bool DiffDrivePi::DiffDriveControl(double left_vel, double right_vel){

    // If system is not ready, return false
    if (!isReady()){
        if (verbose_debug_) {std::cerr << "DiffDrivePi: System not ready for velocity commands" << std::endl;}
        if(!recover()){
            return false;
        }
    }
    // Set the motor velocities
    bool running = true;
    running &= setMotorVel(Motor::LEFT, left_vel);
    running &= setMotorVel(Motor::RIGHT, right_vel);

    if (running && (right_vel != 0 || left_vel != 0)){
        status_ = SysStatus::RUNNING;
    } else if (running && right_vel == 0 && left_vel == 0 ){
        status_ = SysStatus::READY;
    }

    return running;
}

bool DiffDrivePi::Stop(){

    if(verbose_debug_)
        std::cout << "DiffDrivePi: Stop Called!" << std::endl;

    if(!pigpio_status_){
        if(verbose_debug_) {std::cerr << "DiffDrivePi: Cannot stop, pigpio not working!" << std::endl;}
        return false;
    }

    // Turn off all motors
    for (size_t i = 0; i < static_cast<size_t>(Motor::TOTAL); i++){
        // Create a const reference to the motor pins defined in Robot Config
        const MotorConfig& motor_config = config_.motors[i];

        gpio_write(pi_, motor_config.pinForward, 0);
        gpio_write(pi_, motor_config.pinReverse, 0);
       
        time_sleep(0.2);

        hardware_PWM(pi_, motor_config.pinEnable, 0, 0);

        motor_status_[i].enabled = false;
        motor_status_[i].last_cmd_vel = 0.0f;
        motor_status_[i].cmd_pwm = 0;
    }

    // Update system status
    if (isReady()){
        status_ = SysStatus::STOP;
    }

    if (verbose_debug_) std::cout << "DiffDrivePi: All motors stopped" << std::endl;
    return true;
}

// Return motor status
DiffDrivePi::MotorStatus& DiffDrivePi::getMotorStatus(Motor motor){
    
    // Check if the query is for an invalid motor
    assert(isValidMotor(motor) && "Invalid motor index");

    // Else return the motor status
    return motor_status_[static_cast<size_t>(motor)];
}

// Return motor configuration
DiffDrivePi::MotorConfig DiffDrivePi::getMotorConfig(Motor motor) const{
   
    if (!isValidMotor(motor)){
        // Return default motor_config for an invalid motor
        MotorConfig default_config = {};
        return default_config;
    }
    // Return motor configuration
    return config_.motors[static_cast<size_t>(motor)];
}

// Return true if inquired motor exists, false otherwise
bool DiffDrivePi::isValidMotor(Motor motor) const{
    
    // Check if inquired motor exists
    size_t index = static_cast<size_t>(motor);
    if (index >= static_cast<size_t>(Motor::TOTAL)){
        if (verbose_debug_) {std::cerr << "DiffDrivePi: Invalid motor index" << std::endl;}
        return false;
    }
    return true;
}

bool DiffDrivePi::isValidEncoder(Encoder encoder) const{
    
    // Check if inquired encoder exists
    size_t index = static_cast<size_t>(encoder);
    if (index >= static_cast<size_t>(Encoder::TOTAL)){
        if (verbose_debug_) {std::cerr << "DiffDrivePi: Invalid encoder index" << std::endl;}
        return false;
    }
    return true;
}

bool DiffDrivePi::isReady(){
   
    if (status_ == SysStatus::READY || status_ == SysStatus::RUNNING){
        return true;
    }
    // Auto-recover from recovarble states
    if (status_ == SysStatus::STOP || status_ == SysStatus::SLEEP){
        if(verbose_debug_){std::cout << "DiffDrivePi: System auto-recovering from state: " << static_cast<int>(status_) << std::endl;}
        if(pigpio_status_){
            // Making sure not in an ERROR state
            status_ = SysStatus::READY;
            return true;
        }
    }
    return false;
}

int32_t DiffDrivePi::getEncoderCounts(Encoder encoder){
   
    if(!isValidEncoder(encoder)){
        if(verbose_debug_){
            std::cerr << "DiffDrivPi: Invalid encoder index" << std::endl;
        }
        return 0;
    }
    return encoders_state_[static_cast<size_t>(encoder)].filtered.load();
}


void DiffDrivePi::resetEncoderCounts(){
   
    for(size_t i = 0; i < static_cast<size_t>(Encoder::TOTAL); i++){
        encoders_state_[i].count.store(0);
        encoders_state_[i].filtered.store(0);
    }

    if(verbose_debug_){
            std::cerr << "DiffDrivPi: Reset all encoder counts" << std::endl;
    }
}

double DiffDrivePi::getDistance(Encoder encoder){

    if(!isValidEncoder(encoder)){
        if(verbose_debug_){
            std::cerr << "DiffDrivPi: Invalid encoder index" << std::endl;
        }
        return 0.0;
    }
    const EncoderConfig& encoder_config = config_.encoders[static_cast<size_t>(encoder)];
    int32_t counts = getEncoderCounts(encoder);

    // Convert encoder counts to wheel rotations
    double real_cpr = static_cast<double>(encoder_config.cpr);
    double rotations = static_cast<double>(counts) / real_cpr;

    // Convert to meters using wheel circumference
    double distance = rotations * encoder_config.wheel_circumference;
    return distance;
}

double DiffDrivePi::getVelocity(Encoder encoder, int32_t previous_counts, std::chrono::duration<double> dt){
    
    if(!isValidEncoder(encoder) || dt.count()<= 0.0){
        return 0.0;
    }
    const EncoderConfig& encoder_config = config_.encoders[static_cast<size_t>(encoder)];

    // Calculate the change in counts since last read
    int32_t counts = getEncoderCounts(encoder);
    int32_t difference = counts - previous_counts;

    double real_cpr = static_cast<double>(encoder_config.cpr);
    double rotations = static_cast<double>(difference) / real_cpr;

    double velocity = rotations * encoder_config.wheel_circumference / dt.count();

    return velocity;
}

double DiffDrivePi::getDistanceRad(Encoder encoder){

    if(!isValidEncoder(encoder)){
        if(verbose_debug_){
            std::cerr << "DiffDrivPi: Invalid encoder index" << std::endl;
        }
        return 0.0;
    }
    int32_t counts = getEncoderCounts(encoder);

    // Convert encoder counts to wheel rotations in radians
    double distance = static_cast<double>(counts) * 2.0 * M_PI / static_cast<double>(config_.encoders[static_cast<size_t>(encoder)].cpr);
    return distance;
}    


double DiffDrivePi::getVelocityRad(Encoder encoder, int32_t previous_counts, std::chrono::duration<double> dt){
    
    if(!isValidEncoder(encoder) || dt.count()<= 0.0){
        return 0.0;
    }

    // Calculate the change in counts since last read
    int32_t counts = getEncoderCounts(encoder);
    int32_t difference = counts - previous_counts;

    double velocity = static_cast<double>(difference) * 2.0 * M_PI / static_cast<double>(config_.encoders[static_cast<size_t>(encoder)].cpr) / dt.count();

    return velocity;
}

double DiffDrivePi::PIDController(Controller controller, double velocity, double measured, std::chrono::duration<double> dt){

    // If controller is not enabled, return setpoint
    if(!config_.controllers[static_cast<size_t>(controller)].enable){
        return velocity;
    }
    // Calculate tracking error
    double error = velocity - measured;
    double corrected_velocity = velocity;

    // Controller Configurations
    ControllerState& controller_state = controllers_state_[static_cast<size_t>(controller)];
    const ControllerConfig& controller_config = config_.controllers[static_cast<size_t>(controller)];

    if(controller_config.kp != 0){
        corrected_velocity += p_controller(controller_config.kp, error);
    }

    if(controller_config.ki != 0){
        controller_state.integral +=  error * dt.count();

        if (controller_state.integral > controller_config.limits) {
            controller_state.integral = controller_config.limits;
        } else if (controller_state.integral < - controller_config.limits){
                controller_state.integral = -controller_config.limits;
        }

        corrected_velocity += i_controller(controller_config.ki, controller_state.integral);
    }

    if(controller_config.kd != 0){
        corrected_velocity += d_controller(controller_config.kd, error, controller_state.prev_error, dt);
        controller_state.prev_error = error;
    }
    
    return corrected_velocity;
}

// Private helper functions
bool DiffDrivePi::initpigpio(){
    
    // Start connection to Pigpiod Daemon
    pi_ = pigpio_start(nullptr, nullptr);

    if (pi_!= 0){
        if(verbose_debug_){
            std::cerr << "DiffDrivePi: pigpio initialization failed!" << std::endl;
            std::cerr << "DiffDrivePi: Did you activate the pigpiod daemon?" << std::endl;
        }
        pigpio_status_ = false;
        return false;
    }
    
    if(!configureGPIO()){
        if(verbose_debug_){
            std::cerr << "DiffDrivePi: GPIO pin configuration failed!" << std::endl;
            std::cerr << "DiffDrivePi: Make sure the configured GPIO are connected!" << std::endl;
        }
        pigpio_status_ = false;
        return false;
    }

    // If connection was succesfull return true
    pigpio_status_ = true;
    if (verbose_debug_)
        std::cout << "DiffDrivePi: pigpio initialization succesful" << std::endl;

    return true;
}

bool DiffDrivePi::configureGPIO(){
    
    for (size_t i = 0; i < static_cast<size_t>(Motor::TOTAL); i++){
        // Create a const reference to the motor pins defined in Robot Config
        const MotorConfig& motor_config = config_.motors[i];

        // Set Motor Pins as Outputs and set to LOW
        if(set_mode(pi_, motor_config.pinForward, PI_OUTPUT) != 0 || set_mode(pi_, motor_config.pinReverse, PI_OUTPUT) != 0){
            if(verbose_debug_){
                std::cerr << "DiffDrivePi: Error setting Directional Pins as Output" << std::endl;
            }
            return false;
        }

        time_sleep(0.1);

        if(gpio_write(pi_, motor_config.pinForward, 0) != 0 || gpio_write(pi_, motor_config.pinReverse, 0) != 0){
            if(verbose_debug_){
                std::cerr << "DiffDrivePi: Error writing to GPIO Pins" << std::endl;
            }
            return false;
        }

        // Set Hardware PWM      --      
        if(set_mode(pi_, motor_config.pinEnable, PI_OUTPUT) != 0){
            if(verbose_debug_){
                std::cerr << "DiffDrivePi: Error setting Enable Pin as Output" << std::endl;
            }
            return false;
        }

        time_sleep(0.1);

        // Frequency, Duty cycle(0 - 1,000,000)
        if(hardware_PWM(pi_, motor_config.pinEnable, 0, 0) != 0){
            if(verbose_debug_){
                std::cerr << "Error setting setting hardware PWM" << std::endl;
            }
            return false;
        }
        
        // If all tests pass, GPIO pins are configured                                        
        motor_status_[i].forward = true;
        motor_status_[i].enabled = false;
        motor_status_[i].last_cmd_vel = 0.0f;
        motor_status_[i].cmd_pwm = 0;
    }

    if(verbose_debug_){
        std::cout << "DiffDrivePi: GPIO configuration completed" << std::endl;
    }
    return true;
}

bool DiffDrivePi::initializeEncoders(){ 
    
    resetEncoderCounts();
    for (size_t i = 0; i < static_cast<size_t>(Encoder::TOTAL); i++){
        // Get encoder information
        Encoder encoder = static_cast<Encoder>(i);
        const EncoderConfig& encoder_config = config_.encoders[i];
        EncoderState& encoder_state = encoders_state_[i];

        // Initialize encoder state       
        encoder_state.last_state = 0;
        encoder_state.init = false;
        encoder_state.callback_a_id = -1;
        encoder_state.callback_b_id = -1;

        // Set Encoder Pins as Inputs and set to LOW
        if(set_mode(pi_, encoder_config.pinA, PI_INPUT) != 0 || set_mode(pi_, encoder_config.pinB, PI_INPUT) != 0){
            if(verbose_debug_){
                std::cerr << "DiffDrivePi: Error setting Encoder Pins as Inputs" << std::endl;
            }
            return false;
        }

        // Get the indices of the encoder channels for callback data structure
        size_t callback_a = i*2;
        size_t callback_b = i*2 + 1; 

        callback_data_[callback_a] = {this, encoder, true}; // Channel A;
        callback_data_[callback_b] = {this, encoder, false}; // Channel B;

        int callback_idx_a = callback_ex(pi_, encoder_config.pinA, EITHER_EDGE, encoderCallbackWrapper, &callback_data_[callback_a]);
        if(callback_idx_a < 0) {
            if(verbose_debug_){
                std::cerr << "DiffDrivePi: Failed to register callback for encoder: " << i << " channel A" << std::endl;
            }
            return false;
        }

        int callback_idx_b = callback_ex(pi_, encoder_config.pinB, EITHER_EDGE, encoderCallbackWrapper, &callback_data_[callback_b]);
        if(callback_idx_b < 0) {
            if(verbose_debug_){
                std::cerr << "DiffDrivePi: Failed to register callback for encoder: " << i << " channel B" << std::endl;
            }
            return false;
        }

        // If callback declarations succed, add to encoder state
        encoder_state.callback_a_id = callback_idx_a;
        encoder_state.callback_b_id = callback_idx_b;

        // Read initial state's to determine current position
        uint8_t pinA_state = gpio_read(pi_, encoder_config.pinA);
        uint8_t pinB_state = gpio_read(pi_, encoder_config.pinB);
        // State form: BA, Potential states:
        /*
        00: Both pins Low
        01: Pin A High, Pin B Low
        11: Both pins High
        10: Pin A Low, Pin B High
        */
        encoder_state.last_state = (pinB_state << 1) | pinA_state;
        encoder_state.init = true;
    }

    if(verbose_debug_){
        std::cout << "DiffDrivePi: All encoders initialized succesfully" << std::endl;
    }

    return true;
}

void DiffDrivePi::shutdownEncoders(){
    if(verbose_debug_){
        std::cout << "DiffDrivePi: Shutting down encoders" << std::endl;
    }
    for (size_t i = 0; i < static_cast<size_t>(Encoder::TOTAL); i++){

        // Get encoder information
        const EncoderConfig& encoder_config = config_.encoders[i];
        EncoderState& encoder_state = encoders_state_[i];

        encoder_state.init = false;

        // Cancel the callbacks
        if(encoder_state.callback_a_id >= 0){
            callback_cancel(encoder_state.callback_a_id);
            encoder_state.callback_a_id = -1;
        }

        if(encoder_state.callback_b_id >= 0){
            callback_cancel(encoder_state.callback_b_id);
            encoder_state.callback_b_id = -1;
        }

        time_sleep(0.05);

        // Return pins to safe state
        set_mode(pi_, encoder_config.pinA, PI_INPUT);
        set_mode(pi_, encoder_config.pinB, PI_INPUT);

        resetEncoderCounts();
        encoder_state.last_state = 0;
    }

    if(verbose_debug_){
        std::cout << "DiffDrivePi: Encoder Shutdown Succesful" << std::endl;
    }
}

uint32_t DiffDrivePi::velocityToPWM(float velocity) const{

    // Convert to absolute velocity (direction is handled by directional pins logic)
    float abs_vel = std::abs(velocity);
   
    if (abs_vel < 0.05) {  // Deadband Velocity
        return 0;
    }
    // Make sure the velocity is greater than the min velocity
    if (abs_vel > 0.0f && abs_vel < config_.min_velocity) {abs_vel = config_.min_velocity;}
    
    // Apply user defined max velocity
    if (abs_vel > config_.max_velocity) {abs_vel = config_.max_velocity;}

    uint32_t pwm_value;

    // Convert to PWM
    pwm_value = static_cast<uint32_t>(abs_vel * 1000000);
    if (pwm_value > 1000000) {pwm_value = 1000000;}

    return static_cast<uint32_t>(pwm_value);
}

void DiffDrivePi::encoderCallbackWrapper(int pi, unsigned gpio_pin, unsigned level, uint32_t tick, void* userdata){
   
    // Cast userdata back to class data structure
    CallbackData* data = static_cast<CallbackData*>(userdata);

    // Call the instance method with the decoded information
    data->controller->handleInterrupt(data->encoder, data->is_channel_a, level, tick);
}

const int8_t DiffDrivePi::quadrature_table_[16] = {
    // Forward rotation: 00->01->11->10->00 (repeating)
    // Reverse rotation: 00->10->11->01->00 (repaeting)
    0, 1, -1, 0,    // From state 00
    -1, 0, 0, 1,    // From state 01
    1, 0, 0, -1,    // From state 10
    0, -1, 1, 0     // From state 11
};

void DiffDrivePi::handleInterrupt(Encoder encoder, bool is_channel_a, unsigned level, uint32_t tick){

    EncoderState& encoder_state = encoders_state_[static_cast<size_t>(encoder)];

    if(!encoder_state.init){
        return;
    }

    uint8_t current_state = encoder_state.last_state;
    if(is_channel_a){
        // Update Channel A bit
        current_state = (current_state & 0b10) | (level & 0b01);
    } else{
        // Update Channel B bit
        current_state = (current_state & 0b01) | ((level & 0b01) << 1);
    }

    // Determine the state and direction
    uint8_t table_idx = (encoder_state.last_state << 2) | current_state;
    int8_t count_change = quadrature_table_[table_idx];

    // Update encoder counts
    encoder_state.count.fetch_add(count_change);
    encoder_state.last_state = current_state;

    int32_t current_count = encoder_state.count.load();
    int32_t filtered_count = encoder_state.filtered.load();

    int32_t new_filtered = static_cast<int32_t>(0.2 * current_count + (1.0 - 0.2) * filtered_count);
    encoder_state.filtered.store(new_filtered);
}

double DiffDrivePi::p_controller(double kp, double error) const{
    return kp * error;
}
double DiffDrivePi::i_controller(double ki, double integrator) const{
    return ki * integrator;
}
double DiffDrivePi::d_controller(double kd, double error, double prev_error, std::chrono::duration<double> dt) const{
    return kd * (error - prev_error) / dt.count();
}

void DiffDrivePi::resetControllers(){
    for (size_t i=0; i < static_cast<size_t>(Controller::TOTAL); i++){
        controllers_state_[i].prev_error = 0;
        controllers_state_[i].integral = 0;
    }
}
