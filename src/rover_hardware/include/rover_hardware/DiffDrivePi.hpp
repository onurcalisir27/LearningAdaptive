
// This header file is for DIFF_DRIVE_PI_HPP, 
// This class implements the hardware interface functionality between a 
// Raspberry Pi 5, and H-Bridge (L298N) motor driver boards, as well as quadrature
// encoders. The API is intended to be used in together with ROS2_Control, which will call the
// methods of this class to control a Differential Drive Robot. This class will make use of pigpio library
// https://github.com/joan2937/pigpio, Please refer to the documentation the repository provides for clarification

// Author: Onur Calisir

#ifndef DIFF_DRIVE_PI_HPP
#define DIFF_DRIVE_PI_HPP

// #include <pigpiod_if2.h>
#include "pigpiod_if2.h"

#include <chrono>
#include <cstdint>
#include <atomic>
#include <cstdlib>
#include <array>
#include <assert.h>
#include <iostream>
#include <cstring>

// Current Core API:
/*
1. Lifecycle Management
    - Constructor/Destructor: used to start and destroy the motor control system
    - initialize(): used to configure the system; wiringPi, GPIO assignment, system status

    - shutdown(): used to cleanup resource allocation by the system by reseting to defaults and removing pinMode's

2. Motor Control Interface
    - setRobotConfig: set up desired robot configuration
    - setMotorVel: set velocities for individual motor
    - DiffDriveControl: implement proper diff drive logic
    - stop(): stop with proper decel
    - emergencyStop(): stops the system instantly, used my lifecycle management

3.  Encoder State Interface
    - catchInterrupts()
    - calculatePos()
    - calculateVel()

4. State Monitor
    - getDefaultConfig(): used to initialize the class using a dummy config
    - get_config(): return current robot configuration
    - getMotorStatus(): return current motor status
    - getStatus(): return current motor state
    - isReady(): returns true if system is at ready or running state
    - getLastCmdTime(): returns the duration since last command was received

5. Helpers
    - (): initialize the  library
    - (): initialize the GPIO assignments based on the robot configuration
    - velocityToPWM(): convert velocity commands to PWM signals for the motors
    - fixDirectionPins(): invert the direction pins of a motor if the invDir flag is true for consistent velocity setting logic
*/
class DiffDrivePi{
    public:

    // Diff Drive Control - Left and Right motors control both Left and Back
    enum class Motor : uint8_t{
        LEFT = 0,
        RIGHT = 1,
        TOTAL = 2
    };

    // System status assignment
    enum class SysStatus : uint8_t{
        SLEEP = 0,          // uninitialized, non configured
        READY = 1,          // ready and listening for vel cmd
        RUNNING = 2,        // executing vel cmd
        STOP = 3,           // stopping at destination
        ERROR = 4           // returned an error
    };

    // Motor Configurations
    struct MotorConfig{
        uint8_t pinForward; // forward direction pin
        uint8_t pinReverse; // reverse direction pin
        uint8_t pinEnable; // enable pin of the motor
        uint16_t pwm_freq; // pwm frequency (5-25 kHz for DC motors)
    };

    // Motor Status
    struct MotorStatus{
        bool forward;       // is motor in forward
        bool enabled;       // is motors enabled
        float last_cmd_vel; // last commanded velocity
        uint16_t cmd_pwm;    // current pwm signal
    };

    enum class Encoder : uint8_t{
        LEFT_FRONT = 0,
        LEFT_BACK = 1,
        RIGHT_FRONT = 2,
        RIGHT_BACK = 3,
        TOTAL = 4
    };

    // Encoder Configurations
    struct EncoderConfig{
        uint8_t pinA;               // Encoder pin 1
        uint8_t pinB;               // Encoder pin 2
        int32_t cpr;                // Counts per revolution
        double wheel_circumference; // wheel circumference in meters
    };

    struct EncoderState{
        std::atomic<int32_t> count{0};      // Used to accumulate encoder pulses
        std::atomic<int32_t> filtered{0}; // Used to accumulate previous time step encoder pulses
        volatile uint8_t last_state;        // used to determine rotation direction
        volatile bool init;                 // do not start reading before initialization
        int callback_a_id;                  // id to store declared encoder callback
        int callback_b_id;                  // id to store declared encoder callback
    };

    // Controller Configurations
    enum class Controller : uint8_t{
        LEFT = 0,
        RIGHT = 1,
        TOTAL = 2
    };

    // Encoder Configurations
    struct ControllerConfig{
        uint8_t enable;     // 0:disabled, 1:enabled
        double kp;          // Proportional Controller Gain
        double ki;          // Integral Controller Gain
        double kd;          // Derivative Controller Gain
        double limits;      // Integral Windup Limits
    };

    struct ControllerState{
        double prev_error;      // Controller Previous Error
        double integral;        // Controller Previous Integral
    };


    // Robot Configuration to govern the complete robot
    struct RobotConfig{
        std::array<MotorConfig, static_cast<size_t>(Motor::TOTAL)> motors;
        std::array<EncoderConfig, static_cast<size_t>(Encoder::TOTAL)> encoders;
        std::array<ControllerConfig, static_cast<size_t>(Controller::TOTAL)> controllers;

        float min_velocity;     // to get the robot to start moving
        float max_velocity;     // max allowed velocity by the hardware (by default 1.0)
    };

    /// Constructor - Destructor ///
    DiffDrivePi();
    DiffDrivePi(bool verbose_debug);
    ~DiffDrivePi();

    // Hardware initialization, return false if any errors are encountered, true otherwise
    bool initialize(const RobotConfig& config);

    // Safe shutdown & cl1eanup of hardware objects
    void shutdown();

    bool reset();
    bool recover();

    // Return system status
    SysStatus getStatus() const{return status_;}

    // Return motor status
    MotorStatus& getMotorStatus(Motor motor);

    // Return motor configuration
    MotorConfig getMotorConfig(Motor motor) const;

    // Return true if inquired motor exists, false otherwise
    bool isValidMotor(Motor motor) const;

    // Return true if inquired encoder exists, false otherwise
    bool isValidEncoder(Encoder encoder) const;

    // Returns true if system is ready for velocity commands, false otherwise
    bool isReady();

    // Return default Robot Configuration
    static RobotConfig getDefaultConfig();

    // Return current Robot Configuration(motors, parameters ...)
    const RobotConfig& getConfig() const{ return config_;}

    // Set velocities for individual motor; return true if succesful, false otherwise
    bool setMotorVel(Motor motor, double velocity);

    // Differential Drive logic - main function to call with diff drive velocities
    bool DiffDriveControl(double left_vel, double right_vel);

    // Stop motors - gradual decrease in motor velocities
    bool Stop();

    // Read from encoders
    int32_t getEncoderCounts(Encoder encoder);

    // Reset encoder counts
    void resetEncoderCounts();

    // Calculate Position and Velocity in metersfrom Encoder Data
    double getDistance(Encoder encoder);
    double getVelocity(Encoder encoder, int32_t previous_counts, std::chrono::duration<double> dt);

    // Calculate Position and Velocity in radians from Encoder Data
    double getDistanceRad(Encoder encoder);
    double getVelocityRad(Encoder encoder, int32_t previous_counts, std::chrono::duration<double> dt);
    
    // Calculate Controller correction for tracking error
    double PIDController(Controller controller, double velocity, double measured, std::chrono::duration<double> dt);
    
    void resetControllers();
    
    private:
    // Init WiringPi library, configure Pin assignments return true if succesful, false otherwise
    bool initpigpio();

    bool configureGPIO();

    bool initializeEncoders();
    
    void shutdownEncoders();

    // Convert velocity commands to mapped PWM outputs
    uint32_t velocityToPWM(float velocity) const;

    // Member variables
    RobotConfig config_;              // Current robot configuration
    SysStatus status_;                // Current system status
    int pi_;                          // Pigpiod daemon connection object
    bool pigpio_status_;              // pigpio interface status

    // Motor status tracking
    std::array<MotorStatus, static_cast<size_t>(Motor::TOTAL)> motor_status_;
    std::array<EncoderState, static_cast<size_t>(Encoder::TOTAL)> encoders_state_;
    std::array<ControllerState, static_cast<size_t>(Controller::TOTAL)> controllers_state_;

    bool verbose_debug_;        // By default is set to true, can be turned off in the Robotconfig
    int reset_counter_;

    // Encoder Callback for catching interrupts and counting 
    struct CallbackData{
        DiffDrivePi* controller;
        Encoder encoder;
        bool is_channel_a;
    };

    // Encoder lookup table
    static const int8_t quadrature_table_[16];

    // Encoder Callback function for Interrupt on Channel A, B 
    std::array<CallbackData, static_cast<size_t>(Encoder::TOTAL)*2> callback_data_;
    static void encoderCallbackWrapper(int pi, unsigned gpio_pin, unsigned level, uint32_t tick, void* userdata);
    void handleInterrupt(Encoder encoder, bool is_channel_a, unsigned level, uint32_t tick);

    // Controllers
    double p_controller(double kp, double error) const;
    double i_controller(double ki, double integrator) const;
    double d_controller(double kd, double error, double prev_error, std::chrono::duration<double> dt) const;

};

#endif // DIFF_DRIVE_PI_HPP