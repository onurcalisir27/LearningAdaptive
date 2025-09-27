#include "rclcpp/rclcpp.hpp"
#include "rover_control/self_tuning_regulator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rover_control/msg/params.hpp"
#include <tuple>
#include <cmath>
#include <memory>
#include <Eigen/Dense>
#include <vector>
#include <chrono>
using namespace std::chrono_literals;

class PendulumControlNode : public rclcpp::Node
{
    public:

        PendulumControlNode() : Node("pendulum_control_node"){

            this->declare_parameter("lambda", 0.98);
            this->get_parameter("lambda", lambda);

            this->declare_parameter("desired_angle", 0.0);
            this->get_parameter("desired_angle", desired_angle);

            this->declare_parameter("u_bound", 20.0);
            double input_bound = this->get_parameter("u_bound").as_double();

            this->declare_parameter("update_freq", 20);
            int update_frequency = this->get_parameter("update_freq").as_int();

            this->declare_parameter("kp", 0.0);
            kp = this->get_parameter("kp").as_double();

            this->declare_parameter("ki", 0.0);
            ki = this->get_parameter("ki").as_double();

            this->declare_parameter("kd", 0.0);
            kd = this->get_parameter("kd").as_double();

            int state_history = 2;
            int state_dim = 1;
            int input_history = 2;
            int input_dim = 1;
            double covariance = 1e6;
            controller_.init(state_dim, input_dim, state_history, input_history, lambda);
            RCLCPP_INFO(this->get_logger(), "Self Tuning Regulator Initialized!");

            theta_bound = 30.0;
            controller_.set_bounds(theta_bound, input_bound);
            controller_.set_frequency(update_frequency);
            controller_.set_covariance(covariance);

            VectorXd Theta_guess(4);
            Theta_guess << -1.0, 0.5, 0.1, 0.05;

            p_inputs = VectorXd::Zero(input_history*input_dim);
            p_states = VectorXd::Zero(state_history*state_dim);

            auto pid_gains = std::make_tuple(kp, ki, kd);
            controller_.set_pid(pid_gains);

            // prev_time_ = std::chrono::high_resolution_clock::now();
            param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

            auto sensor_qos = rclcpp::QoS(2).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
            joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&PendulumControlNode::read, this, std::placeholders::_1));

            auto control_qos = rclcpp::QoS(5).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
            torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pendulum_controller/commands", 10);
            params_pub_ = this->create_publisher<rover_control::msg::Params>("params", 10);

            params_timer_ = this->create_wall_timer(20ms, std::bind(&PendulumControlNode::control, this));

            RCLCPP_INFO(this->get_logger(), "Self Tuning Regulator started!");
            counter_ = 0;
        }

    private:

        void read(const sensor_msgs::msg::JointState::SharedPtr msg){

            double current_angle = wrap(msg->position[0]);
            double current_torque = msg->effort[0];
            if (counter_ % 4) {
              RCLCPP_INFO(this->get_logger(), "Current Angle: %f", current_angle);
            }
            angles.push_back(current_angle);
            torques.push_back(current_torque);
            counter_++;
           //  if (counter_ > 2) {
           //    control();
           // }
        }

        void control(){

            double input=0.0;
            int step = std::min(angles.size(), torques.size());
            if (step < 3){
              input = 0.0;
              publish_torque(input);

            } else {
              p_states << angles[step-2], angles[step-3];
              p_inputs << torques[step-2], torques[step-3];

              double current = angles[step-1];
              double desired = M_PI - desired_angle;

              input = controller_.str(desired, current, p_states, p_inputs);
              auto error = controller_.get_error(desired, current);
              RCLCPP_INFO(this->get_logger(), "State Error %f, Prediction Error %f, Control Error %f:", error(0), error(1), error(2));
              publish_torque(input);
            }

        }
        void publish_torque(double input){

            auto control_msg = std_msgs::msg::Float64MultiArray();
            control_msg.data = {input};
            torque_pub_->publish(control_msg);
            RCLCPP_INFO(this->get_logger(), "Input Computed: %f", input);
        }

        void feedback(){

            MatrixXd Theta = controller_.get_theta();
            MatrixXd Cov = controller_.get_covariance();
            auto msg = rover_control::msg::Params();

            msg.estimate.resize(Theta.size());
            for(int i = 0; i < Theta.size(); ++i) {
                msg.estimate[i] = Theta(i);
            }

            msg.covariance.resize(Cov.rows() * Cov.cols());
            for(int i = 0; i < Cov.rows(); ++i) {
                for(int j = 0; j < Cov.cols(); ++j) {
                    msg.covariance[i * Cov.cols() + j] = Cov(i, j);
                }
            }

            params_pub_->publish(msg);
        }


        double wrap(double x){
          x = fmod(x , 2.00 * M_PI);
          if (x < 0)
              x += 2.00 * M_PI;
          return x;
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
        rclcpp::Publisher<rover_control::msg::Params>::SharedPtr params_pub_;
        rclcpp::TimerBase::SharedPtr params_timer_;
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
        std::chrono::high_resolution_clock::time_point prev_time_;

        SelfTuningRegulator controller_;
        double desired_angle;
        VectorXd p_states;
        VectorXd p_inputs;

        std::vector<double> angles;
        std::vector<double> torques;

        std::vector<double> process_errors;
        double theta_bound;
        double lambda;
        int counter_;
        double kp, kd, ki;

};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PendulumControlNode>());
  rclcpp::shutdown();
  return 0;
}
