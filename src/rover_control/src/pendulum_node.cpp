#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <cstdlib>
#include <memory>
#include <Eigen/Dense>
#include <iostream>
#include <cstdlib>
#include <ctime>

using namespace std::chrono_literals;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class PendulumNode : public rclcpp::Node
{
    public:
        PendulumNode() : Node("pendulum_node"){

            this->declare_parameter("forgetting_factor", 0.98);
            lambda_ = this->get_parameter("forgetting_factor").as_double();

            this->declare_parameter("desired_angle", 0.0);
            desired_state_ = this->get_parameter("desired_angle").as_double();

            joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&PendulumNode::get_joint_states, this, std::placeholders::_1));

            torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("pendulum_controller/commands",50);
            metrics_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("metrics",10);

            control_timer_ = this->create_wall_timer(50ms, std::bind(&PendulumNode::controlPendulum, this));
            update_timer_ = this->create_wall_timer(200ms, std::bind(&PendulumNode::updatePendulum, this));
            srand(time(0));

            limit = 250.0;
            n_ = 2;
            m_ = 1;
            nm_ = n_ + m_;

            A = VectorXd::Zero(n_);
            B = 0.0;
            Theta = VectorXd::Random(nm_);
            Phi = VectorXd::Zero(nm_);
            L = VectorXd::Zero(nm_);
            P = MatrixXd::Identity(nm_, nm_) * 10000;

            current_state_ = VectorXd::Zero(n_);
            previous = 0.0;
            prev_previous = 0.0;
            prev_input_ = 0.0;
    }

    private:

        void get_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg){

            double current = msg->position[0];
            current_state_ << current, previous;
            Phi << previous, prev_previous, prev_input_;

            // Construct the Phi Vector
            // std::cout << "Your Phi Vector: " <<  Phi << std::endl;
            prev_previous = previous;
            previous = current;
        }

        void controlPendulum(){

            // double input = (double)rand() / RAND_MAX * 0.005;
            // std::cout << "Random noisy input is: " << input << std::endl;
            A  = Theta.segment(0, 2);
            // std::cout << "A is: " << A << std::endl;
            B = Theta(2);
            // std::cout << "B is: " << B << std::endl;

            double error = desired_state_ - (A.transpose() * current_state_)(0);
            std::cout << "Computed error: " << error << std::endl;

            double input = error / B;
            std::cout << "Input Computed: " << input << std::endl;
            input = std::clamp(input, -limit, limit);
            prev_input_ = input;

            std_msgs::msg::Float64MultiArray command;
            command.data = {input};
            torque_pub_->publish(command);

            double measurement_error = desired_state_ - current_state_(0);
            std_msgs::msg::Float64MultiArray metrics;
            metrics.data = {desired_state_, current_state_(0), error, measurement_error};
            metrics_pub_->publish(metrics);
        }

        void updatePendulum(){

            // Phi << angles_.front(), angles_.back(), prev_input_;
            Theta = Theta + L * (current_state_(0) - Phi.transpose() * Theta);
            // std::cout << "Your Theta is: " << Theta << std::endl;

            double L_den = lambda_ + (Phi.transpose() * P * Phi)(0,0);
            L = P * Phi / L_den;
            std::cout << "L Gain Vector: " << L << std::endl;

            // P = (MatrixXd::Identity(3,3) - L * Phi.transpose()) * P / lambda_;
            P = (P - L * Phi.transpose() * P) / lambda_;

            std::cout << "Covariance MAtrix: \n " << P << "\n\n";

       }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr metrics_pub_;

        rclcpp::TimerBase::SharedPtr control_timer_;
        rclcpp::TimerBase::SharedPtr update_timer_;

        VectorXd current_state_;
        double prev_input_;
        double desired_state_;

        VectorXd A;
        double B;
        MatrixXd P;

        VectorXd Theta;
        VectorXd Phi;
        VectorXd L;
        uint n_, m_, nm_;
        double lambda_;
        double limit;
        double previous;
        double prev_previous;

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PendulumNode>());
  rclcpp::shutdown();
  return 0;
}
