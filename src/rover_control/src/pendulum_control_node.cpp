#include "rclcpp/rclcpp.hpp"
#include "rover_control/self_tuning_regulator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rover_control/msg/params.hpp"

#include <cmath>
#include <memory>
#include <Eigen/Dense>
#include <vector>
using namespace std::chrono_literals;

class PendulumControlNode : public rclcpp::Node
{
    public:
        PendulumControlNode() : Node("pendulum_control_node"){

            this->declare_parameter("forgetting_factor", 0.98);
            double lambda = this->get_parameter("forgetting_factor").as_double();

            this->declare_parameter("desired_angle", 0.0);
            double desired_angle = this->get_parameter("desired_angle").as_double();

            this->declare_parameter("u_bound", 10.0);
            double input_bound = this->get_parameter("u_bound").as_double();

            this->declare_parameter("update_freq", 20);
            int update_frequency = this->get_parameter("update_freq").as_int();

            // Controller Parameters
            int state_history = 2;
            int state_dim = 1;
            int input_history = 2;
            int input_dim = 1;
            double covariance = 1e6;
            controller_.init(state_dim, input_dim, state_history, input_history, lambda);
            RCLCPP_INFO(this->get_logger(), "Self Tuning Regulator Initialized!");

            // Set the bounds
            double theta_bound = 10.0;
            controller_.set_bounds(theta_bound, input_bound);
            controller_.set_frequency(update_frequency);
            controller_.set_covariance(covariance);

            // MatrixXd Theta_i(4,1);
            // Theta_i << -1.0, 1.0, 0.1, 0.1;
            // controller_.set_theta(Theta_i);

            // States and Inputs
            desired_state_ = VectorXd::Zero(state_dim);
            desired_state_(0) = desired_angle;

            current_state = VectorXd::Zero(state_dim);
            prev_input = VectorXd::Zero(input_dim);

            joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&PendulumControlNode::control, this, std::placeholders::_1));

            torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pendulum_controller/commands", 10);
            params_pub_ = this->create_publisher<rover_control::msg::Params>("params", 10);
            params_timer_ = this->create_wall_timer(10ms, std::bind(&PendulumControlNode::feedback, this));

            RCLCPP_INFO(this->get_logger(), "Self Tuning Regulator started!");
        }

    private:

        void control(const sensor_msgs::msg::JointState::SharedPtr msg){

            current_state(0) = wrap(msg->position.at(0));
            VectorXd input = controller_.compute_input(desired_state_, current_state, prev_input);
            prev_input = input;

            auto control_msg = std_msgs::msg::Float64MultiArray();
            control_msg.data.resize(input.size());
            for(int i = 0; i < input.size(); ++i) {
                control_msg.data[i] = input(i);
            }
            torque_pub_->publish(control_msg);

            auto error = controller_.get_error(desired_state_);
            process_errors.push_back(error);
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
            error_metrics();
        }

        void error_metrics(){
            double total_error=0.0;
            for(auto element : process_errors){
              total_error += element;
            }
            double ave_error = total_error / process_errors.size();
            std::cout << "Average Error of this run was: " << ave_error << std::endl;

        }
        double wrap(double x){
          x = fmod(x + M_PI, 2.00 * M_PI);
          if (x < 0)
              x += 2.00 * M_PI;
          return x - M_PI;
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
        rclcpp::Publisher<rover_control::msg::Params>::SharedPtr params_pub_;
        rclcpp::TimerBase::SharedPtr params_timer_;

        SelfTuningRegulator controller_;
        VectorXd current_state;
        VectorXd prev_input;
        VectorXd desired_state_;
        std::vector<double> process_errors;

};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PendulumControlNode>());
  rclcpp::shutdown();
  return 0;
}
