#include "rclcpp/rclcpp.hpp"
#include "rover_control/self_tuning_regulator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <chrono>
#include <array>
#include <memory>
#include <Eigen/Dense>

using namespace std::chrono_literals;

class PendulumControlNode : public rclcpp::Node
{
    public:
        PendulumControlNode() : Node("pendulum_control_node"){

            joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&PendulumControlNode::get_feedback, this, std::placeholders::_1));
            torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pendulum_controller/commands", 10);
            controller_timer_ = this->create_wall_timer(1ms, std::bind(&PendulumControlNode::controlPendulum, this));

            covariance_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/covarianceMatrix", 10);
            covariance_timer_ = this->create_wall_timer(200ms, std::bind(&PendulumControlNode::publishCov, this));

            parameters_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/parameters", 10);
            parameters_timer_ = this->create_wall_timer(200ms, std::bind(&PendulumControlNode::publishTheta, this));

            controller_ = std::make_unique<SelfTuningRegulator>();

            input_history_order_ = 1;
            output_history_order_ = 2; 

            input_dim_ = 1;             // torque
            output_dim_ = 1;            // angle

            lambda_ = 0.99;
            double init_cov = 10000.0;   // Start with large initial covariance
	    
	        prev_input_ = VectorXd::Zero(input_dim_);
	        current_state_ = VectorXd::Zero(output_dim_);
             
            controller_->init(output_history_order_, input_history_order_, input_dim_, output_dim_, lambda_, init_cov);
            controller_->start();
            
            desired_state_ = VectorXd::Zero(output_dim_); 
        }

    private:

        void get_feedback(const sensor_msgs::msg::JointState::SharedPtr msg){
            
            // for(size_t i=0; i < msg->name.size(); i++){ 
            //     current_state_.segment(i*output_dim_, 2) << msg->position[i];
            // }
            current_state_(0,0) = msg->position[0];
            // RCLCPP_INFO(this->get_logger(), "The current state is: ", current_state_);

        } 

        void controlPendulum(){

            VectorXd control_effort = controller_->computeControl(desired_state_, current_state_, prev_input_);
            RCLCPP_INFO(this->get_logger(), "Commanding torque: ", control_effort);

            float input = std::clamp(control_effort(0,0), -5.0, 5.0);
	        prev_input_(0,0) =  input;

            std_msgs::msg::Float64MultiArray command;
            command.data = {input};
            torque_pub_->publish(command);
        }

        void publishCov(){
            MatrixXd covMat = controller_->get_covariance_matrix();
            std_msgs::msg::Float64MultiArray covariance;
            
            covariance.data.assign(covMat.data(), covMat.data() + covMat.size());
            covariance_pub_->publish(covariance);
        }

        void publishTheta(){
            VectorXd theta = controller_->get_theta_parameters();
            std_msgs::msg::Float64MultiArray parameters;
            
            parameters.data.assign(theta.data(), theta.data() + theta.size());
            covariance_pub_->publish(parameters);
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr covariance_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr parameters_pub_;

        rclcpp::TimerBase::SharedPtr controller_timer_;
        rclcpp::TimerBase::SharedPtr covariance_timer_;
        rclcpp::TimerBase::SharedPtr parameters_timer_;

        
        std::unique_ptr<SelfTuningRegulator> controller_;
        
        VectorXd current_state_;
        VectorXd prev_input_;
        VectorXd desired_state_;

        int input_history_order_, output_history_order_;
        int system_dim_, phi_dim_, output_dim_, input_dim_;
        double lambda_;
};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PendulumControlNode>());
  rclcpp::shutdown();
  return 0;
}
