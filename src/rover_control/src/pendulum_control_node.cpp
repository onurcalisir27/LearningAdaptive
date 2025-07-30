#include "rclcpp/rclcpp.hpp"
#include "rover_control/self_tuning_regulator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <chrono>
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
            controller_timer_ = this->create_wall_timer(100ms, std::bind(&PendulumControlNode::controlPendulum, this));
            controller_ = std::make_unique<SelfTuningRegulator>();

            input_history_order_ = 1;   // Starting off small
            output_history_order_ = 2; 

            input_dim_ = 1;             // torque on one joint [j] for j many joints
            output_dim_ = 2;            // position and velocity of the joint [2j] for j many joints

            lambda_ = 0.98;             // start with theoretical prescription
            double init_cov = 1000.0;   // Start with large initial covariance
	    
	        prev_input_ = VectorXd::Zero(input_dim_);
	        current_state_ = VectorXd::Zero(output_dim_);
             
            controller_->init(output_history_order_, input_history_order_, input_dim_, output_dim_, lambda_, init_cov);
            controller_->start();
            
            desired_state_ = VectorXd::Zero(output_dim_); 
        }

    private:

        void get_feedback(const sensor_msgs::msg::JointState::SharedPtr msg){
            
            for(size_t i=0; i < msg->name.size(); i++){ 
                current_state_.segment(i*output_dim_, 2) << msg->position[i], msg->velocity[i];
            }
        } 

        void controlPendulum(){

            VectorXd control_effort = controller_->computeControl(desired_state_, current_state_, prev_input_);
	        prev_input_ = control_effort;
            std_msgs::msg::Float64MultiArray command;
            command.data = {control_effort(0,0)};
            torque_pub_->publish(command);
        }


        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
        rclcpp::TimerBase::SharedPtr controller_timer_;
        
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
