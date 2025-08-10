#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <memory>
#include <Eigen/Dense>
#include <algorithm>

#define CLAMP_TORQUE (double) 200.0

using namespace std::chrono_literals;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class PendulumControlNode : public rclcpp::Node
{
    public:
        PendulumControlNode() : Node("pendulum_control_node"){

            this->declare_parameter("forgetting_factor", 0.98);
            lambda_ = this->get_parameter("forgetting_factor").as_double();

            this->declare_parameter("desired_angle", 0.0);
            double desired_angle = this->get_parameter("desired_angle").as_double();

            joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&PendulumControlNode::get_feedback, this, std::placeholders::_1));
            torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pendulum_controller/commands", 10);
            controller_timer_ = this->create_wall_timer(100ms, std::bind(&PendulumControlNode::controlPendulum, this));

            covariance_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/covarianceMatrix", 10);
            covariance_timer_ = this->create_wall_timer(200ms, std::bind(&PendulumControlNode::publishCov, this));

            parameters_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/parameters", 10);
            parameters_timer_ = this->create_wall_timer(10ms, std::bind(&PendulumControlNode::publishTheta, this));

            input_history_order_ = 1;
            output_history_order_ = 2;

            double init_cov = 10000.0;   // Start with large initial covariance

	        prev_input_ = VectorXd::Zero(1);
	        current_state_ = VectorXd::Zero(2);

            desired_state_ = VectorXd::Ones(2) * desired_angle;
        }

    private:

        void get_feedback(const sensor_msgs::msg::JointState::SharedPtr msg){

            // for(size_t i=0; i < msg->name.size(); i++){
            //     current_state_.segment(i*output_dim_, 2) << msg->position[i];
            // }
            current_state_(0,0) = msg->position[0];
        }

        void controlPendulum(){

            float input = std::clamp(control_effort(0,0), -CLAMP_TORQUE, CLAMP_TORQUE);
	    //            float input = control_effort(0,0);
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
	  //            VectorXd theta = controller_->update(desired_state_);
	    controller_->update(desired_state_);
	    /*
            std_msgs::msg::Float64MultiArray parameters;

            parameters.data.assign(theta.data(), theta.data() + theta.size());
	        parameters_pub_->publish(parameters);
	    */
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
        double lambda_;
};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PendulumControlNode>());
  rclcpp::shutdown();
  return 0;
}
