#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rover_utils/self_tuning_regulator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "rover_msgs/msg/str_params.hpp"
#include <cmath>
#include <memory>
#include <Eigen/Dense>
#include <deque>

using namespace std::chrono_literals;
using rover_utils::SelfTuningRegulator;
using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace pendulum_action
{
class PendulumControlNode : public rclcpp::Node
{
  using Float64 = std_msgs::msg::Float64;
  using Params = rover_msgs::msg::StrParams;

public:
  explicit PendulumControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("pendulum_control_node", options)
{
    this->declare_parameter("lambda", 0.98);
    this->get_parameter("lambda", lambda);
    this->declare_parameter("u_bound", 3.0);
    this->get_parameter("u_bound", input_bound);
    // this->declare_parameter("desired_angle", 0.0);
    // this->get_parameter("desired_angle", desired_angle);

    desired_angle = 0.0;
    int state_history = 2;
    int state_dim = 1;
    int input_history = 2;
    int input_dim = 1;
    double covariance = 1e4;
    controller_.init(state_dim, input_dim, state_history, input_history, lambda);
    RCLCPP_INFO(this->get_logger(), "Self Tuning Regulator Initialized!");

    VectorXd bounds = VectorXd::Ones(input_dim) * input_bound;
    controller_.set_bounds(bounds);
    controller_.set_covariance(covariance);

    MatrixXd Theta_guess(4,1);
    Theta_guess << -1.0, 1.0, 0.1, 0.1;
    controller_.set_theta(Theta_guess);

    p_inputs = VectorXd::Zero(input_history*input_dim);
    p_states = VectorXd::Zero(state_history*state_dim);

    desired_state = VectorXd::Zero(state_dim);
    current_state = VectorXd::Zero(state_dim);

    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    // auto callback_angle= [this](const rclcpp::Parameter &p) {
    //   desired_angle = p.as_double();
    // };

   auto callback_lambda = [this](const rclcpp::Parameter &p) {
      lambda = p.as_double();
      controller_.update_forgetting_factor(lambda);

    };
   auto callback_bound = [this](const rclcpp::Parameter &p) {
      input_bound = p.as_double();
      VectorXd bounds = VectorXd::Ones(1) * input_bound;
      controller_.set_bounds(bounds);
    };

    // angle_handle_ = param_subscriber_->add_parameter_callback("desired_angle", callback_angle);
    lambda_handle_ = param_subscriber_->add_parameter_callback("lambda",  callback_lambda);
    bound_handle_ = param_subscriber_->add_parameter_callback("u_bound", callback_bound);

    auto sensor_qos = rclcpp::QoS(2).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", sensor_qos, std::bind(&PendulumControlNode::read, this, std::placeholders::_1));

    this->goal_sub_ = this->create_subscription<Float64>(
      "/desired_angle",
      10,
      [&](const Float64::SharedPtr msg){
        desired_angle = msg->data;
      }
    );

    // auto control_qos = rclcpp::QoS(5).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pendulum/commands", 10);
    params_pub_ = this->create_publisher<Params>("/params", 10);
    params_timer_ = this->create_wall_timer(10ms, std::bind(&PendulumControlNode::feedback, this));
    RCLCPP_INFO(this->get_logger(), "Self Tuning Regulator started!");
  }

private:
  uint BUFFER_SIZE = 4;
  void read(const sensor_msgs::msg::JointState::SharedPtr msg){

    // double current_angle = wrap(msg->position[0]);
    double current_angle = msg->position[0];
    double current_torque = msg->effort[0];

    angles.push_back(current_angle);
    torques.push_back(current_torque);

    while (angles.size() > BUFFER_SIZE){
      angles.pop_front();
    }
    while (torques.size() > BUFFER_SIZE){
      torques.pop_front();
    }

    control();
  }

  void control(){
    int step = std::min(angles.size(), torques.size())-1;
    if (step > 2){

      p_states << angles[step-1], angles[step-2];
      p_inputs << torques[step-1], torques[step-2];
      double current = angles[step];

      auto input = controller_.str(desired_angle, current, p_states, p_inputs);
      publish_torque(input);
    }
  }

  void publish_torque(double input){
      auto control_msg = std_msgs::msg::Float64MultiArray();
      control_msg.data = {input};
      torque_pub_->publish(control_msg);
  }

  void feedback(){

    MatrixXd Theta = controller_.get_parameters();
    MatrixXd Cov = controller_.get_covariance();

    auto desired_state = VectorXd::Ones(1) * desired_angle;
    auto current_state = VectorXd::Ones(1) * angles.back();
    auto Errors = controller_.get_error(desired_state, current_state);

    auto msg = Params();

    msg.estimate.resize(Theta.cols() * Theta.rows());
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

  rclcpp::Subscription<Float64>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::Publisher<Params>::SharedPtr params_pub_;
  rclcpp::TimerBase::SharedPtr params_timer_;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> lambda_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> angle_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> bound_handle_;

  SelfTuningRegulator controller_;
  VectorXd desired_state, current_state;
  VectorXd p_states;
  VectorXd p_inputs;
  VectorXd process_errors;
  std::deque<double> angles;
  std::deque<double> torques;
  double input_bound, lambda, desired_angle;

}; // PendulumControlNode

} // pendulum_action

RCLCPP_COMPONENTS_REGISTER_NODE(pendulum_action::PendulumControlNode)
