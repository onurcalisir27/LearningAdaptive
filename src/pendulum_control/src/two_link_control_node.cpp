#include "rclcpp/rclcpp.hpp"
#include "rover_utils/self_tuning_regulator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rover_msgs/msg/str_params.hpp"
#include <cmath>
#include <memory>
#include <Eigen/Dense>
#include <deque>
#include <unordered_map>
#include <tuple>

using namespace std::chrono_literals;
using rover_utils::SelfTuningRegulator;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class TwoLinkControlNode : public rclcpp::Node
{
public:
  explicit TwoLinkControlNode() : Node("two_link_control_node"){

    this->declare_parameter("lambda", 0.98);
    this->get_parameter("lambda", lambda);

    this->declare_parameter("desired_angle", 0.0);
    this->get_parameter("desired_angle", desired_angle);

    this->declare_parameter("u1_bound", 6.0);
    this->get_parameter("u1_bound", input1_bound);

    this->declare_parameter("u2_bound", 3.0);
    this->get_parameter("u2_bound", input2_bound);

    int state_history = 2;
    int state_dim = 2;
    int input_history = 2;
    int input_dim = 2;
    double covariance = 1e6;
    controller_.init(state_dim, input_dim, state_history, input_history, lambda);
    RCLCPP_INFO(this->get_logger(), "Self Tuning Regulator Initialized!");

    bounds = VectorXd::Zero(input_dim);
    bounds << input1_bound, input2_bound;
    controller_.set_bounds(bounds);
    controller_.set_covariance(covariance);

    p_inputs = VectorXd::Zero(input_history*input_dim);
    p_states = VectorXd::Zero(state_history*state_dim);

    desired_state = VectorXd::Zero(state_dim);
    current_state = VectorXd::Zero(state_dim);

    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto callback_angle= [this](const rclcpp::Parameter &p) {
      desired_angle = p.as_double();
    };
   auto callback_lambda = [this](const rclcpp::Parameter &p) {
      lambda = p.as_double();
      controller_.update_forgetting_factor(lambda);
    };
   auto callback_bound1 = [this](const rclcpp::Parameter &p) {
      input1_bound = p.as_double();
      bounds << input1_bound, input2_bound;
      controller_.set_bounds(bounds);
    };
    auto callback_bound2 = [this](const rclcpp::Parameter &p) {
      input2_bound = p.as_double();
      bounds << input1_bound, input2_bound;
      controller_.set_bounds(bounds);
    };

    angle_handle_ = param_subscriber_->add_parameter_callback("desired_angle", callback_angle);
    lambda_handle_ = param_subscriber_->add_parameter_callback("lambda",  callback_lambda);
    bound1_handle_ = param_subscriber_->add_parameter_callback("u1_bound", callback_bound1);
    bound2_handle_ = param_subscriber_->add_parameter_callback("u2_bound", callback_bound2);

    auto sensor_qos = rclcpp::QoS(2).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", sensor_qos, std::bind(&TwoLinkControlNode::read, this, std::placeholders::_1));

    // auto control_qos = rclcpp::QoS(5).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pendulum/commands", 10);
    params_pub_ = this->create_publisher<rover_msgs::msg::StrParams>("params", 10);
    params_timer_ = this->create_wall_timer(20ms, std::bind(&TwoLinkControlNode::feedback, this));
    RCLCPP_INFO(this->get_logger(), "Self Tuning Regulator started!");

    this->joint_map.insert({this->joint1_name, 0});
    this->joint_map.insert({this->joint2_name, 1});
  }

private:
  const size_t MAX_HISTORY = 6;
  void read(const sensor_msgs::msg::JointState::SharedPtr msg){

    double angle1, angle2, torque1, torque2;
    for (auto joint : msg->name){
      auto it = joint_map.find(joint);
      if (it->first == joint1_name){
        angle1 = wrap(msg->position[it->second]);
        torque1 = msg->effort[it->second];
      }
      else if (it->first == joint2_name){
        angle2 = wrap(msg->position[it->second]);
        torque2 = msg->effort[it->second];
      }
      else {
        RCLCPP_INFO(this->get_logger(), "Joint Information Not Found");
        return;
      }
    }
    angles.push_back(angle2);
    angles.push_back(angle1);
    torques.push_back(torque2);
    torques.push_back(torque1);
    // angles = [angle2(t-2), angle1(t-2), angle2(t-1), angle1(t-1), angle2(t), angle1(t)]

    while (angles.size() > MAX_HISTORY) {
        angles.pop_front();
    }
    while (torques.size() > MAX_HISTORY) {
        torques.pop_front();
    }

    control();
  }

  void control(){

    if (angles.size() < 6 || torques.size() < 6){
        return;  // Not enough data
    }

    int step = angles.size() - 1;
    // p_states = [angle1(t-1), angle2(t-1), angle1(t-2), angle2(t-2)]
    p_states << angles[step-2], angles[step-3], angles[step-4], angles[step-5];
    p_inputs << torques[step-2], torques[step-3], torques[step-4], torques[step-5];

    desired_state << desired_angle, 0.0;
    // current_state = [angle1(t), angle2(t)]
    current_state << angles[step], angles[step-1];

    auto input = controller_.compute_input(desired_state, current_state, p_states, p_inputs);
    publish_torque(input);

  }

  void publish_torque(VectorXd input){

    if(input.size() != 2){
        RCLCPP_ERROR(this->get_logger(), "Expected 2 joints, got %f", static_cast<float>(input.size()));
        return;
    }
    auto control_msg = std_msgs::msg::Float64MultiArray();
    control_msg.data = {input(0), input(1)};

    torque_pub_->publish(control_msg);
  }

  void feedback(){

    auto Theta = controller_.get_parameters();
    auto Cov = controller_.get_covariance();
    auto Errors = controller_.get_error(desired_state, current_state);
    //
    // VectorXd state_error;
    // VectorXd estimate_error;
    // VectorXd control_error;
    // std::tie(state_error, estimate_error, control_error) = Errors;
    //
    auto msg = rover_msgs::msg::StrParams();

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

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::Publisher<rover_msgs::msg::StrParams>::SharedPtr params_pub_;
  rclcpp::TimerBase::SharedPtr params_timer_;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> lambda_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> angle_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> bound1_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> bound2_handle_;

  SelfTuningRegulator controller_;

  VectorXd desired_state, current_state;

  VectorXd p_states, p_inputs;

  VectorXd process_errors;

  std::deque<double> angles, torques;

  std::string joint1_name = "pendulum_joint1";
  std::string joint2_name = "pendulum_joint2";
  std::unordered_map<std::string, size_t> joint_map;

  VectorXd bounds;
  double input1_bound, input2_bound, lambda, desired_angle;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwoLinkControlNode>());
  rclcpp::shutdown();
  return 0;
}
