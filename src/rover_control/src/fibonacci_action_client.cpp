#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "rover_msgs/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace rover_action
{
class FibonacciActionClient : public rclcpp::Node
{
public:
  using Fibonacci = rover_msgs::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit FibonacciActionClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("fibonacci_action_server", options)
  {
    this->action_client_ = rclcpp_action::create_client<Fibonacci>(
      this,
      "fibonacci"
    );

    auto timer_callback = [this](){
      return this->send_goal();
    };

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      timer_callback
    );
  }

private:

  void send_goal(){
    using namespace std::placeholders;
    this->timer_->cancel();

    if(!this->action_client_->wait_for_action_server(std::chrono::seconds(10))){
      RCLCPP_ERROR(this->get_logger(),"Action server not available");
      rclcpp::shutdown();
    }
    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const GoalHandleFibonacci::SharedPtr & goal_handle)
    {
      if(!goal_handle){
        RCLCPP_ERROR(this->get_logger(),"Goal rejected by server");
      } else{
        RCLCPP_INFO(this->get_logger(),"Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback = [this](GoalHandleFibonacci::SharedPtr,
                                                 const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
      std::stringstream ss;
      ss << "Next number in sequence received: ";
      for(auto number : feedback->partial_sequence){
          ss << number << " ";
      }

      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    };

    send_goal_options.result_callback = [this](const GoalHandleFibonacci::WrappedResult& result)
    {
      switch(result.code){
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown Result code");
          return;
      }
      std::stringstream ss;
      ss << "Result received: ";
      for(auto number : result.result->sequence){
          ss << number << " ";
      }
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
      rclcpp::shutdown();
    };
    this->action_client_->async_send_goal(goal_msg, send_goal_options);

  }
  rclcpp_action::Client<Fibonacci>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;

};// FibonacciActionClient

} // rover_action

RCLCPP_COMPONENTS_REGISTER_NODE(rover_action::FibonacciActionClient)
