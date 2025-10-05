#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "rover_msgs/action/pendulum_trajectory.hpp"
#include "rover_msgs/msg/params.hpp"

#include <functional>
#include <future>
#include <memory>
#include <sstream>
using namespace std::chrono_literals;

namespace pendulum_action
{
class PendulumActionClient : public rclcpp::Node
{

public:
  using Trajectory = rover_msgs::action::PendulumTrajectory;
  using Params = rover_msgs::msg::Params;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Trajectory>;

  explicit PendulumActionClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("pendulum_action_client", options)
  {
    this->action_client_ = rclcpp_action::create_client<Trajectory>(this, "pendulum_trajectory");

    auto timer_callback = [this](){
      return this->send_goal();
    };

    this->timer_ = this->create_wall_timer(500ms, timer_callback);
  }
  void send_goal(){

    auto logger = this->get_logger();

    this->timer_->cancel();
    if(!this->action_client_->wait_for_action_server(std::chrono::seconds(10))){
      RCLCPP_ERROR(logger,"Action server not available");
      rclcpp::shutdown();
    }

    auto goal_msg = Trajectory::Goal();
    // goal_msg.trajectory = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.14};
    goal_msg.trajectory = {0.0, 0.5, 1.57, 3.14, 2.57, 3.7, 0.5, 5.5};
    goal_msg.dt = 2.0;
    RCLCPP_INFO(logger, "Sending goal trajectory to pendulum");

    auto options = rclcpp_action::Client<Trajectory>::SendGoalOptions();
    options.goal_response_callback = [this, logger](const GoalHandle::SharedPtr& goal_handle)
    {
      if(!goal_handle){
        RCLCPP_ERROR(logger,"Goal rejected by server");
      } else{
        RCLCPP_INFO(logger,"Goal accepted by server, waiting for result");
      }
    };

    options.feedback_callback = [this, logger](GoalHandle::SharedPtr, const std::shared_ptr<const Trajectory::Feedback> feedback)
    {
        auto at_point = feedback->current_waypoint;
        auto curr_params = feedback->current_params;
        std::stringstream errors;
        errors << "Tracking error's at each time stamp: ";
        for(auto error : feedback->tracking_error){
          errors << error << " ";
        }

        RCLCPP_INFO(logger,"At current waypoint %i", at_point);
        // RCLCPP_INFO(logger, errors.str().c_str());
    };

    options.result_callback = [this,logger](const GoalHandle::WrappedResult& result)
    {
        switch(result.code){
          case rclcpp_action::ResultCode::SUCCEEDED:
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(logger, "Goal was aborted");
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(logger, "Goal was aborted");
            return;
          default:
            RCLCPP_ERROR(logger, "Unknown Result code");
            return;
      }

        auto is_successfull = result.result->success;
        auto final_params = result.result->final_params;
        auto message = result.result->message;
        // rclcpp::shutdown();
    };

    this->action_client_->async_send_goal(goal_msg, options);
  }

private:
  rclcpp_action::Client<Trajectory>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;

}; // PendulumActionClient
}  // pendulum_action

RCLCPP_COMPONENTS_REGISTER_NODE(pendulum_action::PendulumActionClient)
