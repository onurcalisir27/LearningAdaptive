#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "rover_msgs/msg/str_params.hpp"
#include "rover_msgs/action/pendulum_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "rover_utils/trajectory_interpolator.hpp"

#include <functional>
#include <memory>
#include <thread>
#include <vector>

namespace pendulum_action
{
class PendulumActionServer : public rclcpp::Node
{
public:
  using Trajectory = rover_msgs::action::PendulumTrajectory;
  using Params = rover_msgs::msg::StrParams;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Trajectory>;
  using JointStates = sensor_msgs::msg::JointState;
  using FloatMultiArray = std_msgs::msg::Float64MultiArray;
  using Float64 = std_msgs::msg::Float64;

  explicit PendulumActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("pendulum_action_server", options)
  {

    auto logger = this->get_logger();
    auto handle_goal = [logger](const rclcpp_action::GoalUUID& uuid,
                                     std::shared_ptr<const Trajectory::Goal> goal)
      {

        RCLCPP_INFO(logger, "Received goal trajectory, starting...");
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      };

    auto handle_cancel = [logger](const std::shared_ptr<GoalHandle> goal_handle)
      {
        RCLCPP_INFO(logger, "Received request to cancel action");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
      };

    auto handle_accepted = [this](const std::shared_ptr<GoalHandle> goal_handle)
      {
        auto offload_to_thread =  [this, goal_handle]()
          {
            return this->drive_pendulum(goal_handle);
          };
        std::thread{offload_to_thread}.detach();
      };

    this->action_server_ = rclcpp_action::create_server<Trajectory>(
      this,
      "pendulum_trajectory",
      handle_goal,
      handle_cancel,
      handle_accepted
    );

    this->params_sub_ = this->create_subscription<Params>(
        "/params", 10,
        std::bind(&PendulumActionServer::params_callback, this, std::placeholders::_1));

    this->joint_state_subscriber_ = this->create_subscription<JointStates>(
        "/joint_states", 10,
        std::bind(&PendulumActionServer::joints_callback, this, std::placeholders::_1));

    this->goal_pub_ = this->create_publisher<Float64>("/desired_angle", 10);
  }

private:
  rclcpp_action::Server<Trajectory>::SharedPtr action_server_;
  rclcpp::Subscription<Params>::SharedPtr params_sub_;
  rclcpp::Subscription<JointStates>::SharedPtr joint_state_subscriber_;
  rclcpp::Publisher<Float64>::SharedPtr goal_pub_;
  std::mutex data_mutex_;
  std::vector<Params> parameters_;
  Params current_params_, final_params_;
  double current_angle_, current_velocity_;

  void drive_pendulum(const std::shared_ptr<GoalHandle> goal_handle)
  {
    using Interpolator = rover_utils::PendulumInterpolator;
    Interpolator interpolator;
    auto goal = goal_handle->get_goal();
    std::vector<Interpolator::State> waypoints;
    for(double angle : goal->trajectory){
      Interpolator::State state;
      state << angle;
      waypoints.push_back(state);
    }
    interpolator.init_trajectory(waypoints, goal->dt, this->now());
    rclcpp::Rate loop_rate(1000);
    RCLCPP_INFO(this->get_logger(), "Starting Execution of Goal Trajectory");

    auto feedback = std::make_shared<Trajectory::Feedback>();
    auto result = std::make_shared<Trajectory::Result>();

    while(rclcpp::ok() && !interpolator.is_finished(this->now()))
    {

      if(goal_handle->is_canceling())
      {
        result->success = false;
        result->message = "Action Cancelled";
        goal_handle->canceled(result);
        return;
      }

      auto current_time = this->now();
      auto desired_position = interpolator.get_desired(current_time);
      auto desired_vel = 0.0;

      auto msg = Float64();
      msg.data = desired_position(0);
      goal_pub_->publish(msg);

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        feedback->current_waypoint = interpolator.get_current_index();
        feedback->tracking_error.push_back(desired_position(0) - current_angle_);
        feedback->current_params = current_params_;
      }

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                          1000,
                          "Time remaining: %.1fs",
                          interpolator.get_remaining_time(current_time));
     goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        result->success = true;
        result->message = "Trajectory completed";
        result->final_params = current_params_;
    }
    auto last_msg = Float64();
    last_msg.data = current_angle_;
    goal_pub_->publish(last_msg);
    goal_handle->succeed(result);

  }

  void params_callback(const Params::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_params_ = *msg;
  }

  void joints_callback(const JointStates::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_angle_ = msg->position[0];
    current_velocity_ = msg->velocity[0];
  }

}; // PendulumActionServer
} // pendulum_action

RCLCPP_COMPONENTS_REGISTER_NODE(pendulum_action::PendulumActionServer)
