#ifndef TRAJECTORY_INTERPOLATOR_HPP
#define TRAJECTORY_INTERPOLATOR_HPP
#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <vector>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
namespace rover_utils
{
template<int DIM>
class TrajectoryInterpolator
{
public:
  using State = Eigen::Matrix<double, DIM, 1>;
  using StateDer = Eigen::Matrix<double, DIM, 1>;
  struct Waypoint{
    State position;
    State velocity;
    State acceleration;
    rclcpp::Time timestamp;

    Waypoint()
    {
      position.setZero();
      velocity.setZero();
      acceleration.setZero();
    }
  };

  TrajectoryInterpolator() : initialized(false), current_index_(0){}
  ~TrajectoryInterpolator() = default;
  /**
   * @brief: initialize a trajectory from a list of desired robot pose's, with desired
   * trajectory sampling time.
   * @params: waypoints: set of desired pose's for the robot's state, dt: requested sampling time
   * of the trajectory, starting_time, initial ROS timestamp at trajectory generation
   * @output: True if passed parameters can be used to create a valid trajectory,
   * false otherwise
   */
  bool init_trajectory(const std::vector<State>& waypoints,
                       double dt,
                       const rclcpp::Time& starting_time)
  {
    if(waypoints.empty()){
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryInterpolator"), "Empty trajectory received");
      return false;
    }
    this->buffer_.clear();
    this->start_time_ = starting_time;
    this->current_index_ = 0;

    for(size_t i=0; i<waypoints.size(); ++i){
      Waypoint w;
      w.position = waypoints[i];
      w.velocity.setZero();
      w.acceleration.setZero();
      w.timestamp = start_time_ + rclcpp::Duration::from_seconds(i*dt);

      buffer_.push_back(w);
    }
    initialized = true;
    RCLCPP_INFO(rclcpp::get_logger("TrajectoryInterpolator"), "Loaded trajectory with %zu waypoints", buffer_.size());
    return true;
  }

  /**
   * @brief: get desired state at current time from trajectory
   * @params: current_time, internal clock of the ROS executor
   * @output: State desired for the controller
   */
  State get_desired(const rclcpp::Time& current_time)
  {
    if(!initialized || buffer_.empty()){
      State zero;
      zero.setZero();
      return zero;
    }
    update_index(current_time);

    if(current_time <= buffer_.front().timestamp){
      return buffer_.front().position;
    } else if(current_time >= buffer_.back().timestamp){
      return buffer_.back().position;
    }

    const auto& w1 = buffer_[current_index_];
    const auto& w2 = buffer_[current_index_+1];

    double segment_time = (w2.timestamp-w1.timestamp).seconds();
    double elapsed_time = (current_time - w1.timestamp).seconds();
    double alpha = elapsed_time / segment_time;

    return linearInterpolate(w1, w2, alpha);
  }

  /**
   * @brief: query whether the planned trajectory has elapsed at time
   * @params: current_time, internal clock of the ROS executor
   * @output: true if trajectory is finished, false otherwise
   */
  bool is_finished(const rclcpp::Time& current_time) const
  {
    if(!initialized || buffer_.empty()
      || current_time >= buffer_.back().timestamp){
      return true;
    }
    return false;
  }

  /**
   * @brief: access the index of the current waypoint in the trajectory
   * @params: -
   * @output: return the current index from the buffer
   */
  size_t get_current_index() const {return current_index_;}

  /**
   * @brief: Access the total size of the buffer
   * @params: -
   * @output: number of waypoints in the buffer
   */
  size_t get_buffer_size() const {return buffer_.size();}

  /**
   * @brief: Return the amount of time left for trajectory to be completed
   * @params: current time of execution
   * @output: remaining time in seconds
   */
  double get_remaining_time(const rclcpp::Time& current_time) const
  {
    if(!initialized || buffer_.empty())
    {
      return 0.0;
    }
    return (buffer_.back().timestamp - current_time).seconds();
  }

  /**
   * @brief: Access the interpolated waypoints positions for visualization
   * @params: -
   * @output: a vector of positions the interpolater calculated
   */
  std::vector<State> get_waypoints() const
  {
    std::vector<State> positions;
    for(const auto& w : buffer_)
    {
      positions.push_back(w.position);
    }
    return positions;
  }

private:

  std::deque<Waypoint> buffer_;
  rclcpp::Time start_time_;
  size_t current_index_;
  bool initialized;

  /**
   * @brief:
   * @params:
   * @output:
   */
  State linearInterpolate(const Waypoint& w1, const Waypoint& w2, double alpha)
  {
    return (1.0 - alpha)*w1.position + alpha * w2.position;
  }

  /**
   * @brief: Move the buffer index to the next position if not at the end, or the timestamp has elapsed
   * @params: current_time of the execution
   * @output: -
   */
  void update_index(const rclcpp::Time& current_time)
  {
    while(current_index_ < buffer_.size()-1 && current_time >= buffer_[current_index_+1].timestamp)
    {
      current_index_++;
    }
  }
}; // class TrajectoryInterpolator

using PendulumInterpolator = TrajectoryInterpolator<1>;  // Single joint
using DoublePendulumInterpolator = TrajectoryInterpolator<2>;  // Two joints
using RoverInterpolator = TrajectoryInterpolator<3>;  // x,y,theta
using QuadcopterInterpolator = TrajectoryInterpolator<12>;  // x,y,z,roll,pitch,yaw + derivatives

} // namespace rover_utils
#endif
