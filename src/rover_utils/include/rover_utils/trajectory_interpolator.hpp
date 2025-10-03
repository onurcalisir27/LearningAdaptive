#ifndef TRAJECTORY_INTERPOLATOR_HPP
#define TRAJECTORY_INTERPOLATOR_HPP
#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <vector>
#include <Eigen/Dense>

namespace rover_utils
{
template<int DIM>
class TrajectoryInterpolator
{
public:
  using State = Eigen::Matrix<double, DIM, 1>;
  struct Waypoint{
    State pose;
    State velocity;
    rclcpp::Time timestamp;
  };

  explicit TrajectoryInterpolator();

  /**
   * @brief: initialize a trajectory from a list of desired robot pose's, with desired
   * trajectory sampling time.
   * @params: waypoints: set of desired pose's for the robot's state, dt: requested sampling time
   * of the trajectory, starting_time, initial ROS timestamp at trajectory generation
   * @output: True if passed parameters can be used to create a valid trajectory,
   * false otherwise
   */
  bool init_trajectory(
    const std::vector<State>& waypoints,
    double dt,
    const rclcpp::Time& starting_time);

  /**
   * @brief: get desired state at current time from trajectory
   * @params: current_time, internal clock of the ROS executor
   * @output: State desired for the controller
   */
  State get_desired(const rclcpp::Time& current_time);

  /**
   * @brief: query whether the planned trajectory has elapsed at time
   * @params: current_time, internal clock of the ROS executor
   * @output: true if trajectory is finished, false otherwise
   */
  bool is_finished(const rclcpp::Time& current_time) const;

  /**
   * @brief: access the index of the current waypoint in the trajectory
   * @params: -
   * @output: return the current index from the buffer
   */
  size_t get_current_index() const{return current_index_;}

private:
  std::deque<Waypoint> buffer_;
  size_t current_index_;

  State linearInterpolate(const Waypoint& w1, const Waypoint& w2, double alpha);
}; // class TrajectoryInterpolator

using PendulumInterpolator = TrajectoryInterpolator<1>;  // Single joint
using DoublePendulumInterpolator = TrajectoryInterpolator<2>;  // Two joints
using CarInterpolator = TrajectoryInterpolator<3>;  // x,y,theta
using QuadcopterInterpolator = TrajectoryInterpolator<12>;  // x,y,z,roll,pitch,yaw + derivatives

} // namespace rover_utils
#endif
