#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node {

public:

  struct Pose2D {
    double x_, y_, yaw_;
    double linear_vel_, angular_vel_;

    // Default constructor
    Pose2D(){}

    // Construct from topics
    Pose2D(const nav_msgs::msg::Odometry::SharedPtr odom){

      x_ = odom->pose.pose.position.x; y_ = odom->pose.pose.position.y;

      Eigen::Quaternionf q(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
      Eigen::Vector3f rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
      yaw_ = rpy[2];

      linear_vel_ = odom->twist.twist.linear.x;
      angular_vel_ = odom->twist.twist.angular.z;
    }

    Pose2D(const geometry_msgs::msg::PoseStamped::SharedPtr pose) : linear_vel_(0.0), angular_vel_(0.0){

      x_ = pose->pose.position.x; y_ = pose->pose.position.y;
      Eigen::Quaternionf q(pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z);
      Eigen::Vector3f rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
      yaw_ = rpy[2];
    }
  };

  ControlNode() : Node("control_node"), kp_linear(1.0), kd_linear(0.1), ki_linear(0.0),
                  kp_angular(2.0), kd_angular(0.2), ki_angular(0.0),
                  kp_lateral(1.5), received_path_(false), received_goal_(false), received_odom_(false) {

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&ControlNode::goalCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/planned_path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/diff_drive_controller/cmd_vel", 10);
    control_timer_ = this->create_wall_timer(50ms, std::bind(&ControlNode::computeControl, this));
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){

    // goal_pose_ = *msg;
    Pose2D pose(msg);
    goal_= pose;
    received_goal_ = true;

  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){

    Pose2D pose(msg);
    current_ = pose;
    received_odom_ = true;

  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {

    path_ = *msg;
    received_path_ = true;
  }

  void computeControl(){

    if (path_.poses.empty() || !received_odom_) {
      // Publish 0 velocity to stop
      geometry_msgs::msg::TwistStamped stop_msg;
      stop_msg.header.frame_id = "base_footprint";
      stop_msg.header.stamp = this->now();
      stop_msg.twist.linear.x = 0.0;
      stop_msg.twist.angular.z = 0.0;
      vel_pub_->publish(stop_msg);
      return;
    }
    // Check if we've reached the goal (within threshold of last waypoint)
    double goal_threshold = 0.3; // 30cm threshold
    size_t last_waypoint = path_.poses.size() - 1;
    double dx_goal = path_.poses[last_waypoint].pose.position.x - current_.x_;
    double dy_goal = path_.poses[last_waypoint].pose.position.y - current_.y_;
    double distance_to_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);

    if (distance_to_goal < goal_threshold) {
        // Goal reached - stop the robot
        geometry_msgs::msg::TwistStamped stop_msg;
        stop_msg.header.frame_id = "base_footprint";
        stop_msg.header.stamp = this->now();
        stop_msg.twist.linear.x = 0.0;
        stop_msg.twist.angular.z = 0.0;
        vel_pub_->publish(stop_msg);
        RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping robot.");
        return;
    }

    // Get the lookahead point instead of closest point
    double lookahead_distance = 1.0; // 1 meter lookahead
    size_t target = lookAheadPoint(current_, path_, lookahead_distance);

    // Calculate desired heading to target point (Pure Pursuit)
    double dx_target = path_.poses[target].pose.position.x - current_.x_;
    double dy_target = path_.poses[target].pose.position.y - current_.y_;
    double desired_heading = std::atan2(dy_target, dx_target);

    double heading_error = desired_heading - current_.yaw_;
    heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));

    // Calculate cross-track error for lateral control
    double cross_track_err = crossTrackError(current_, path_);

    // Combined control: heading + lateral correction
    double angular_vel = kp_angular * heading_error + kp_lateral * cross_track_err;

    // Slow down as we approach the goal
    double linear_vel = 0.5;
    if (distance_to_goal < 1.0) {  // Start slowing down within 1m of goal
        linear_vel = 0.2 + 0.3 * (distance_to_goal / 1.0);  // Scale from 0.2 to 0.5 m/s
    }

    // Publish velocity command
    geometry_msgs::msg::TwistStamped msg;
    msg.header.frame_id = "base_footprint";
    msg.header.stamp = this->now();
    msg.twist.linear.x = linear_vel;
    msg.twist.angular.z = angular_vel;
    vel_pub_->publish(msg);
  }

  size_t closestPoint(Pose2D current, nav_msgs::msg::Path path){

    size_t closest_ind = 0;
    double closest_distance = std::numeric_limits<double>::max();

    for(size_t i=0; i < path.poses.size(); i++){

      double dx = path.poses[i].pose.position.x - current.x_;
      double dy = path.poses[i].pose.position.y - current.y_;
      double distance = std::sqrt(dx*dx + dy*dy);

      if (distance < closest_distance){
        closest_distance = distance;
        closest_ind = i;
      }
    }
    // Return the closest waypoint to current's indice in the path
    return closest_ind;
  }

  size_t lookAheadPoint(Pose2D current, nav_msgs::msg::Path path, double lookahead_distance) {
    size_t closest = closestPoint(current, path);

    // Start from closest point and find first point beyond lookahead distance
    for (size_t i = closest; i < path.poses.size(); i++) {
      double dx = path.poses[i].pose.position.x - current.x_;
      double dy = path.poses[i].pose.position.y - current.y_;
      double distance = std::sqrt(dx*dx + dy*dy);

      if (distance >= lookahead_distance) {
        return i;
      }
    }

    // If no point is far enough, return last point
    return path.poses.size() - 1;
  }

  double crossTrackError(Pose2D robot, nav_msgs::msg::Path path) {
    if (path.poses.size() < 2) return 0.0;

    size_t closest = closestPoint(robot, path);

    // Get the path segment to calculate cross-track error against
    size_t segment_start = closest;
    size_t segment_end = closest + 1;

    // Handle edge case: if at last waypoint, use previous segment
    if (closest >= path.poses.size() - 1) {
      segment_start = path.poses.size() - 2;
      segment_end = path.poses.size() - 1;
    }

    // Path segment vector (from start to end)
    double dx_path = path.poses[segment_end].pose.position.x - path.poses[segment_start].pose.position.x;
    double dy_path = path.poses[segment_end].pose.position.y - path.poses[segment_start].pose.position.y;
    double path_length = std::sqrt(dx_path * dx_path + dy_path * dy_path);

    if (path_length < 1e-6) return 0.0; // Avoid division by zero

    // Robot position relative to segment start
    double dx_robot = robot.x_ - path.poses[segment_start].pose.position.x;
    double dy_robot = robot.y_ - path.poses[segment_start].pose.position.y;

    // Cross product gives signed perpendicular distance
    // Positive = robot is to the left of path direction
    // Negative = robot is to the right of path direction
    return (dx_path * dy_robot - dy_path * dx_robot) / path_length;
  }

private:

  // Control Variables
  double kp_linear, kd_linear, ki_linear;
  double kp_angular, kd_angular, ki_angular;
  double kp_lateral;




  // Member variables
  Pose2D current_;
  Pose2D goal_;

  geometry_msgs::msg::PoseStamped curr_pose_;
  geometry_msgs::msg::PoseStamped goal_pose__;
  nav_msgs::msg::Path path_;

  bool received_path_;
  bool received_goal_;
  bool received_odom_;

  // ROS interface
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char* argv[]){

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
