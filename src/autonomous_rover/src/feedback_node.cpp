// feedback_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <memory>
#include <chrono>
#include <cmath>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

class FeedbackCalibration : public rclcpp::Node {
public:
  FeedbackCalibration() : Node("feedback_node") {
    wheel_radius_ = 0.0325;
    positions_.resize(4, 0.0f);
    velocities_.resize(4, 0.0f);

    umap_={
      {"front_left_wheel_joint", 0},
      {"back_left_wheel_joint", 1},
      {"front_right_wheel_joint", 2},
      {"back_right_wheel_joint", 3},
    };

    encoder_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&FeedbackCalibration::encoder_callback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10, std::bind(&FeedbackCalibration::odom_callbak, this, std::placeholders::_1));

    publish_timer_ = this->create_wall_timer(100ms, std::bind(&FeedbackCalibration::message_publisher, this));
  }

private:

  void encoder_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {

    for (size_t i=0; i < msg->name.size();++i){

      auto it = umap_.find(msg->name[i]);

      if (it != umap_.end()){
        size_t encoder_index = it->second;

        positions_[encoder_index] = msg->position[i] * wheel_radius_;
        velocities_[encoder_index] = msg->velocity[i] * wheel_radius_;

      } else{
          RCLCPP_WARN(this->get_logger(), "This encoder does not exist, check the map keys");
          return;
      }
    }
  }

  void odom_callbak(const nav_msgs::msg::Odometry::SharedPtr msg) {

    robot_x_.push_back(msg->pose.pose.position.x);
    robot_y_.push_back(msg->pose.pose.position.y);
    robot_z_.push_back(msg->pose.pose.position.z);
    
    Eigen::Quaternionf q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
      msg->pose.pose.orientation.z); // ( w, x, y, z )
    
    Eigen::Vector3f rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
    robot_yaw_.push_back(rpy[2] * 180.0 / M_PI);

    linear_vel_.push_back(msg->twist.twist.linear.x);
    angular_vel_.push_back(msg->twist.twist.angular.z);
  }

  void message_publisher(){

    system("clear");
    RCLCPP_INFO(this->get_logger(), "---------Encoder Feedback--------");
    RCLCPP_INFO(this->get_logger(), "Left Encoder:");
    RCLCPP_INFO(this->get_logger(), "Position: %f m", (positions_[0] + positions_[1]) / 2);
    RCLCPP_INFO(this->get_logger(), "Velocities: %f m/s", (velocities_[0] + velocities_[1]) / 2);
    
    RCLCPP_INFO(this->get_logger(), "---------------------------------");
    RCLCPP_INFO(this->get_logger(), "Right Encoder:");
    RCLCPP_INFO(this->get_logger(), "Position: %f m", (positions_[2] + positions_[3]) / 2 );
    RCLCPP_INFO(this->get_logger(), "Velocities: %f m/s", (velocities_[2] + velocities_[3]) / 2);

    RCLCPP_INFO(this->get_logger(), "---------------------------------");
    RCLCPP_INFO(this->get_logger(), "Encoder Error (L-R):");
    RCLCPP_INFO(this->get_logger(), "Position: %f m",((positions_[0] + positions_[1]) / 2) - ((positions_[2] + positions_[3])/2));
    RCLCPP_INFO(this->get_logger(), "Velocities: %f m/s",((velocities_[0] + velocities_[1]) / 2) - ((velocities_[2] + velocities_[3])/2));

    RCLCPP_INFO(this->get_logger(), "---------------------------------");
    RCLCPP_INFO(this->get_logger(), "------EKF Odometry Feedback------");
    RCLCPP_INFO(this->get_logger(), "Robot Position:");
    RCLCPP_INFO(this->get_logger(), "X: %d m ", robot_x_.back());
    RCLCPP_INFO(this->get_logger(), "Y: %d m/s", robot_y_.back());
    RCLCPP_INFO(this->get_logger(), "Yaw: %d ", robot_yaw_.back());

    RCLCPP_INFO(this->get_logger(), "---------------------------------");
    RCLCPP_INFO(this->get_logger(), "Robot Velocity:");
    RCLCPP_INFO(this->get_logger(), "Linear X: %d m ", linear_vel_.back());
    RCLCPP_INFO(this->get_logger(), "Angular Z: %d m/s", angular_vel_.back());

  }
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr encoder_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::unordered_map<std::string, size_t> umap_;
  std::vector<float> positions_;
  std::vector<float> velocities_;

  std::vector<double> robot_x_;
  std::vector<double> robot_y_;
  std::vector<double> robot_z_;
  std::vector<double> robot_yaw_;
  std::vector<double> linear_vel_;
  std::vector<double> angular_vel_;

  double wheel_radius_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FeedbackCalibration>());
  rclcpp::shutdown();
  return 0;
}
