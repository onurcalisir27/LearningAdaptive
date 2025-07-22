// encoder_calibration_node.cpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <vector>
#include <memory>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class EncoderCalibration : public rclcpp::Node {
public:
  EncoderCalibration() : Node("encoder_calibration_node") {

    // Initialize 
    wheel_radius_ = 0.0325;
    positions_.resize(4, 0.0f);
    velocities_.resize(4, 0.0f);

    umap_={
      {"front_left_wheel_joint", 0},
      {"back_left_wheel_joint", 1},
      {"front_right_wheel_joint", 2},
      {"back_right_wheel_joint", 3},
    };

    // Subscribers
    encoder_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&EncoderCalibration::encoder_callback, this, std::placeholders::_1));
    
    publish_timer_ = this->create_wall_timer(1000ms, std::bind(&EncoderCalibration::message_publisher, this));
  }

private:
  /* Joint State Message
  std_msgs/Header header

  string[] name
  float64[] position
  float64[] velocity
  float64[] effort*/

  void encoder_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {

    for (size_t i=0; i < msg->name.size();++i){

      // Find the index of the encoder from the map
      auto it = umap_.find(msg->name[i]);

      if (it != umap_.end()){
        // Append the encoder readings to the according joint
        size_t encoder_index = it->second;

        // Convert message content to meters
        positions_[encoder_index] = msg->position[i] * wheel_radius_;
        velocities_[encoder_index] = msg->velocity[i] * wheel_radius_;

      } else{
          RCLCPP_WARN(this->get_logger(), "This encoder does not exist, check the map keys");
          return;
      }
    }
  }

  void message_publisher(){

    system("clear");
    
    RCLCPP_INFO(this->get_logger(), "---------------------------------");
    RCLCPP_INFO(this->get_logger(), "FRONT LEFT ENCODER:");
    RCLCPP_INFO(this->get_logger(), "Position: %f m ", positions_[0]);
    RCLCPP_INFO(this->get_logger(), "Velocities: %f m/s", velocities_[0]);

    RCLCPP_INFO(this->get_logger(), "---------------------------------");

    RCLCPP_INFO(this->get_logger(), "BACK LEFT ENCODER:");
    RCLCPP_INFO(this->get_logger(), "Position: %f m", positions_[1]);
    RCLCPP_INFO(this->get_logger(), "Velocities: %f m/s", velocities_[1]);

    RCLCPP_INFO(this->get_logger(), "---------------------------------"); 

    RCLCPP_INFO(this->get_logger(), "FRONT RIGHT ENCODER:");
    RCLCPP_INFO(this->get_logger(), "Position: %f m", positions_[2]);
    RCLCPP_INFO(this->get_logger(), "Velocities: %f m/s", velocities_[2]);

    RCLCPP_INFO(this->get_logger(), "---------------------------------");

    RCLCPP_INFO(this->get_logger(), "BACK RIGHT ENCODER:");
    RCLCPP_INFO(this->get_logger(), "Position: %f m", positions_[3]);
    RCLCPP_INFO(this->get_logger(), "Velocities: %f m/s", velocities_[3]);
  }

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr encoder_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  
  // State variables
  std::unordered_map<std::string, size_t> umap_;
  std::vector<float> positions_;
  std::vector<float> velocities_;

  float wheel_radius_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderCalibration>());
  rclcpp::shutdown();
  return 0;
}