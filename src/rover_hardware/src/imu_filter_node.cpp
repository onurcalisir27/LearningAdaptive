#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <memory>

class ImuFilterNode : public rclcpp::Node {
public:
  ImuFilterNode() : Node("imu_filter_node"), mpu6050_filter_(80.0, 200.0), oak_imu_filter_(60.0, 250.0) {

    // Publishers
    mpu6050_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/mpu6050/filtered", 10);
    oak_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/oak/filtered", 10);

    // Subscribers
    mpu6050_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/mpu6050", 10, std::bind(&ImuFilterNode::mpu6050_callback, this, std::placeholders::_1));
    
    oak_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/oak", 10, std::bind(&ImuFilterNode::oak_imu_callback, this, std::placeholders::_1));
  }

private:

  void mpu6050_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    
    auto filtered = *msg;
    filtered.angular_velocity.x = mpu6050_filter_.filter(msg->angular_velocity.x);
    filtered.angular_velocity.y = mpu6050_filter_.filter(msg->angular_velocity.y);
    filtered.angular_velocity.z = mpu6050_filter_.filter(msg->angular_velocity.z);

    mpu6050_pub_->publish(filtered);
  }

  void oak_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
   
    auto filtered = *msg;
    // Also fix the tf mismatch here ->
    filtered.angular_velocity.x = oak_imu_filter_.filter(msg->angular_velocity.z);
    filtered.angular_velocity.y = oak_imu_filter_.filter(msg->angular_velocity.x);
    filtered.angular_velocity.z = oak_imu_filter_.filter(msg->angular_velocity.y);

    oak_imu_pub_->publish(filtered);
  }

  // Publishers for filtered IMU messages
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr mpu6050_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr oak_imu_pub_;

  // Subscribers to imu messages
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr mpu6050_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr oak_imu_sub_;

  // IMU Filter 
   struct Filter {
        double filter_variable_;
        double prev_output;
        bool init;
        
        Filter(double cutoff, double sample) : prev_output(0.0), init(false) {
            
            double RC = 1.0 / (2.0 * M_PI * cutoff);
            double dt = 1.0 / sample;
            filter_variable_ = dt / (RC + dt);
        }
        
        double filter(double input) {
            if(!init) {
                prev_output = input;
                init = true;
                return input;
            }
            
            prev_output = filter_variable_ * input + (1.0 - filter_variable_) * prev_output;
            return prev_output;
        }
    };

    Filter mpu6050_filter_, oak_imu_filter_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuFilterNode>());
  rclcpp::shutdown();
  return 0;
}