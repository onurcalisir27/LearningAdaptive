#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class PendulumServiceHandle : public rclcpp::Node
{
public:
    PendulumServiceHandle() : Node("pendulum_service_handle")
    {
        this->declare_parameter("is_two_link", true);
        this->get_parameter("is_two_link", is_two_link);

        reset_service_ = this->create_service<std_srvs::srv::Trigger>(
            "reset_pendulum",
            std::bind(&PendulumServiceHandle::handle_reset, this,
                     std::placeholders::_1, std::placeholders::_2));

        gz_set_pose_client_ = this->create_client<ros_gz_interfaces::srv::SetEntityPose>(
            "/world/controller_test/set_pose");

        torque_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pendulum/commands", 10);
        RCLCPP_INFO(this->get_logger(), "Pendulum reset service ready");
    }

private:
    void handle_reset(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        auto msg = std_msgs::msg::Float64MultiArray();
        if(is_two_link){
          msg.data = {0.0, 0.0};
        } else {
          msg.data = {0.0};
        }
        torque_publisher_->publish(msg);

        auto gz_request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
        gz_request->entity.name = "pendulum";
        gz_request->pose.position.x = 1.5;
        gz_request->pose.position.y = 1.5;
        gz_request->pose.position.z = 0.2;
        gz_request->pose.orientation.w = 1.0;
        gz_request->pose.orientation.x = 0.0;
        gz_request->pose.orientation.y = 0.0;
        gz_request->pose.orientation.z = 0.0;

        if (!gz_set_pose_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Gazebo set_pose service not available");
            response->success = false;
            response->message = "Gazebo service unavailable";
            return;
        }

        auto future = gz_set_pose_client_->async_send_request(gz_request);

        // Wait for response WITHOUT spinning
        auto status = future.wait_for(std::chrono::seconds(2));

        if (status == std::future_status::ready) {
            auto result = future.get();
            response->success = result->success;
            response->message = "Pendulum reset successful";
            RCLCPP_INFO(this->get_logger(), "Reset successful");
        } else {
            response->success = false;
            response->message = "Timeout waiting for Gazebo service";
            RCLCPP_ERROR(this->get_logger(), "Gazebo service timeout");
        }
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr gz_set_pose_client_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_publisher_;
    bool is_two_link;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PendulumServiceHandle>());
    rclcpp::shutdown();
    return 0;
}
