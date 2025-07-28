#ifndef ADAPTIVE_CONTROLLER_HPP
#define ADAPTIVE_CONTROLLER_HPP

#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rover_control/self_tuning_regulator.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace adaptive_controller
{
class AdaptiveController : public nav2_core::Controller
{
    public:

        AdaptiveController() = default;
        ~AdaptiveController() override = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
            std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
            const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

        void cleanup() override;
        void activate() override;
        void deactivate() override;
        void setSpeedLimit(const double & speed_limit, const bool & percantage) override;

        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped & pose,
            const geometry_msgs::msg::Twist & velocity,
            nav2_core::GoalChecker * goal_checker) override;
        
        void setPlan(const nav_msgs::msg::Path & path) override;

    private:
        nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

        bool transformPose(
            const std::shared_ptr<tf2_ros::Buffer> tf,
            const std::string frame,
            const geometry_msgs::msg::PoseStamped & in_pose,
            geometry_msgs::msg::PoseStamped & out_pose,
            const rclcpp::Duration & transform_tolerance
        ) const;

        // Plan parameters
        nav_msgs::msg::Path global_plan_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;
        
        rclcpp::Duration transform_tolerance_ {0, 0}; // seconds, nanoseconds

        // ROS defined parameters
        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::string plugin_name_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        rclcpp::Logger logger_ {rclcpp::get_logger("AdaptiveController")};
        rclcpp::Clock::SharedPtr clock_;

        // Controller
        std::unique_ptr<SelfTuningRegulator> controller_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>> error_pub_;

        // Controller Specific Paramaters
        int n_, m_, input_dim_, output_dim_;
        double lambda_, init_cov_;
        double desired_linear_vel_;
        double max_angular_vel_;        
};

} // adaptive_controller
#endif // ADAPTIVE_CONTROLLER_HPP
