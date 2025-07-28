#include "rover_control/adaptive_controller.hpp"
#include <memory>
#include <algorithm>
#include <string>
#include <Eigen/Dense>
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace adaptive_controller
{
    void AdaptiveController::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
            
            // Populating class variables
            node_ = parent;
            auto node = node_.lock();
            costmap_ros_ = costmap_ros;
            tf_ = tf;
            plugin_name_ = name;
            logger_ = node->get_logger();
            clock_ = node->get_clock();

            // Here is where we get the params from the yaml file
            // declare_parameter_if_not_declared(
            //     node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(
            //     0.2));
            // declare_parameter_if_not_declared(
            //     node, plugin_name_ + ".lookahead_dist",
            //     rclcpp::ParameterValue(0.4));
            // declare_parameter_if_not_declared(
            //     node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(
            //     1.0));
            // declare_parameter_if_not_declared(
            //     node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(
            //     0.1));

            node->get_parameter(plugin_name_ + ".output_history_size", n_);
            node->get_parameter(plugin_name_ + ".input_history_size", m_);

            node->get_parameter(plugin_name_ + ".input_dimension", input_dim_);
            node->get_parameter(plugin_name_ + ".output_dimension", output_dim_);

            node->get_parameter(plugin_name_ + ".forgetting_factor", lambda_);

            node->get_parameter(plugin_name_ + ".initial_covariance", init_cov_);

            node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
            node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);

            double transform_tolerance;
            node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
            transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

            // Publishers
            global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
            error_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("error_metrics", 10);

            // Self Tuning Regulator
            controller_ = std::make_unique<SelfTuningRegulator>();
        }

    void AdaptiveController::cleanup() {
        RCLCPP_INFO(
            logger_,
            "Cleaning up controller: %s of type adaptive_controller::AdaptiveController",
            plugin_name_.c_str());
        global_pub_.reset();
        error_pub_.reset();

        controller_->shutdown();
    }

    void AdaptiveController::activate() {
          RCLCPP_INFO(
            logger_,
            "Activating controller: %s of type adaptive_controller::AdaptiveController\"  %s",
            plugin_name_.c_str(),plugin_name_.c_str());
        global_pub_->on_activate();
        error_pub_->on_activate();

        controller_->init(n_, m_, input_dim_, output_dim_, lambda_, init_cov_);
    }

    void AdaptiveController::deactivate() {
        RCLCPP_INFO(
            logger_,
            "Dectivating controller: %s of type adaptive_controller::AdaptiveController\"  %s",
            plugin_name_.c_str(),plugin_name_.c_str());
        global_pub_->on_deactivate();
        error_pub_->on_deactivate();
        
        controller_->reset();
    }

    void AdaptiveController::setSpeedLimit(const double & /*speed_limit*/, const bool & /*percantage*/) {
        
    }
    
    /// @brief Use the controller to compute desired velocity commands based on the current pose and velocity of the robot
    /// @param pose 
    /// @param velocity 
    /// @param goal_checker 
    /// @return the desired velocity command as the /cmd_vel topic
    geometry_msgs::msg::TwistStamped AdaptiveController::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) {

            // Convert the plan from the global planner's frame into usable robot coordinate frame
            auto reference_plan = transformGlobalPlan(pose);
            reference_plan.poses[0].pose.position.x;

            // Also need to change the orientation to yaw !
            
            VectorXd desired({reference_plan.poses[0].pose.position.x, reference_plan.poses[0].pose.position.y, reference_plan.poses[0].pose.orientation.z});
            VectorXd current_state({pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.z});
            VectorXd prev_input({velocity.linear.x, velocity.angular.z});

            VectorXd control_input = controller_->computeControl(desired, current_state, prev_input);
            
            geometry_msgs::msg::TwistStamped cmd_vel;
            cmd_vel.header.frame_id = pose.header.frame_id;
            cmd_vel.header.stamp = clock_->now();
            cmd_vel.twist.linear.x = control_input(0,0);
            cmd_vel.twist.angular.z = control_input(1,0);
            return cmd_vel;
        }
    
    void AdaptiveController::setPlan(const nav_msgs::msg::Path & path) {
        global_pub_->publish(path);
        global_plan_ = path;
    }

    ////////////////////////////////////////////////////////////////////////
    // Navigation Layer Helper Functions
    ////////////////////////////////////////////////////////////////////////
    
    /// @brief 
    /// @tparam Iter 
    /// @tparam Getter 
    /// @param begin 
    /// @param end 
    /// @param getCompareVal 
    /// @return 
    template<typename Iter, typename Getter>
    Iter min_by(Iter begin, Iter end, Getter getCompareVal)
    {
    if (begin == end) {
        return end;
    }
    auto lowest = getCompareVal(*begin);
    Iter lowest_it = begin;
    for (Iter it = ++begin; it != end; ++it) {
        auto comp = getCompareVal(*it);
        if (comp < lowest) {
        lowest = comp;
        lowest_it = it;
        }
    }
    return lowest_it;
    }

    /// @brief 
    /// @param pose 
    /// @return 
    nav_msgs::msg::Path AdaptiveController::transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose)
    {

    if (global_plan_.poses.empty()) {
        throw nav2_core::PlannerException("Received plan with zero length");
    }

    // let's get the pose of the robot in the frame of the plan
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!transformPose(
        tf_, global_plan_.header.frame_id, pose,
        robot_pose, transform_tolerance_))
    {
        throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
    }

    // We'll discard points on the plan that are outside the local costmap
    nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();

    double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
        costmap->getResolution() / 2.0;

    // First find the closest pose on the path to the robot
    auto transformation_begin =
        min_by(
        global_plan_.poses.begin(), global_plan_.poses.end(),
        [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
        return nav2_util::geometry_utils::euclidean_distance(robot_pose, ps);
        });

    // From the closest point, look for the first point that's further then dist_threshold from the
    // robot. These points are definitely outside of the costmap so we won't transform them.
    
    auto transformation_end = std::find_if(
        transformation_begin, end(global_plan_.poses),
        [&](const auto & global_plan_pose) {
        return nav2_util::geometry_utils::euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
        });

    // Helper function for the transform below. Transforms a PoseStamped from global frame to local
    auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
        // We took a copy of the pose, let's lookup the transform at the current time
        geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
        stamped_pose.header.frame_id = global_plan_.header.frame_id;
        stamped_pose.header.stamp = pose.header.stamp;
        stamped_pose.pose = global_plan_pose.pose;
        transformPose(
            tf_, costmap_ros_->getBaseFrameID(),
            stamped_pose, transformed_pose, transform_tolerance_);
        return transformed_pose;
        };

    // Transform the near part of the global plan into the robot's frame of reference.
    nav_msgs::msg::Path transformed_plan;
    std::transform(
        transformation_begin, transformation_end,
        std::back_inserter(transformed_plan.poses),
        transformGlobalPoseToLocal);
    transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
    transformed_plan.header.stamp = pose.header.stamp;

    // Remove the portion of the global plan that we've already passed so we don't
    // process it on the next iteration (this is called path pruning)
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    global_pub_->publish(transformed_plan);

    if (transformed_plan.poses.empty()) {
        throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
    }

    return transformed_plan;
    }

    /// @brief Used to convert tranforms in the map frame to the robot's own frame
    /// @param tf 
    /// @param frame 
    /// @param in_pose 
    /// @param out_pose 
    /// @param transform_tolerance 
    /// @return 
    bool AdaptiveController::transformPose(
        const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::string frame,
        const geometry_msgs::msg::PoseStamped & in_pose,
        geometry_msgs::msg::PoseStamped & out_pose,
        const rclcpp::Duration & transform_tolerance
    ) const
    {

        if (in_pose.header.frame_id == frame) {
            out_pose = in_pose;
            return true;
        }

        try {
            tf->transform(in_pose, out_pose, frame);
            return true;
        } catch (tf2::ExtrapolationException & ex) {
            auto transform = tf->lookupTransform(
            frame,
            in_pose.header.frame_id,
            tf2::TimePointZero
            );
            if (
            (rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
            transform_tolerance)
            {
            RCLCPP_ERROR(
                rclcpp::get_logger("tf_help"),
                "Transform data too old when converting from %s to %s",
                in_pose.header.frame_id.c_str(),
                frame.c_str()
            );
            RCLCPP_ERROR(
                rclcpp::get_logger("tf_help"),
                "Data time: %ds %uns, Transform time: %ds %uns",
                in_pose.header.stamp.sec,
                in_pose.header.stamp.nanosec,
                transform.header.stamp.sec,
                transform.header.stamp.nanosec
            );
            return false;
            } else {
            tf2::doTransform(in_pose, out_pose, transform);
            return true;
            }
        } catch (tf2::TransformException & ex) {
            RCLCPP_ERROR(
            rclcpp::get_logger("tf_help"),
            "Exception in transformPose: %s",
            ex.what()
            );
            return false;
        }
        return false;
    }

}

PLUGINLIB_EXPORT_CLASS(adaptive_controller::AdaptiveController, nav2_core::Controller)