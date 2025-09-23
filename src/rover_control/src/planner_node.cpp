#include "nav_msgs/msg/odometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/srv/get_plan.h>

#include <utility>
#include "rover_control/astar.hpp"

using namespace std::chrono_literals;

class PlannerNode : public rclcpp::Node {

public:
  PlannerNode() : Node("planner_node"){

    // Map callback
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

    // Path publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    // Position Subscription
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Planner has started");
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){

    if (map_received_){
      return;
    } else {
        current_map_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Received map: %dx%d, resolution: %f",
                       msg->info.width, msg->info.height, msg->info.resolution);

        printMapInfo(*msg);
        map_received_ = true;
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){

    current_pose_.pose.position.x = msg->pose.pose.position.x;
    current_pose_.pose.position.y = msg->pose.pose.position.y;
    pose_received_ = true;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){

    if (!map_received_ || !pose_received_) {
      RCLCPP_WARN(this->get_logger(), "Map or pose not available yet");
      return;
    }
    // Plan and publish path
    planAndPublishPath(*msg);
  }

  void printMapInfo(const nav_msgs::msg::OccupancyGrid& map){

     RCLCPP_INFO(this->get_logger(), "Map origin: x=%f, y=%f",
                   map.info.origin.position.x, map.info.origin.position.y);

    RCLCPP_INFO(this->get_logger(), "First few map values: %d, %d, %d, %d",
               map.data[0], map.data[1], map.data[2], map.data[3]);
  }

  std::pair<int, int> worldTogrid(double world_x, double world_y){


    int grid_x = static_cast<int>((world_x - current_map_.info.origin.position.x) / current_map_.info.resolution);
    int grid_y = static_cast<int>((world_y - current_map_.info.origin.position.y) / current_map_.info.resolution);

    return {grid_x, grid_y};
  }

  std::pair<double, double> gridToworld(int grid_x, int grid_y){

    double world_x = grid_x * current_map_.info.resolution + current_map_.info.origin.position.x;
    double world_y = grid_y * current_map_.info.resolution + current_map_.info.origin.position.y;

    return {world_x, world_y};
  }

  std::vector<std::vector<int>> convertgrid(){

    std::vector<std::vector<int>> grid(current_map_.info.height, std::vector<int>(current_map_.info.width));

    for(int i{0}; i < grid.size(); i++){

      for(int j{0}; j < grid[i].size(); j++){

        int index = i * current_map_.info.width + j;
        grid[i][j] = current_map_.data[index];
      }
    }

    return grid;
  }

  void planAndPublishPath(const geometry_msgs::msg::PoseStamped& goal) {

        auto start_grid = worldTogrid(current_pose_.pose.position.x, current_pose_.pose.position.y);

        auto goal_grid = worldTogrid(goal.pose.position.x, goal.pose.position.y);

        auto grid = convertgrid();
        auto path_result = planner_.planPath(grid, start_grid, goal_grid);

        if (path_result) {
            auto ros_path = gridPathToRosPath(*path_result);
            path_pub_->publish(ros_path);
            RCLCPP_INFO(this->get_logger(), "Path published with %zu waypoints", path_result->size());
        }
    }

  nav_msgs::msg::Path gridPathToRosPath(const std::vector<std::pair<int, int>>& grid_path) {
    nav_msgs::msg::Path ros_path;
    ros_path.header.frame_id = "map";
    ros_path.header.stamp = this->now();

    for (const auto& grid_point : grid_path) {
        auto world_point = gridToworld(grid_point.first, grid_point.second);

        geometry_msgs::msg::PoseStamped pose;
        pose.header = ros_path.header;
        pose.pose.position.x = world_point.first;
        pose.pose.position.y = world_point.second;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;  // No rotation

        ros_path.poses.push_back(pose);
    }

    return ros_path;
  }

  // Node objects
  A_star planner_;
  nav_msgs::msg::OccupancyGrid current_map_;
  geometry_msgs::msg::PoseStamped current_pose_;

  bool map_received_ = false;
  bool pose_received_ = false;
  bool goal_received_ = false;

  // ROS interface
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_ ;

};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
