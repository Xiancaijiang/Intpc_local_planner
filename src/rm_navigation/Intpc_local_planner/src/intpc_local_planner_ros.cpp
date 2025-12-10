/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024,
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <intpc_local_planner/intpc_local_planner_ros.h>
#include <intpc_local_planner/intpc_local_planner.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav2_util/node_utils.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>

namespace intpc_local_planner
{

IntpcLocalPlannerROS::IntpcLocalPlannerROS()
{
  // Initialize variables
  max_vel_x_ = 0.5;
  max_vel_x_backwards_ = 0.5;
  max_vel_theta_ = 1.0;
  acc_lim_x_ = 2.5;
  acc_lim_theta_ = 3.2;
  
  // Initialize Intpc planner core
  planner_ = std::make_unique<IntpcLocalPlanner>();
  
  // Intpc specific parameters
  k_gain_ = 1.0;
  obstacle_radius_ = 0.1; // 默认障碍物半径
}

IntpcLocalPlannerROS::~IntpcLocalPlannerROS()
{
  // Cleanup
}

void IntpcLocalPlannerROS::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // Initialize node and parameters
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in configure");
  }
  
  logger_ = node->get_logger();
  RCLCPP_INFO(logger_, "Configuring IntpcLocalPlannerROS");
  clock_ = node->get_clock();
  name_ = name;
  
  // Set up transform and costmap
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  
  // Load parameters
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".max_vel_x", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".max_vel_x_backwards", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".max_vel_theta", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".acc_lim_x", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".acc_lim_theta", rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".lookahead_dist", rclcpp::ParameterValue(1.0));
  
  // Intpc specific parameters
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".k_gain", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".obstacle_radius", rclcpp::ParameterValue(0.25));
  
  node->get_parameter(name_ + ".max_vel_x", max_vel_x_);
  node->get_parameter(name_ + ".max_vel_x_backwards", max_vel_x_backwards_);
  node->get_parameter(name_ + ".max_vel_theta", max_vel_theta_);
  node->get_parameter(name_ + ".acc_lim_x", acc_lim_x_);
  node->get_parameter(name_ + ".acc_lim_theta", acc_lim_theta_);
  node->get_parameter(name_ + ".lookahead_dist", lookahead_dist_);
  
  // Get Intpc specific parameters
  node->get_parameter(name_ + ".k_gain", k_gain_);
  node->get_parameter(name_ + ".obstacle_radius", obstacle_radius_);
  
  RCLCPP_INFO(logger_, "Intpc parameters: k_gain=%.2f, obstacle_radius=%.2f", 
              k_gain_, obstacle_radius_);
  
  // Set up visualization publisher
  marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(name_ + "/markers", 1);
  
  RCLCPP_INFO(logger_, "IntpcLocalPlannerROS configured successfully");
}

void IntpcLocalPlannerROS::activate()
{
  RCLCPP_INFO(logger_, "Activating IntpcLocalPlannerROS");
}

void IntpcLocalPlannerROS::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating IntpcLocalPlannerROS");
}

void IntpcLocalPlannerROS::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up IntpcLocalPlannerROS");
}

void IntpcLocalPlannerROS::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  // Implement speed limit functionality
  if (percentage) {
    // Speed limit is a percentage of the maximum velocity
    max_vel_x_ *= (speed_limit / 100.0);
    max_vel_theta_ *= (speed_limit / 100.0);
  } else {
    // Speed limit is an absolute value
    max_vel_x_ = speed_limit;
    // For angular velocity, we might want to maintain a ratio or set directly
    // Here we'll set it directly for simplicity
    max_vel_theta_ = speed_limit * 1.5; // Maintain some ratio for angular velocity
  }
  
  RCLCPP_DEBUG(logger_, "Set speed limit: linear=%.2f, angular=%.2f (percentage=%s)",
               max_vel_x_, max_vel_theta_, percentage ? "true" : "false");
}

void IntpcLocalPlannerROS::setPlan(const nav_msgs::msg::Path & orig_global_plan)
{
  RCLCPP_DEBUG(logger_, "Setting new plan with %zu points", orig_global_plan.poses.size());
  global_plan_ = orig_global_plan;
}

geometry_msgs::msg::TwistStamped IntpcLocalPlannerROS::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped &pose,
  const geometry_msgs::msg::Twist &velocity,
  nav2_core::GoalChecker * goal_checker) override
{
  // Store current pose and velocity
  current_pose_ = pose;
  current_velocity_ = velocity;
  
  // If the global plan is empty, return zero velocity
  if (global_plan_.poses.empty()) {
    RCLCPP_WARN(logger_, "Global plan is empty");
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.header.frame_id = pose.header.frame_id;
    return cmd_vel;
  }
  
  // Transform the global plan to the local frame
  nav_msgs::msg::Path transformed_plan;
  try {
    transformed_plan = transformGlobalPlan(global_plan_, pose.header.frame_id, pose);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Failed to transform global plan: %s", e.what());
    // Return zero velocity
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.header.frame_id = pose.header.frame_id;
    return cmd_vel;
  }
  
  // If the transformed plan is empty, return zero velocity
  if (transformed_plan.poses.empty()) {
    RCLCPP_WARN(logger_, "Transformed plan is empty");
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.header.frame_id = pose.header.frame_id;
    return cmd_vel;
  }
  
  // Get the local goal from the transformed plan
  geometry_msgs::msg::PoseStamped local_goal = getLocalGoal(transformed_plan);
  
  // Calculate the vector from the robot to the local goal
  double dx = local_goal.pose.position.x - pose.pose.position.x;
  double dy = local_goal.pose.position.y - pose.pose.position.y;
  double distance = std::hypot(dx, dy);
  
  // Check if we've reached the goal
  if (distance < 0.1 || (goal_checker && goal_checker->isGoalReached(pose.pose, pose.pose, velocity))) {
    RCLCPP_INFO(logger_, "Goal reached, distance: %f", distance);
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.header.frame_id = pose.header.frame_id;
    return cmd_vel;
  }
  
  // Calculate current yaw from quaternion
  tf2::Quaternion q(
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z,
    pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  
  // Extract obstacles from the costmap
  std::vector<double> x_obstacles, y_obstacles, obstacle_radii;
  extractObstaclesFromCostmap(pose, x_obstacles, y_obstacles, obstacle_radii);
  
  // Check for obstacles in the immediate vicinity
  bool obstacle_nearby = false;
  double obstacle_distance_threshold = 0.5; // 50cm threshold for nearby obstacles
  for (size_t i = 0; i < x_obstacles.size(); i++) {
    double obs_dist = std::hypot(x_obstacles[i] - pose.pose.position.x, 
                                 y_obstacles[i] - pose.pose.position.y);
    if (obs_dist < obstacle_distance_threshold) {
      obstacle_nearby = true;
      break;
    }
  }
  
  RCLCPP_DEBUG(logger_, "Extracted %zu obstacles for local planning, obstacle nearby: %s", 
               x_obstacles.size(), obstacle_nearby ? "true" : "false");
  
  // Initialize the path parameters in the Intpc planner
  planner_->initializePathParams(local_goal.pose.position.x, local_goal.pose.position.y);
  
  // Calculate robot state
  double robot_x = pose.pose.position.x;
  double robot_y = pose.pose.position.y;
  double robot_theta = yaw;
  
  // Calculate distance-based velocity factor
  double speed_factor = std::min(1.0, distance / lookahead_dist_ * 2.0);
  
  // Reduce speed if obstacles are nearby
  if (obstacle_nearby) {
    speed_factor *= 0.5; // Reduce speed by half when obstacles are nearby
    RCLCPP_DEBUG(logger_, "Obstacle detected nearby, reducing speed factor to %.2f", speed_factor);
  }
  
  // Adjust lookahead distance based on obstacle presence
  double adjusted_lookahead_dist = lookahead_dist_;
  if (obstacle_nearby) {
    adjusted_lookahead_dist = lookahead_dist_ * 0.7; // Reduce lookahead distance when obstacles are nearby
  }
  
  // Compute Intpc control velocity with adjusted parameters
  Eigen::Vector2d control_velocity = planner_->computeIntpcControl(
    robot_x, robot_y, robot_theta,
    x_obstacles, y_obstacles, obstacle_radii,
    k_gain_, max_vel_x_ * speed_factor);
  
  // Build velocity command
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = pose.header.frame_id;
  
  // Set velocity commands based on distance to goal and obstacle presence
  cmd_vel.twist.linear.x = control_velocity(0) * speed_factor;
  
  // For angular velocity, increase responsiveness when near obstacles
  double angular_velocity_factor = obstacle_nearby ? 1.2 : 1.0;
  cmd_vel.twist.angular.z = control_velocity(1) * angular_velocity_factor;
  
  // Saturate velocity commands
  saturateVelocity(cmd_vel.twist);
  
  // Add acceleration constraints for smoother motion
  double dt = 0.1; // Approximate time step
  if (cmd_vel.twist.linear.x - current_velocity_.linear.x > acc_lim_x_ * dt) {
    cmd_vel.twist.linear.x = current_velocity_.linear.x + acc_lim_x_ * dt;
  } else if (cmd_vel.twist.linear.x - current_velocity_.linear.x < -acc_lim_x_ * dt) {
    cmd_vel.twist.linear.x = current_velocity_.linear.x - acc_lim_x_ * dt;
  }
  
  if (cmd_vel.twist.angular.z - current_velocity_.angular.z > acc_lim_theta_ * dt) {
    cmd_vel.twist.angular.z = current_velocity_.angular.z + acc_lim_theta_ * dt;
  } else if (cmd_vel.twist.angular.z - current_velocity_.angular.z < -acc_lim_theta_ * dt) {
    cmd_vel.twist.angular.z = current_velocity_.angular.z - acc_lim_theta_ * dt;
  }
  
  RCLCPP_DEBUG(logger_, "Generated Intpc velocity command: linear.x = %f, angular.z = %f, distance to goal: %f, obstacle nearby: %s", 
               cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, distance, obstacle_nearby ? "true" : "false");
  
  return cmd_vel;
}

nav_msgs::msg::Path IntpcLocalPlannerROS::transformGlobalPlan(const nav_msgs::msg::Path &global_plan, 
                                                           const std::string &global_frame, 
                                                           const geometry_msgs::msg::PoseStamped &robot_pose)
{
  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.stamp = robot_pose.header.stamp;
  transformed_plan.header.frame_id = global_frame;
  
  if (global_plan.poses.empty()) {
    RCLCPP_WARN(logger_, "Global plan is empty");
    return transformed_plan;
  }
  
  // Store the robot's current position for distance calculations
  double robot_x = robot_pose.pose.position.x;
  double robot_y = robot_pose.pose.position.y;
  
  // Find the closest point on the global plan to the robot
  size_t closest_idx = 0;
  double min_distance = std::numeric_limits<double>::max();
  
  for (size_t i = 0; i < global_plan.poses.size(); ++i) {
    try {
      // Transform the global pose to the robot's frame
      geometry_msgs::msg::PoseStamped global_pose = global_plan.poses[i];
      geometry_msgs::msg::PoseStamped transformed_pose;
      
      // Set the target frame and stamp
      global_pose.header.stamp = robot_pose.header.stamp;
      
      // Transform the pose
      tf_->transform(global_pose, transformed_pose, global_frame);
      
      // Calculate distance from robot
      double distance = std::hypot(
        transformed_pose.pose.position.x - robot_x,
        transformed_pose.pose.position.y - robot_y
      );
      
      if (distance < min_distance) {
        min_distance = distance;
        closest_idx = i;
      }
      
      // Add to transformed plan
      transformed_plan.poses.push_back(transformed_pose);
      
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(logger_, "Transform exception: %s", ex.what());
      continue;
    }
  }
  
  if (transformed_plan.poses.empty()) {
    RCLCPP_WARN(logger_, "Failed to transform any points in the global plan");
    return transformed_plan;
  }
  
  // Remove points before the closest point to reduce plan size
  // This helps focus on the relevant part of the path
  if (closest_idx > 0 && closest_idx < transformed_plan.poses.size()) {
    // Keep some points before the closest point for smoother transitions
    size_t start_idx = std::max(size_t(0), closest_idx - 3);
    transformed_plan.poses.erase(transformed_plan.poses.begin(), 
                               transformed_plan.poses.begin() + start_idx);
  }
  
  RCLCPP_DEBUG(logger_, "Transformed global plan with %zu points", transformed_plan.poses.size());
  
  return transformed_plan;
}

geometry_msgs::msg::PoseStamped IntpcLocalPlannerROS::getLocalGoal(const nav_msgs::msg::Path &transformed_plan)
{
  if (transformed_plan.poses.empty()) {
    throw std::runtime_error("Empty plan provided to getLocalGoal");
  }
  
  // 默认前瞻距离
  double lookahead_dist = 0.5; // 50cm前瞻距离
  
  // Find the first pose that is at least lookahead_dist away from the robot
  // For this, we need to find the point closest to the robot in the transformed plan first
  size_t closest_idx = 0;
  double min_distance = std::numeric_limits<double>::max();
  
  // Since the transformed plan is in the robot's current frame, the robot is at (0,0)
  for (size_t i = 0; i < transformed_plan.poses.size(); ++i) {
    double distance = std::hypot(
      transformed_plan.poses[i].pose.position.x,
      transformed_plan.poses[i].pose.position.y
    );
    
    if (distance < min_distance) {
      min_distance = distance;
      closest_idx = i;
    }
  }
  
  // Now find the local goal point that is lookahead_dist away from the closest point
  size_t local_goal_idx = closest_idx;
  double cumulative_distance = 0.0;
  
  // Start from the closest point and move forward in the plan
  for (size_t i = closest_idx + 1; i < transformed_plan.poses.size(); ++i) {
    // Calculate distance between consecutive points
    double segment_dist = std::hypot(
      transformed_plan.poses[i].pose.position.x - transformed_plan.poses[i-1].pose.position.x,
      transformed_plan.poses[i].pose.position.y - transformed_plan.poses[i-1].pose.position.y
    );
    
    cumulative_distance += segment_dist;
    
    // If we've reached or exceeded the lookahead distance
    if (cumulative_distance >= lookahead_dist) {
      local_goal_idx = i;
      break;
    }
  }
  
  // If we didn't find a point far enough, use the furthest point in the plan
  if (local_goal_idx == closest_idx) {
    local_goal_idx = transformed_plan.poses.size() - 1;
  }
  
  RCLCPP_DEBUG(logger_, "Local goal index: %zu, distance from robot: %f", 
               local_goal_idx, 
               std::hypot(transformed_plan.poses[local_goal_idx].pose.position.x, 
                         transformed_plan.poses[local_goal_idx].pose.position.y));
  
  return transformed_plan.poses[local_goal_idx];
}

void IntpcLocalPlannerROS::saturateVelocity(geometry_msgs::msg::Twist &twist)
{
  // Saturate linear velocity
  if (twist.linear.x > max_vel_x_) {
    twist.linear.x = max_vel_x_;
  } else if (twist.linear.x < -max_vel_x_backwards_) {
    twist.linear.x = -max_vel_x_backwards_;
  }
  
  // Saturate angular velocity
  if (twist.angular.z > max_vel_theta_) {
    twist.angular.z = max_vel_theta_;
  } else if (twist.angular.z < -max_vel_theta_) {
    twist.angular.z = -max_vel_theta_;
  }
}

void IntpcLocalPlannerROS::extractObstaclesFromCostmap(
  const geometry_msgs::msg::PoseStamped &pose,
  std::vector<double> &x_obstacles,
  std::vector<double> &y_obstacles,
  std::vector<double> &obstacle_radii) {
  if (!costmap_ros_) {
    RCLCPP_WARN(logger_, "Costmap not available for obstacle extraction");
    return;
  }
  
  nav2_costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  if (!costmap) {
    RCLCPP_WARN(logger_, "Costmap pointer is null");
    return;
  }
  
  // 提取机器人当前位置周围的障碍物
  double robot_x = pose.pose.position.x;
  double robot_y = pose.pose.position.y;
  
  // 转换到地图坐标系
  unsigned int map_x, map_y;
  if (!costmap->worldToMap(robot_x, robot_y, map_x, map_y)) {
    RCLCPP_WARN(logger_, "Robot position outside costmap");
    return;
  }
  
  // 搜索半径（地图单元格）
  int search_radius = static_cast<int>(lookahead_dist_ * costmap->getResolution());
  int max_search_x = std::min(static_cast<int>(map_x), static_cast<int>(costmap->getSizeInCellsX()) - static_cast<int>(map_x));
  int max_search_y = std::min(static_cast<int>(map_y), static_cast<int>(costmap->getSizeInCellsY()) - static_cast<int>(map_y));
  search_radius = std::min(search_radius, max_search_x);
  search_radius = std::min(search_radius, max_search_y);
  
  // 记录已访问的障碍物单元格，避免重复
  std::vector<std::vector<bool>> visited(costmap->getSizeInCellsX(), 
                                       std::vector<bool>(costmap->getSizeInCellsY(), false));
  
  // 搜索障碍物
  for (int i = -search_radius; i <= search_radius; i++) {
    for (int j = -search_radius; j <= search_radius; j++) {
      unsigned int current_x = map_x + i;
      unsigned int current_y = map_y + j;
      
      // 检查边界和是否已访问
      if (current_x >= costmap->getSizeInCellsX() || current_y >= costmap->getSizeInCellsY() || visited[current_x][current_y]) {
        continue;
      }
      
      // 检查是否为障碍物
      unsigned char cost = costmap->getCost(current_x, current_y);
      if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        // 转换回世界坐标系
        double world_x, world_y;
        costmap->mapToWorld(current_x, current_y, world_x, world_y);
        
        // 添加障碍物
        x_obstacles.push_back(world_x);
        y_obstacles.push_back(world_y);
        obstacle_radii.push_back(obstacle_radius_);
        
        // 标记周围单元格为已访问，避免重复
        for (int dx = -1; dx <= 1; dx++) {
          for (int dy = -1; dy <= 1; dy++) {
            int nx = current_x + dx;
            int ny = current_y + dy;
            if (nx >= 0 && nx < static_cast<int>(costmap->getSizeInCellsX()) && 
                ny >= 0 && ny < static_cast<int>(costmap->getSizeInCellsY())) {
              visited[nx][ny] = true;
            }
          }
        }
      }
    }
  }
  
  RCLCPP_DEBUG(logger_, "Extracted %zu obstacles from costmap", x_obstacles.size());
}

} // namespace intpc_local_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(intpc_local_planner::IntpcLocalPlannerROS, nav2_core::Controller)
