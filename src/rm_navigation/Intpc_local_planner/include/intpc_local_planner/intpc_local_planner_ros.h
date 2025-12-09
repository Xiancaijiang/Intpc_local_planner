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

#ifndef INPC_LOCAL_PLANNER_ROS_H_
#define INPC_LOCAL_PLANNER_ROS_H_

#include <pluginlib/class_loader.hpp>

#include <rclcpp/rclcpp.hpp>

// Navigation2 local planner base class and utilities
#include <nav2_core/controller.hpp>

// message types
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// transforms
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>

// costmap
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/cost_values.hpp>

#include <nav2_util/lifecycle_node.hpp>
#include <nav2_util/node_utils.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

// Intpc core planner
#include <intpc_local_planner/intpc_local_planner.h>

// Eigen for matrix operations
#include <Eigen/Dense>

namespace intpc_local_planner
{
using TFBufferPtr = std::shared_ptr<tf2_ros::Buffer>;
using CostmapROSPtr = std::shared_ptr<nav2_costmap_2d::Costmap2DROS>;

/**
  * @class IntpcLocalPlannerROS
  * @brief Implements the Intpc local planner plugin for Navigation2
  * This planner uses Fourier path representation, proportional-tangential-normal control,
  * Control Barrier Functions (CBF), and quadratic programming optimization.
  */
class IntpcLocalPlannerROS : public nav2_core::Controller
{
public:
  /**
    * @brief Constructor of the Intpc plugin
    */
  IntpcLocalPlannerROS();

  /**
    * @brief  Destructor of the plugin
    */
  ~IntpcLocalPlannerROS();
  
  /**
   * @brief Configure the Intpc plugin
   * 
   * @param node The node of the instance
   * @param name The name of the plugin
   * @param tf Pointer to a transform buffer
   * @param costmap_ros Cost map representing occupied and free space
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    
  void activate() override;
  void deactivate() override;
  void cleanup() override;

  /**
    * @brief Set the plan that the Intpc local planner is following
    * @param orig_global_plan The plan to pass to the local planner
    */
  void setPlan(const nav_msgs::msg::Path & orig_global_plan) override;

  /**
    * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
    * @param pose is the current position
    * @param velocity is the current velocity
    * @return velocity commands to send to the base
    */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker * goal_checker);
  
protected:
  /**
    * @brief Transform the global plan to the local frame
    * @param global_plan The global plan
    * @param global_frame The global frame
    * @param robot_pose The robot pose
    * @return The transformed plan
    */
  nav_msgs::msg::Path transformGlobalPlan(const nav_msgs::msg::Path &global_plan, 
                                         const std::string &global_frame, 
                                         const geometry_msgs::msg::PoseStamped &robot_pose);
  
  /**
    * @brief Extract the local goal from the transformed plan
    * @param transformed_plan The transformed plan
    * @return The local goal pose
    */
  geometry_msgs::msg::PoseStamped getLocalGoal(const nav_msgs::msg::Path &transformed_plan);
  
  /**
    * @brief Saturate the velocity commands to be within the robot's capabilities
    * @param twist The twist command to saturate
    */
  void saturateVelocity(geometry_msgs::msg::Twist &twist);
  
  /**
    * @brief Extract obstacles from the costmap
    * @param pose Current robot pose
    * @param x_obstacles Output vector of obstacle x coordinates
    * @param y_obstacles Output vector of obstacle y coordinates
    * @param obstacle_radii Output vector of obstacle radii
    */
  void extractObstaclesFromCostmap(
    const geometry_msgs::msg::PoseStamped &pose,
    std::vector<double> &x_obstacles,
    std::vector<double> &y_obstacles,
    std::vector<double> &obstacle_radii);
  
  // ROS node and parameters
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  
  // Transform and costmap
  TFBufferPtr tf_;
  CostmapROSPtr costmap_ros_;
  
  // Current plan and state
  nav_msgs::msg::Path global_plan_;
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::Twist current_velocity_;
  
  // Parameters
  double max_vel_x_;
  double max_vel_x_backwards_;
  double max_vel_theta_;
  double acc_lim_x_;
  double acc_lim_theta_;
  double lookahead_dist_;
  
  // Intpc core planner
  std::unique_ptr<IntpcLocalPlanner> planner_;
  
  // Intpc specific parameters
  double k_gain_;         // Gain for path following
  double obstacle_radius_; // Radius of obstacles
  double robot_radius_;   // Radius of the robot
  
  // Visualization
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace intpc_local_planner

#endif  // INPC_LOCAL_PLANNER_ROS_H_
