/*******************************************************************************
 *  Copyright (c) 2016-2022
 *  Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
 *  Christopher Newport University
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright notice,
 *       this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 *       THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *       "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *       LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *       FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *       COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *       INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *       BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *       LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *       CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *       LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 *       WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *       POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/**
 * @class FollowTopic
 * @brief An extension of base_local_planner that follows generated Paths
 *
 *   FollowTopic accepts Paths generated by a Flex Planner that were published
 *   on a topic. FollowTopic also has the ability to clear the local cost map
 *   by sending a ClearCostmapAction. The published Paths are accepted by way
 *   of a FollowTopicAction that instructs the node to subscribe to a planner's
 *   Path topic. The ability to clear the local cost map is accessible through
 *   the `clear_costmap` topic.
 */

#ifndef FLEX_CONTROLLER_FOLLOW_TOPIC_H
#define FLEX_CONTROLLER_FOLLOW_TOPIC_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_core/goal_checker.hpp"
#include "nav2_core/progress_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav_2d_utils/odom_subscriber.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"
#include "dwb_core/dwb_local_planner.hpp"
#include "nav_msgs/msg/path.h"
#include "flex_nav_common/action/clear_costmap.hpp"
#include "flex_nav_common/action/follow_topic.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <pluginlib/class_loader.hpp>

namespace flex_nav {
using ClearCostmapAction = flex_nav_common::action::ClearCostmap;
using ClearCostmapActionServer = nav2_util::SimpleActionServer<ClearCostmapAction>;

using FollowTopicAction = flex_nav_common::action::FollowTopic;
using FollowTopicActionServer = nav2_util::SimpleActionServer<FollowTopicAction>;

class FollowTopic : public nav2_util::LifecycleNode {
public:
  /**
   * @brief The constructor to instantiate a node
   * @param tf A reference to a TransformListener
   */
  FollowTopic(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief The destructor to tear down a node
   */
  ~FollowTopic();

protected:
  /**
   * @brief Configures controller parameters and member variables
   *
   * Configures controller plugin and costmap; Initialize odom subscriber,
   * velocity publisher and follow path action server.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   * @throw pluginlib::PluginlibException When failed to initialize controller
   * plugin
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates member variables
   *
   * Activates controller, costmap, velocity publisher and follow path action
   * server
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates member variables
   *
   * Deactivates follow path action server, controller, costmap and velocity
   * publisher. Before calling deactivate state, velocity is being set to zero.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Calls clean up states and resets member variables.
   *
   * Controller and costmap clean up state is called, and resets rest of the
   * variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /**
   * @brief Calls velocity publisher to publish zero velocity
   */
  void publishZeroVelocity();

  /**
   * @brief Called to terminate action and publish zero velocity
   * @param pose To store current pose of the robot
   */
  void terminateController(const geometry_msgs::msg::PoseStamped & pose);


  /**
   * @brief Calculates velocity and publishes to "cmd_vel" topic
   * @param pose To store current pose of the robot
   */
  void computeAndPublishVelocity(geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Calls velocity publisher to publish the velocity on "cmd_vel" topic
   * @param velocity Twist velocity to be published
   */
  void publishVelocity(const geometry_msgs::msg::TwistStamped & velocity);

  /**
   * @brief Assigns path to controller
   * @param path Path received from action server
   */
  void setPlannerPath(const nav_msgs::msg::Path & path);

  /**
   * @brief Checks if goal is reached
   * @return true or false
   */
  bool isGoalReached(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Obtain current pose of the robot
   * @param pose To store current pose of the robot
   * @return true if able to obtain current pose of the robot, else false
   */
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief The call back for the FollowTopicActionServer
   * @param goal A reference to the published goal
   */
  void execute();

  /**
   * @brief The call back for the Path Subscriber
   * @param goal A reference to the published goal
   */
  void topic_cb(const nav_msgs::msg::Path::SharedPtr data);

  /**
   * @brief The call back for the ClearCostmapActionServer
   * @param goal A reference to the published goal
   */
  void clear_costmap();

  std::unique_ptr<FollowTopicActionServer> ft_server_;
  std::unique_ptr<ClearCostmapActionServer> cc_server_;

  // The controller needs a costmap node
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;

  // Publishers and subscribers
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_stamp_publisher_;
  std::string vel_publisher_name_, vel_stamp_publisher_name_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_;
  std::unique_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;

  // Progress Checker Plugin
  pluginlib::ClassLoader<nav2_core::ProgressChecker> progress_checker_loader_;
  nav2_core::ProgressChecker::Ptr progress_checker_;
  std::string default_progress_checker_id_;
  std::string default_progress_checker_type_;
  std::string progress_checker_id_;
  std::string progress_checker_type_;

  // Goal Checker Plugin
  pluginlib::ClassLoader<nav2_core::GoalChecker> goal_checker_loader_;
  nav2_core::GoalChecker::Ptr goal_checker_;
  std::string default_goal_checker_id_;
  std::string default_goal_checker_type_;
  std::string goal_checker_id_;
  std::string goal_checker_type_;

  // Controller Plugins
  pluginlib::ClassLoader<nav2_core::Controller> lp_loader_;
  nav2_core::Controller::Ptr controller_;
  std::string default_id_;
  std::string default_type_;
  std::string controller_id_;
  std::string controller_type_;

  double controller_frequency_;
  double min_x_velocity_threshold_;
  double min_y_velocity_threshold_;
  double min_theta_velocity_threshold_;

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  bool running_;

  nav_msgs::msg::Path::SharedPtr current_path_ptr_;
  nav_msgs::msg::Path::SharedPtr latest_path_ptr_;
  geometry_msgs::msg::Pose end_pose_;

  std::string name_;
};
};

#endif
