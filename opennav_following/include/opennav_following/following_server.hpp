// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OPENNAV_FOLLOWING__FOLLOWING_SERVER_HPP_
#define OPENNAV_FOLLOWING__FOLLOWING_SERVER_HPP_

#include <vector>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/twist_publisher.hpp"
#include "opennav_docking/controller.hpp"
#include "opennav_docking/pose_filter.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "opennav_following/following_exceptions.hpp"
#include "opennav_following_msgs/action/follow_object.hpp"

namespace opennav_following
{
/**
 * @class opennav_following::FollowingServer
 * @brief An action server which implements a dynamic following behavior
 */
class FollowingServer : public nav2_util::LifecycleNode
{
public:
  using FollowObjectAction = opennav_following_msgs::action::FollowObject;
  using FollowingActionServer = nav2_util::SimpleActionServer<FollowObjectAction>;

  /**
   * @brief A constructor for opennav_following::FollowingServer
   * @param options Additional options to control creation of the node.
   */
  explicit FollowingServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief A destructor for opennav_following::FollowingServer
   */
  ~FollowingServer() = default;

  /**
   * @brief Publish feedback from a following action.
   * @param state Current state - should be one of those defined in message.
   */
  void publishFollowingFeedback(uint16_t state);

  /**
   * @brief Do initial perception, up to a timeout.
   * @param object_pose Initial object pose, will be refined by perception.
   */
  void doInitialPerception(geometry_msgs::msg::PoseStamped & object_pose);

  /**
   * @brief Use control law and perception to approach the object
   * @param object_pose Initial object pose, will be refined by perception.
   * @returns True if successfully approached, False if cancelled. For
   *          any internal error, will throw.
   */
  bool approachObject(geometry_msgs::msg::PoseStamped & object_pose);

  /**
   * @brief Get the robot pose (aka base_frame pose) in another frame.
   * @param frame The frame_id to get the robot pose in.
   * @returns Computed robot pose, throws TF2 error if failure.
   */
  virtual geometry_msgs::msg::PoseStamped getRobotPoseInFrame(const std::string & frame);

  /**
   * @brief Gets a preempted goal if immediately requested
   * @param Goal goal to check or replace if required with preemption
   * @param action_server Action server to check for preemptions on
   * @return SUCCESS or FAILURE
   */
  template<typename ActionT>
  void getPreemptedGoalIfRequested(
    typename std::shared_ptr<const typename ActionT::Goal> goal,
    const std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server);

  /**
   * @brief Checks and logs warning if action canceled
   * @param action_server Action server to check for cancellation on
   * @param name Name of action to put in warning message
   * @return True if action has been cancelled
   */
  template<typename ActionT>
  bool checkAndWarnIfCancelled(
    std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server,
    const std::string & name);

  /**
   * @brief Checks and logs warning if action preempted
   * @param action_server Action server to check for preemption on
   * @param name Name of action to put in warning message
   * @return True if action has been preempted
   */
  template<typename ActionT>
  bool checkAndWarnIfPreempted(
    std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server,
    const std::string & name);

  /**
   * @brief Configure member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Publish zero velocity at terminal condition
   */
  void publishZeroVelocity();

  /**
   * @brief Method to obtain the refined dynamic pose
   * @param pose The initial estimate of the dynamic pose
   * which will be updated with the refined pose.
   */
  virtual bool getRefinedPose(geometry_msgs::msg::PoseStamped & pose);

protected:
  /**
   * @brief Projects the given pose backwards the specified distance.
   *
   * @param pose Input pose to project
   * @param distance Distance to move (in meters)
   * @return Pose distance meters behind the input pose
   */
  geometry_msgs::msg::PoseStamped getBackwardPose(
    const geometry_msgs::msg::PoseStamped & pose, double distance);

  /**
   * @brief Check if the goal has been reached.
   *
   * @param goal_pose The goal pose to check
   * @return true If the goal has been reached
   */
  bool isGoalReached(const geometry_msgs::msg::PoseStamped & goal_pose);

  /**
   * @brief Main action callback method to complete following request
   */
  void followObject();

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

  // Frequency to run control loops
  double controller_frequency_;
  // Timeout for initially detecting the object
  double initial_perception_timeout_;
  // Tolerance for transforming coordinates
  double transform_tolerance_;
  // Tolerances for arriving at the safe_distance pose
  double linear_tolerance_, angular_tolerance_;
  // This is the root frame of the robot - typically "base_link"
  std::string base_frame_;
  // This is our fixed frame for controlling - typically "odom"
  std::string fixed_frame_;
  // Does the robot drive backwards to follow the object? Default is forwards
  bool backwards_;
  // Desired distance to keep from the object
  double desired_distance_;
  // Skip perception orientation
  bool skip_orientation_;

  // This is a class member so it can be accessed in publish feedback
  rclcpp::Time action_start_time_;

  // Subscribe to the dynamic pose
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dynamic_pose_sub_;

  // Publish the filtered dynamic pose
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    filtered_dynamic_pose_pub_;

  // Publish the trajectory
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> trajectory_pub_;

  // Latest message
  geometry_msgs::msg::PoseStamped detected_dynamic_pose_;

  // This is the actual dynamic pose once it has been filtered
  geometry_msgs::msg::PoseStamped dynamic_pose_;

  // Filtering of detected poses
  std::unique_ptr<opennav_docking::PoseFilter> filter_;
  double detection_timeout_;

  std::unique_ptr<nav2_util::TwistPublisher> vel_publisher_;
  std::unique_ptr<FollowingActionServer> following_action_server_;

  std::unique_ptr<opennav_docking::Controller> controller_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
};

}  // namespace opennav_following

#endif  // OPENNAV_FOLLOWING__FOLLOWING_SERVER_HPP_
