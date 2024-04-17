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

#include "angles/angles.h"
#include "opennav_following/following_server.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace opennav_following
{

FollowingServer::FollowingServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("following_server", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating %s", get_name());

  declare_parameter("controller_frequency", 50.0);
  declare_parameter("initial_perception_timeout", 5.0);
  declare_parameter("detection_timeout", 1.0);
  declare_parameter("object_approach_timeout", 30.0);
  declare_parameter("transform_tolerance", 0.2);
  declare_parameter("base_frame", "base_link");
  declare_parameter("fixed_frame", "odom");
  declare_parameter("backwards", false);
  declare_parameter("filter_coef", 0.1);
  declare_parameter("safe_distance", 1.0);
}

nav2_util::CallbackReturn
FollowingServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring %s", get_name());
  auto node = shared_from_this();

  get_parameter("controller_frequency", controller_frequency_);
  get_parameter("initial_perception_timeout", initial_perception_timeout_);
  get_parameter("detection_timeout", detection_timeout_);
  get_parameter("object_approach_timeout", object_approach_timeout_);
  get_parameter("transform_tolerance", transform_tolerance_);
  get_parameter("base_frame", base_frame_);
  get_parameter("fixed_frame", fixed_frame_);
  get_parameter("backwards", backwards_);
  get_parameter("safe_distance", safe_distance_);
  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

  vel_publisher_ = std::make_unique<nav2_util::TwistPublisher>(node, "cmd_vel", 1);
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

  double action_server_result_timeout;
  nav2_util::declare_parameter_if_not_declared(
    node, "action_server_result_timeout", rclcpp::ParameterValue(10.0));
  get_parameter("action_server_result_timeout", action_server_result_timeout);
  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);

  // Create the action server for dynamic following
  following_action_server_ = std::make_unique<FollowingActionServer>(
    node, "follow_object",
    std::bind(&FollowingServer::followObject, this),
    nullptr, std::chrono::milliseconds(500),
    true, server_options);

  // Create composed utilities
  controller_ = std::make_unique<opennav_docking::Controller>(node);

  // Setup filter
  double filter_coef;
  get_parameter("filter_coef", filter_coef);
  filter_ = std::make_unique<opennav_docking::PoseFilter>(filter_coef, detection_timeout_);

  // Subscribe to dynamic pose
  dynamic_pose_.header.stamp = rclcpp::Time(0);
  dynamic_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "detected_dynamic_pose", 1,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
      detected_dynamic_pose_ = *pose;
    });

  filtered_dynamic_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "filtered_dynamic_pose", 1);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
FollowingServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating %s", get_name());

  auto node = shared_from_this();

  vel_publisher_->on_activate();
  filtered_dynamic_pose_pub_->on_activate();
  following_action_server_->activate();

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&FollowingServer::dynamicParametersCallback, this, _1));

  // Create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
FollowingServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s", get_name());

  following_action_server_->deactivate();
  filtered_dynamic_pose_pub_->on_deactivate();
  vel_publisher_->on_deactivate();

  dyn_params_handler_.reset();
  tf2_listener_.reset();
  tf2_buffer_.reset();

  // Destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
FollowingServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s", get_name());
  following_action_server_.reset();
  controller_.reset();
  vel_publisher_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
FollowingServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down %s", get_name());
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename ActionT>
void FollowingServer::getPreemptedGoalIfRequested(
  typename std::shared_ptr<const typename ActionT::Goal> goal,
  const std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename ActionT>
bool FollowingServer::checkAndWarnIfCancelled(
  std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server,
  const std::string & name)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_WARN(get_logger(), "Goal was cancelled. Cancelling %s action", name.c_str());
    return true;
  }
  return false;
}

template<typename ActionT>
bool FollowingServer::checkAndWarnIfPreempted(
  std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server,
  const std::string & name)
{
  if (action_server->is_preempt_requested()) {
    RCLCPP_WARN(get_logger(), "Goal was preempted. Cancelling %s action", name.c_str());
    return true;
  }
  return false;
}

void FollowingServer::followObject()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  action_start_time_ = this->now();
  rclcpp::Rate loop_rate(controller_frequency_);

  auto goal = following_action_server_->get_current_goal();
  auto result = std::make_shared<FollowObjectAction::Result>();

  if (!following_action_server_ || !following_action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return;
  }

  if (checkAndWarnIfCancelled(following_action_server_, "follow_object")) {
    following_action_server_->terminate_all();
    return;
  }

  getPreemptedGoalIfRequested(goal, following_action_server_);

  try {
    RCLCPP_INFO(
      get_logger(),
      "Attempting to follow object at position (%0.2f, %0.2f).",
      goal->object_pose.pose.position.x, goal->object_pose.pose.position.y);

    // Construct initial estimate of where the object is located in fixed_frame
    geometry_msgs::msg::PoseStamped object_pose = goal->object_pose;
    tf2_buffer_->transform(object_pose, object_pose, fixed_frame_);

    // Get initial detection of the object before proceeding to move
    doInitialPerception(object_pose);
    RCLCPP_INFO(get_logger(), "Successful initial object detection");

    // Following control loop: while not timeout, run controller
    auto start = this->now();
    rclcpp::Duration timeout = goal->max_duration;
    while (rclcpp::ok()) {
      try {
        // Approach the object using control law
        if (approachObject(object_pose)) {
          // We have reached the object, check if we need to approach again
          if (this->now() - start > timeout && timeout.seconds() > 0.0) {
            RCLCPP_INFO(get_logger(), "Reached object, but timed out");
            result->error_code = FollowObjectAction::Result::TIMEOUT;
            result->total_elapsed_time = this->now() - action_start_time_;
            publishZeroVelocity();
            following_action_server_->succeeded_current(result);
            return;
          }
        }

        // Cancelled, preempted, or shutting down (recoverable errors throw Exception)
        result->total_elapsed_time = this->now() - action_start_time_;
        publishZeroVelocity();
        following_action_server_->terminate_all(result);
        return;
      } catch (std::exception & e) {
        RCLCPP_WARN(get_logger(), "Following failed, will retry: %s", e.what());
      }
    }
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "Transform error: %s", e.what());
    result->error_code = FollowObjectAction::Result::TF_ERROR;
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = FollowObjectAction::Result::UNKNOWN;
  }

  // Stop the robot and report
  publishZeroVelocity();
  following_action_server_->terminate_current(result);
}

void FollowingServer::doInitialPerception(geometry_msgs::msg::PoseStamped & object_pose)
{
  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(initial_perception_timeout_);
  while (!getRefinedPose(object_pose)) {
    if (this->now() - start > timeout) {
      // TODO(ajtudela): Throw exception?
      RCLCPP_ERROR(get_logger(), "Failed initial goal detection");
    }

    if (checkAndWarnIfCancelled(following_action_server_, "follow_object") ||
      checkAndWarnIfPreempted(following_action_server_, "follow_object"))
    {
      return;
    }

    loop_rate.sleep();
  }
}

bool FollowingServer::approachObject(geometry_msgs::msg::PoseStamped & object_pose)
{
  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(object_approach_timeout_);
  while (rclcpp::ok()) {
    publishFollowingFeedback();

    // Stop and report success if reached goal
    auto approach_pose = getBackwardPose(object_pose, safe_distance_);
    if (isGoalReached(approach_pose)) {
      return true;
    }

    // Stop if cancelled/preempted
    if (checkAndWarnIfCancelled(following_action_server_, "follow_object") ||
      checkAndWarnIfPreempted(following_action_server_, "follow_object"))
    {
      return false;
    }

    // Update perception of object
    if (!getRefinedPose(object_pose)) {
      // TODO(ajtudela): Throw exception?
      RCLCPP_ERROR(get_logger(), "Failed object detection");
      return false;
    }

    // Transform target_pose into base_link frame
    geometry_msgs::msg::PoseStamped target_pose = object_pose;
    target_pose.header.stamp = rclcpp::Time(0);

    // The control law can get jittery when close to the end when atan2's can explode.
    // Thus, we backward project the controller's target pose a little bit after the
    // object so that the robot never gets to the end of the spiral before its in contact
    // with the object to stop the following procedure.
    const double backward_projection = 0.25;
    const double yaw = tf2::getYaw(target_pose.pose.orientation);
    target_pose.pose.position.x += cos(yaw) * backward_projection;
    target_pose.pose.position.y += sin(yaw) * backward_projection;
    tf2_buffer_->transform(target_pose, target_pose, base_frame_);

    // Compute and publish controls
    auto command = std::make_unique<geometry_msgs::msg::TwistStamped>();
    command->header.stamp = now();
    if (!controller_->computeVelocityCommand(target_pose.pose, command->twist, backwards_)) {
      // TODO(ajtudela): Throw exception?
      RCLCPP_ERROR(get_logger(), "Failed to get control");
      return false;
    }
    vel_publisher_->publish(std::move(command));

    if (this->now() - start > timeout) {
      // TODO(ajtudela): Throw exception?
      RCLCPP_ERROR(get_logger(), "Timed out approaching object");
      return false;
    }

    loop_rate.sleep();
  }
  return false;
}

geometry_msgs::msg::PoseStamped FollowingServer::getRobotPoseInFrame(const std::string & frame)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = base_frame_;
  robot_pose.header.stamp = rclcpp::Time(0);
  tf2_buffer_->transform(robot_pose, robot_pose, frame);
  return robot_pose;
}

bool FollowingServer::getRefinedPose(geometry_msgs::msg::PoseStamped & pose)
{
  // Get current detections and transform to frame
  geometry_msgs::msg::PoseStamped detected = detected_dynamic_pose_;

  // Validate that external pose is new enough
  auto timeout = rclcpp::Duration::from_seconds(detection_timeout_);
  if (this->now() - detected.header.stamp > timeout) {
    RCLCPP_WARN(this->get_logger(), "Lost detection or did not detect: timeout exceeded");
    return false;
  }

  // Transform detected pose into fixed frame. Note that the argument pose
  // is the output of detection, but also acts as the initial estimate
  // and contains the frame_id of the detection.
  if (detected.header.frame_id != pose.header.frame_id) {
    try {
      if (!tf2_buffer_->canTransform(
          pose.header.frame_id, detected.header.frame_id,
          detected.header.stamp, rclcpp::Duration::from_seconds(transform_tolerance_)))
      {
        RCLCPP_WARN(this->get_logger(), "Failed to transform detected object pose");
        return false;
      }
      tf2_buffer_->transform(detected, detected, pose.header.frame_id);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Failed to transform detected object pose");
      return false;
    }
  }

  // Filter the detected pose
  detected = filter_->update(detected);
  filtered_dynamic_pose_pub_->publish(detected);

  // Construct dynamic_pose_
  dynamic_pose_.header = detected.header;
  dynamic_pose_.pose = detected.pose;

  // Return dynamic pose for debugging purposes
  pose = dynamic_pose_;
  return true;
}

void FollowingServer::publishZeroVelocity()
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.stamp = now();
  vel_publisher_->publish(std::move(cmd_vel));
}

void FollowingServer::publishFollowingFeedback()
{
  auto feedback = std::make_shared<FollowObjectAction::Feedback>();
  feedback->following_time = this->now() - action_start_time_;
  following_action_server_->publish_feedback(feedback);
}

geometry_msgs::msg::PoseStamped FollowingServer::getBackwardPose(
  const geometry_msgs::msg::PoseStamped & pose, double distance)
{
  geometry_msgs::msg::PoseStamped backward_pose = pose;
  const double yaw = tf2::getYaw(backward_pose.pose.orientation);
  backward_pose.pose.position.x -= distance * cos(yaw);
  backward_pose.pose.position.y -= distance * sin(yaw);
  return backward_pose;
}

bool FollowingServer::isGoalReached(const geometry_msgs::msg::PoseStamped & goal_pose)
{
  double linear_tolerance = 0.25;
  double angular_tolerance = 0.25;
  geometry_msgs::msg::PoseStamped robot_pose = getRobotPoseInFrame(goal_pose.header.frame_id);
  const double dist = std::hypot(
    robot_pose.pose.position.x - goal_pose.pose.position.x,
    robot_pose.pose.position.y - goal_pose.pose.position.y);
  const double yaw = angles::shortest_angular_distance(
    tf2::getYaw(robot_pose.pose.orientation), tf2::getYaw(goal_pose.pose.orientation));
  return dist < linear_tolerance && abs(yaw) < angular_tolerance;
}

rcl_interfaces::msg::SetParametersResult
FollowingServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "controller_frequency") {
        controller_frequency_ = parameter.as_double();
      } else if (name == "initial_perception_timeout") {
        initial_perception_timeout_ = parameter.as_double();
      } else if (name == "object_approach_timeout") {
        object_approach_timeout_ = parameter.as_double();
      } else if (name == "transform_tolerance") {
        transform_tolerance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == "base_frame") {
        base_frame_ = parameter.as_string();
      } else if (name == "fixed_frame") {
        fixed_frame_ = parameter.as_string();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace opennav_following

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(opennav_following::FollowingServer)
