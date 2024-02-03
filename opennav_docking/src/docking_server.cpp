// Copyright (c) 2024 Open Navigation LLC
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

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "opennav_docking/docking_server.hpp"
#include "opennav_docking/graceful_controller.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace opennav_docking
{

DockingServer::DockingServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("docking_server", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating %s", get_name());

  declare_parameter("controller_frequency", 20.0);
  declare_parameter("initial_perception_timeout", 2.0);
  declare_parameter("wait_charge_timeout", 5.0);
  declare_parameter("undock_tolerance", 0.1);
  declare_parameter("max_retries", 3);
  declare_parameter("base_frame", "base_link");
  declare_parameter("fixed_frame", "odom");
}

nav2_util::CallbackReturn
DockingServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring %s", get_name());
  auto node = shared_from_this();

  get_parameter("controller_frequency", controller_frequency_);
  get_parameter("initial_perception_timeout", initial_perception_timeout_);
  get_parameter("wait_charge_timeout", wait_charge_timeout_);
  get_parameter("undock_tolerance", undock_tolerance_);
  get_parameter("max_retries", max_retries_);
  get_parameter("base_frame", base_frame_);
  get_parameter("fixed_frame", fixed_frame_);
  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  // Setup TF2
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

  navigator_ = std::make_unique<Navigator>(node);
  dock_db_ = std::make_unique<DockDatabase>();
  if (!dock_db_->initialize(node, tf2_buffer_)) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // Create a controller
  controller_ = std::make_unique<GracefulController>();
  controller_->activate();

  double action_server_result_timeout;
  nav2_util::declare_parameter_if_not_declared(
    node, "action_server_result_timeout", rclcpp::ParameterValue(10.0));
  get_parameter("action_server_result_timeout", action_server_result_timeout);
  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);

  // Create the action servers for path planning to a pose and through poses
  docking_action_server_ = std::make_unique<DockingActionServer>(
    node, "dock_robot",
    std::bind(&DockingServer::dockRobot, this),
    nullptr, std::chrono::milliseconds(500),
    true, server_options);

  undocking_action_server_ = std::make_unique<UndockingActionServer>(
    node, "undock_robot",
    std::bind(&DockingServer::undockRobot, this),
    nullptr, std::chrono::milliseconds(500),
    true, server_options);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating %s", get_name());

  auto node = shared_from_this();

  dock_db_->activate();
  navigator_->activate();
  vel_publisher_->on_activate();
  docking_action_server_->activate();
  undocking_action_server_->activate();
  curr_dock_type_.clear();

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&DockingServer::dynamicParametersCallback, this, _1));

  // Create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s", get_name());

  docking_action_server_->deactivate();
  undocking_action_server_->deactivate();
  dock_db_->deactivate();
  navigator_->deactivate();
  vel_publisher_->on_deactivate();

  dyn_params_handler_.reset();
  tf2_listener_.reset();
  tf2_buffer_.reset();

  // Destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s", get_name());
  docking_action_server_.reset();
  undocking_action_server_.reset();
  dock_db_.reset();
  navigator_.reset();
  curr_dock_type_.clear();
  vel_publisher_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down %s", get_name());
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename ActionT>
void DockingServer::getPreemptedGoalIfRequested(
  typename std::shared_ptr<const typename ActionT::Goal> goal,
  const std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

void DockingServer::dockRobot()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  action_start_time_ = this->now();

  rclcpp::Rate loop_rate(controller_frequency_);

  auto goal = docking_action_server_->get_current_goal();
  auto result = std::make_shared<DockRobot::Result>();

  if (!docking_action_server_ || !docking_action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return;
  }

  if (docking_action_server_->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling docking action.");
    docking_action_server_->terminate_all();
    return;
  }

  getPreemptedGoalIfRequested(goal, docking_action_server_);
  Dock * dock{nullptr};
  bool succeeded = false;
  num_retries_ = 0;

  try {
    // Get dock (instance and plugin information) from request
    if (goal->use_dock_id) {
      RCLCPP_INFO(
        get_logger(),
        "Attempting to dock robot at charger %s.", goal->dock_id.c_str());
      dock = dock_db_->findDock(goal->dock_id);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Attempting to dock robot at charger at position (%0.2f, %0.2f).",
        goal->dock_pose.pose.position.x, goal->dock_pose.pose.position.y);

      dock = new Dock();
      dock->frame = goal->dock_pose.header.frame_id;
      dock->pose = goal->dock_pose.pose;
      dock->type = goal->dock_type;
      dock->plugin = dock_db_->findDockPlugin(dock->type);
    }

    // Send robot to its staging pose
    publishDockingFeedback(DockRobot::Feedback::NAV_TO_STAGING_POSE);
    navigator_->goToPose(
      dock->getStagingPose(), rclcpp::Duration::from_seconds(goal->max_staging_time));
    RCLCPP_INFO(get_logger(), "Successful navigation to staging pose");

    // Construct initial estimate of where the dock is located
    geometry_msgs::msg::PoseStamped dock_pose;
    dock_pose.pose = dock->pose;
    dock_pose.header.frame_id = dock->frame;

    // Get initial detection of dock before proceeding to move
    doInitialPerception(dock, dock_pose);
    RCLCPP_INFO(get_logger(), "Successful initial dock detection");

    // Docking control loop: while not docked, run controller
    rclcpp::Time dock_contact_time;
    while (rclcpp::ok() && (num_retries_ < max_retries_)) {
      // Approach the dock using control law
      if (!approachDock(dock, dock_pose)) {
        // Failed to approach
        break;
      }

      // Wait for charging to begin
      RCLCPP_INFO(get_logger(), "Made contact with dock, waiting for charge to start");
      if (waitForCharge(dock)) {
        RCLCPP_INFO(get_logger(), "Robot is charging!");
        succeeded = true;
        docking_action_server_->succeeded_current(result);
        break;
      }

      // Reset to staging pose to try again
      RCLCPP_WARN(get_logger(), "Charging did not start - retrying");
      ++num_retries_;
      if (!resetApproach(dock->getStagingPose())) {
        // Failed to reset?
        break;
      }
      RCLCPP_INFO(get_logger(), "Returned to staging, attempting docking again");
    }
  } catch (opennav_docking_core::DockNotInDB & e) {
    RCLCPP_ERROR(get_logger(), "Invalid mode set: %s", e.what());
    result->error_code = DockRobot::Result::DOCK_NOT_IN_DB;
  } catch (opennav_docking_core::DockNotValid & e) {
    RCLCPP_ERROR(get_logger(), "Invalid mode set: %s", e.what());
    result->error_code = DockRobot::Result::DOCK_NOT_VALID;
  } catch (opennav_docking_core::FailedToStage & e) {
    RCLCPP_ERROR(get_logger(), "Invalid mode set: %s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_STAGE;
  } catch (opennav_docking_core::FailedToDetectDock & e) {
    RCLCPP_ERROR(get_logger(), "Invalid mode set: %s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_DETECT_DOCK;
  } catch (opennav_docking_core::FailedToControl & e) {
    RCLCPP_ERROR(get_logger(), "Invalid mode set: %s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CONTROL;
  } catch (opennav_docking_core::FailedToCharge & e) {
    RCLCPP_ERROR(get_logger(), "Invalid mode set: %s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CHARGE;
  } catch (opennav_docking_core::DockingException & e) {
    RCLCPP_ERROR(get_logger(), "Invalid mode set: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Internal error: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  }

  // Store dock state for later undocking and delete temp dock, if applicable
  if (dock) {
    curr_dock_type_ = dock->type;
  }

  if (!goal->use_dock_id && dock) {
    delete dock;
  }

  if (!succeeded) {
    docking_action_server_->terminate_current(result);
  }
}

void DockingServer::publishDockingFeedback(uint16_t state)
{
  auto feedback = std::make_shared<DockRobot::Feedback>();
  feedback->state = state;
  feedback->docking_time = this->now() - action_start_time_;
  feedback->num_retries = num_retries_;
  docking_action_server_->publish_feedback(feedback);
}

bool DockingServer::doInitialPerception(Dock * dock, geometry_msgs::msg::PoseStamped & dock_pose)
{
  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(initial_perception_timeout_);
  while (!dock->plugin->getRefinedPose(dock_pose)) {
    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToDetectDock("Failed initial dock detection");
    }
    publishDockingFeedback(DockRobot::Feedback::INITIAL_PERCEPTION);
    loop_rate.sleep();
  }
  return true;
}

bool DockingServer::approachDock(Dock * dock, geometry_msgs::msg::PoseStamped & dock_pose)
{
  rclcpp::Rate loop_rate(controller_frequency_);
  while (rclcpp::ok()) {
    // Allocate new zero velocity command
    geometry_msgs::msg::Twist command;

    // Stop and report success if connected to dock
    if (dock->plugin->isDocked()) {
      return true;
    }

    // Stop if cancelled/preempted
    if (docking_action_server_->is_cancel_requested()) {
      RCLCPP_WARN(get_logger(), "Goal was canceled. Canceling docking action.");
      // Publish zero velocity before breaking out of loop
      vel_publisher_->publish(command);
      return false;
    }

    // Update perception
    if (!dock->plugin->getRefinedPose(dock_pose)) {
      // Publish zero velocity before breaking out of loop
      vel_publisher_->publish(command);
      throw opennav_docking_core::FailedToDetectDock("Failed dock detection");
    }

    // Transform target_pose into base_link frame
    // TODO(fergs): this should be pushed into the controller
    geometry_msgs::msg::PoseStamped target_pose = dock_pose;
    try {
      target_pose.header.stamp = rclcpp::Time(0);
      tf2_buffer_->transform(target_pose, target_pose, base_frame_);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "Could not transform pose");
      // Publish zero velocity before breaking out of loop
      vel_publisher_->publish(command);
      return false;
    }

    if (!controller_->computeVelocityCommand(target_pose.pose, command)) {
      throw opennav_docking_core::FailedToControl("Failed to get control");
    }
    vel_publisher_->publish(command);

    publishDockingFeedback(DockRobot::Feedback::CONTROLLING);
    loop_rate.sleep();
  }
  return false;
}

bool DockingServer::waitForCharge(Dock * dock)
{
  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(wait_charge_timeout_);
  while (rclcpp::ok()) {
    if (dock->plugin->isCharging()) {
      return true;
    }

    if (this->now() - start > timeout) {
      return false;
    }

    publishDockingFeedback(DockRobot::Feedback::WAIT_FOR_CHARGE);
    loop_rate.sleep();
  }
  return false;
}

bool DockingServer::resetApproach(geometry_msgs::msg::PoseStamped staging_pose)
{
  rclcpp::Rate loop_rate(controller_frequency_);
  while (rclcpp::ok()) {
    // Allocate new zero velocity command
    geometry_msgs::msg::Twist command;

    // Stop if cancelled/preempted
    if (docking_action_server_->is_cancel_requested()) {
      RCLCPP_WARN(get_logger(), "Goal was canceled. Canceling docking action.");
      // Publish zero velocity before breaking out of loop
      vel_publisher_->publish(command);
      return false;
    }

    // Determine if we have reached staging_pose yet
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPoseInFrame(robot_pose, staging_pose.header.frame_id)) {
      // Unable to approach pose due to TF error
      vel_publisher_->publish(command);
      return false;
    }

    double dist = std::hypot(
      robot_pose.pose.position.x - staging_pose.pose.position.x,
      robot_pose.pose.position.y - staging_pose.pose.position.y);
    if (dist < undock_tolerance_) {
      // Retry backup is complete - publish zero velocity and try to approach dock again
      vel_publisher_->publish(command);
      return true;
    }

    // Transform target_pose into base_link frame
    // TODO(fergs): this should be pushed into the controller
    geometry_msgs::msg::PoseStamped target_pose = staging_pose;
    try {
      target_pose.header.stamp = rclcpp::Time(0);
      tf2_buffer_->transform(target_pose, target_pose, base_frame_);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "Could not transform pose");
      // Publish zero velocity before breaking out of loop
      vel_publisher_->publish(command);
      return false;
    }

    if (!controller_->computeVelocityCommand(target_pose.pose, command)) {
      throw opennav_docking_core::FailedToControl("Failed to get control");
    }

    // Publish command and feedback, then sleep
    vel_publisher_->publish(command);
    publishDockingFeedback(DockRobot::Feedback::RETRY);
    loop_rate.sleep();
  }
  return false;
}

bool DockingServer::getRobotPoseInFrame(
  geometry_msgs::msg::PoseStamped & robot_pose, const std::string & frame)
{
  robot_pose.header.frame_id = base_frame_;
  robot_pose.header.stamp = rclcpp::Time(0);
  try {
    tf2_buffer_->transform(robot_pose, robot_pose, frame);
  } catch (const tf2::TransformException & ex) {
    return false;
  }
  return true;
}

void DockingServer::undockRobot()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  action_start_time_ = this->now();

  rclcpp::Rate loop_rate(controller_frequency_);

  auto goal = undocking_action_server_->get_current_goal();
  auto result = std::make_shared<UndockRobot::Result>();

  if (!undocking_action_server_ || !undocking_action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return;
  }

  if (undocking_action_server_->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling undocking action.");
    undocking_action_server_->terminate_all();
    return;
  }

  getPreemptedGoalIfRequested(goal, undocking_action_server_);
  auto max_duration = rclcpp::Duration::from_seconds(goal->max_undocking_time);

  try {
    // Get dock plugin information from request or docked state, reset state.
    std::string dock_type = curr_dock_type_;
    if (!goal->dock_type.empty()) {
      dock_type = goal->dock_type;
    }

    ChargingDock::Ptr dock = dock_db_->findDockPlugin(dock_type);
    if (!dock) {
      throw opennav_docking_core::DockNotValid("No dock information to undock from!");
    }
    RCLCPP_INFO(
      get_logger(),
      "Attempting to undock robot from charger of type %s.", dock->getName().c_str());

    // Get "dock pose" by finding the robot pose
    geometry_msgs::msg::PoseStamped dock_pose;
    dock_pose.header.frame_id = base_frame_;
    dock_pose.header.stamp = rclcpp::Time(0);
    try {
      tf2_buffer_->transform(dock_pose, dock_pose, fixed_frame_);
    } catch (const tf2::TransformException & ex) {
      throw opennav_docking_core::DockNotValid("Could not transform dock pose!");
    }

    // TODO(fergs): Check if path to undock is clear

    // Control robot to staging pose
    geometry_msgs::msg::PoseStamped staging_pose =
      dock->getStagingPose(dock_pose.pose, dock_pose.header.frame_id);
    rclcpp::Time loop_start = this->now();
    while (rclcpp::ok()) {
      // Command to send to robot base
      geometry_msgs::msg::Twist command;

      // Stop if we exceed max duration
      auto timeout = rclcpp::Duration::from_seconds(goal->max_undocking_time);
      if (this->now() - loop_start > timeout) {
        // Publish zero velocity before breaking out of loop
        vel_publisher_->publish(command);
        throw opennav_docking_core::FailedToControl("Undocking timed out");
      }

      // Don't control the robot until charging is disabled
      if (!dock->disableCharging()) {
        loop_rate.sleep();
        continue;
      }

      // Stop controlling when pose reached
      {
        geometry_msgs::msg::PoseStamped robot_pose;
        robot_pose.header.frame_id = base_frame_;
        robot_pose.header.stamp = rclcpp::Time(0);
        try {
          tf2_buffer_->transform(robot_pose, robot_pose, fixed_frame_);

          double dist = std::hypot(
            robot_pose.pose.position.x - staging_pose.pose.position.x,
            robot_pose.pose.position.y - staging_pose.pose.position.y);

          if (dist < undock_tolerance_ && dock->hasStoppedCharging()) {
            RCLCPP_INFO(get_logger(), "Robot has undocked!");
            undocking_action_server_->succeeded_current(result);
            curr_dock_type_.clear();
            // Publish zero velocity before breaking out of loop
            vel_publisher_->publish(command);
            return;
          }
        } catch (const tf2::TransformException & ex) {
          RCLCPP_ERROR(get_logger(), "Could not transform robot pose");
        }
      }

      // Also stop if cancelled/preempted
      if (undocking_action_server_->is_cancel_requested()) {
        RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling docking action.");
        // Publish zero velocity before breaking out of loop
        vel_publisher_->publish(command);
        break;
      }

      // Transform staging_pose into base_link frame
      geometry_msgs::msg::PoseStamped target_pose;
      try {
        staging_pose.header.stamp = rclcpp::Time(0);
        tf2_buffer_->transform(staging_pose, target_pose, base_frame_);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(get_logger(), "Could not transform pose");
      }

      // Run controller to get command velocity
      if (!controller_->computeVelocityCommand(target_pose.pose, command)) {
        throw opennav_docking_core::FailedToControl("Failed to get control");
      }

      // Publish command and sleep
      vel_publisher_->publish(command);
      loop_rate.sleep();
    }
  } catch (opennav_docking_core::DockNotValid & e) {
    RCLCPP_ERROR(get_logger(), "Invalid mode set: %s", e.what());
    result->error_code = DockRobot::Result::DOCK_NOT_VALID;
  } catch (opennav_docking_core::FailedToControl & e) {
    RCLCPP_ERROR(get_logger(), "Invalid mode set: %s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CONTROL;
  } catch (opennav_docking_core::DockingException & e) {
    RCLCPP_ERROR(get_logger(), "Invalid mode set: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Internal error: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  }

  undocking_action_server_->terminate_current(result);
}

rcl_interfaces::msg::SetParametersResult
DockingServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  rcl_interfaces::msg::SetParametersResult result;
  // for (auto parameter : parameters) {
  //   const auto & type = parameter.get_type();
  //   const auto & name = parameter.get_name();

  // TODO(XYZ): implement dynamic parameters for all applicable parameters
  // }
  (void)parameters;

  result.successful = true;
  return result;
}

}  // namespace opennav_docking

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(opennav_docking::DockingServer)
