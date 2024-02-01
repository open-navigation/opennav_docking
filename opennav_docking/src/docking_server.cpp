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
}

nav2_util::CallbackReturn
DockingServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring %s", get_name());
  auto node = shared_from_this();

  // TODO(XYZ): get parameters, construct objects

  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  navigator_ = std::make_unique<Navigator>(node);
  dock_db_ = std::make_unique<DockDatabase>();
  if (!dock_db_->initialize(node, tf2_buffer_)) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // Create a controller
  controller_ = std::make_unique<GracefulController>();

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

  // Setup TF2
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

  dock_db_->activate();
  navigator_->activate();
  docking_action_server_->activate();
  undocking_action_server_->activate();
  vel_publisher_->on_activate();
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
  auto start_time = this->now();

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

  try {
    // (1) Get dock (instance and plugin information) from request
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

    // (2) Send robot to its staging pose
    navigator_->goToPose(
      dock->getStagingPose(), rclcpp::Duration::from_seconds(goal->max_staging_time));

    // Construct initial estimate of where the dock is located
    geometry_msgs::msg::PoseStamped dock_pose;
    dock_pose.pose = dock->pose;
    dock_pose.header.frame_id = dock->frame;

    // Get initial detection of dock before proceeding to move
    while (!dock->plugin->getRefinedPose(dock_pose)) {
      // TODO(fergs): add timeout
    }

    // Docking control loop: while not docked, run controller
    while (rclcpp::ok()) {
      // Stop controlling when successfully charging
      if (dock->plugin->isCharging()) {
        RCLCPP_INFO(get_logger(), "Robot is charging!");
        break;
      }

      // Update perception
      if (!dock->plugin->getRefinedPose(dock_pose)) {
        // TODO(fergs): handle loss of perception
      }

      // Use the dock pose to determine where to put the robot base
      geometry_msgs::msg::PoseStamped target_pose = dock->plugin->getTargetPose(dock_pose);

      // Transform target_pose into base_link frame
      geometry_msgs::msg::PoseStamped target_in_base;
      try {
        target_pose.header.stamp = rclcpp::Time(0);
        tf2_buffer_->transform(target_pose, target_in_base, "base_link");
      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(get_logger(), "Could not transform pose");
      }

      // Run controller
      geometry_msgs::msg::Twist command;
      if (!controller_->computeVelocityCommand(target_in_base.pose, command)) {
        // If controller has reached/failed goal but we are not yet charging, retry
        // TODO(fergs)
      }

      // Publish command
      vel_publisher_->publish(command);
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
    // TODO(fergs): use this for failure contextual exception
    RCLCPP_ERROR(get_logger(), "Invalid mode set: %s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_DETECT_DOCK;
  } catch (opennav_docking_core::FailedToControl & e) {
    // TODO(fergs): use this for failure contextual exception
    RCLCPP_ERROR(get_logger(), "Invalid mode set: %s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CONTROL;
  } catch (opennav_docking_core::FailedToCharge & e) {
    // TODO(fergs): use this for failure contextual exception
    RCLCPP_ERROR(get_logger(), "Invalid mode set: %s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CHARGE;
  } catch (opennav_docking_core::DockingException & e) {
    RCLCPP_ERROR(get_logger(), "Invalid mode set: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Internal error: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  }

  // (7) Store dock state for later undocking and delete temp dock, if applicable
  if (dock) {
    curr_dock_type_ = dock->type;
  }

  if (!goal->use_dock_id && dock) {
    delete dock;
  }

  docking_action_server_->terminate_current(result);
}

void DockingServer::undockRobot()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  auto start_time = this->now();

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
    // (1) Get dock  plugin information from request or docked state, reset state.
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

    // (2) Check if path to undock is clear  TODO(fergs)

    // (3) TODO(fergs): Send robot to its staging pose, asked dock API (controller).
    //                  check max_duration.
    // (2.0) Make sure docking relative pose in right frame
    //       (docked pose -> staging pose, not dock itself pose)
    // (2.1) In loop, check that we are no longer charging in state

    // (4) return charge level TODO(fergs)
  } catch (opennav_docking_core::DockNotValid & e) {
    RCLCPP_ERROR(get_logger(), "Invalid mode set: %s", e.what());
    result->error_code = DockRobot::Result::DOCK_NOT_VALID;
  } catch (opennav_docking_core::FailedToControl & e) {
    // TODO(fergs): use this for failure contextual exception
    RCLCPP_ERROR(get_logger(), "Invalid mode set: %s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CONTROL;
  } catch (opennav_docking_core::DockingException & e) {
    // TODO(fergs): use this for failure contextual exception
    RCLCPP_ERROR(get_logger(), "Invalid mode set: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Internal error: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  }

  // (5) Reset docking state
  curr_dock_type_.clear();

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
