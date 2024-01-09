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

#include "opennav_docking/docking_server.hpp"

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

  // TODO(XYZ): get parameters, construct objects, plugins

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
  docking_action_server_->activate();
  undocking_action_server_->activate();

  // Add callback for dynamic parameters
  auto node = shared_from_this();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&DockingServer::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s", get_name());
  docking_action_server_->deactivate();
  undocking_action_server_->deactivate();
  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s", get_name());
  docking_action_server_.reset();
  undocking_action_server_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down %s", get_name());
  return nav2_util::CallbackReturn::SUCCESS;
}

template <typename ActionT>
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

  // try {
    // (0) Get dock from request and find its plugin. If only 1 dock type special case


    // if (!/*cannot correlate */) {
    //   throw invalid dock;
    // }

    // if (/*dock name*/) {
    //   RCLCPP_INFO(
    //     get_logger(),
    //     "Attempting to dock robot at charger %s.", );

    // } else { // Dock pose
    //   RCLCPP_INFO(
    //     get_logger(),
    //     "Attempting to dock robot at charger at pose (%0.2f, %0.2f, %0.2f).", );
    // }

    // (1) Send robot to its staging pose

    // (2) Detect dock & docking pose using sensors (TODO process for dead reckoning too)

    // (3) Fergs: main loop here





    // (4) Wait for contact to conduct charging (TODO process to enable charging if not automatic)

    // (5) Check if contact is made. Yes -> success. No -> retry to limit going back to (1).



  } catch (DockingException & e) {  // TODO(sm): set contextual error codes + number range
    RCLCPP_ERROR(get_logger(), "Invalid mode set: %s", e.what());
    result->error_code = DockRobot::Result::INVALID_MODE_SET;
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Internal error: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  }

  // TODO store dock information for undocking
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

  // try {
    // (0) Get dock info from dock request (todo what if starting docked? -> default relative staging pose?) If only 1 dock type special case


    // if (!/*cannot correlate */) {
    //   throw invalid dock;
    // }

    // if (/*dock name*/) {
    //   RCLCPP_INFO(
    //     get_logger(),
    //     "Attempting to undock robot at charger %s.", );

    // } else { // Dock pose
    //   RCLCPP_INFO(
    //     get_logger(),
    //     "Attempting to undock robot at charger at pose (%0.2f, %0.2f, %0.2f).", );
    // }

    // (1) Check if path to undock is clear 
  
    // (2) Send robot to its staging pose (fergs, undocking controller)

      // (2.1) In loop, check that we are no longer charging in state

    // (3) return charge level


  // } catch (DockingException & e) {  // TODO(sm): set contextual error codes+ number range
  //   RCLCPP_ERROR(get_logger(), "Invalid mode set: %s", e.what());
  //   result->error_code = DockRobot::Result::INVALID_MODE_SET;
  // } catch (std::exception & e) {
  //   RCLCPP_ERROR(get_logger(), "Internal Fields2Cover error: %s", e.what());
  //   result->error_code = DockRobot::Result::UNKNOWN;
  // }

  undocking_action_server_->terminate_current(result);
}


rcl_interfaces::msg::SetParametersResult
DockingServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  rcl_interfaces::msg::SetParametersResult result;
  // for (auto parameter : parameters) {
    // const auto & type = parameter.get_type();
    // const auto & name = parameter.get_name();

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
