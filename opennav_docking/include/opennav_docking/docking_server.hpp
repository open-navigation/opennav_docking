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

#ifndef OPENNAV_DOCKING__DOCKING_SERVER_HPP_
#define OPENNAV_DOCKING__DOCKING_SERVER_HPP_

#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "opennav_docking/utils.hpp"
#include "opennav_docking/types.hpp"
#include "opennav_docking/dock_database.hpp"
#include "opennav_docking_core/charging_dock.hpp"

namespace opennav_docking
{
/**
 * @class opennav_docking::DockingServer
 * @brief An action server which implements charger docking node for AMRs
 */
class DockingServer : public nav2_util::LifecycleNode
{
public:
  using DockingActionServer = nav2_util::SimpleActionServer<DockRobot>;
  using UndockingActionServer = nav2_util::SimpleActionServer<UndockRobot>;

  /**
   * @brief A constructor for opennav_docking::DockingServer
   * @param options Additional options to control creation of the node.
   */
  explicit DockingServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief A destructor for opennav_docking::DockingServer
   */
  ~DockingServer() = default;

protected:
  /**
   * @brief Main action callback method to complete docking request
   */
  void dockRobot();

  /**
   * @brief Main action callback method to complete undocking request
   */
  void undockRobot();

  /**
   * @brief Gets a preempted goal if immediately requested
   * @param Goal goal to check or replace if required with preemption
   * @param action_server Action server to check for preemptions on
   * @return SUCCESS or FAILURE
   */
  template <typename ActionT>
  void getPreemptedGoalIfRequested(
    typename std::shared_ptr<const typename ActionT::Goal> goal,
    const std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server);

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
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

  std::unique_ptr<DockingActionServer> docking_action_server_;
  std::unique_ptr<UndockingActionServer> undocking_action_server_;

  std::unique_ptr<DockDatabase> dock_db_;
  std::string curr_dock_type_;
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__DOCKING_SERVER_HPP_
