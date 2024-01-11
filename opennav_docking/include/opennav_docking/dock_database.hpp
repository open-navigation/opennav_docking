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

#ifndef OPENNAV_DOCKING__DOCK_DATABASE_HPP_
#define OPENNAV_DOCKING__DOCK_DATABASE_HPP_

#include <vector>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "opennav_docking/utils.hpp"
#include "opennav_docking/types.hpp"

namespace opennav_docking
{
/**
 * @class opennav_docking::DockDatabase
 * @brief An object to contain docks and docking plugins
 */
class DockDatabase
{
public:
  /**
   * @brief A constructor for opennav_docking::DockDatabase
   */
  DockDatabase();

  /**
   * @brief A setup function to populate database
   * @param parent Weakptr to the node to use to get interances and parameters
   */
  bool initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent);

  /**
   * @brief A destructor for opennav_docking::DockDatabase
   */
  ~DockDatabase();

  /**
   * @brief An activation method
   */
  void activate();

  /**
   * @brief An deactivation method
   */
  void deactivate();

  /**
   * @brief Find a dock instance & plugin in the databases from ID
   * @param dock_id Id of dock to find
   * @return Dock pointer
   */
  Dock * findDock(const std::string & dock_id);

  /**
   * @brief Find a dock plugin to use for a given type
   * @param type Dock type to find plugin for
   * @return Dock plugin pointer
   */
  ChargingDock::Ptr findDockPlugin(const std::string & type);

  /**
   * @brief Get the number of docks in the database
   * @return unsigned int Number of dock instances in the database
   */
  unsigned int instance_size() const;

  /**
   * @brief Get the number of dock types in the database
   * @return unsigned int Number of dock types in the database
   */
  unsigned int plugin_size() const;

protected:
  /**
   * @brief Populate database of dock type plugins
   */
  bool getDockPlugins(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

  /**
   * @brief Populate database of dock instances
   */
  bool getDockInstances(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

  /**
   * @brief Find a dock instance in the database from ID
   * @param dock_id Id of dock to find
   * @return Dock pointer
   */
  Dock * findDockInstance(const std::string & dock_id);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  DockPluginMap dock_plugins_;
  DockMap dock_instances_;
  pluginlib::ClassLoader<opennav_docking_core::ChargingDock> dock_loader_;
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__DOCK_DATABASE_HPP_
