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

#ifndef OPENNAV_DOCKING_CORE__CHARGING_DOCK_HPP_
#define OPENNAV_DOCKING_CORE__CHARGING_DOCK_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace opennav_docking_core
{

/**
 * @class ChargingDock
 * @brief Abstract interface for a charging dock for the docking framework
 */
class ChargingDock
{
public:
  using Ptr = std::shared_ptr<ChargingDock>;

  /**
   * @brief Virtual destructor
   */
  virtual ~ChargingDock() {}

  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active Behavior and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive Behavior and any threads involved in execution.
   */
  virtual void deactivate() = 0;
};

}  // namespace opennav_docking_core

#endif  // OPENNAV_DOCKING_CORE__CHARGING_DOCK_HPP_
