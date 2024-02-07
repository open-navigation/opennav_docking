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

#ifndef OPENNAV_DOCKING__SIMPLE_CHARGING_DOCK_HPP_
#define OPENNAV_DOCKING__SIMPLE_CHARGING_DOCK_HPP_

#include <string>
#include <memory>

#include "sensor_msgs/msg/battery_state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#include "opennav_docking_core/charging_dock.hpp"

namespace opennav_docking
{

class SimpleChargingDock : public opennav_docking_core::ChargingDock
{
  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf);

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup();

  /**
   * @brief Method to active Behavior and any threads involved in execution.
   */
  virtual void activate();

  /**
   * @brief Method to deactive Behavior and any threads involved in execution.
   */
  virtual void deactivate();

  /**
   * @brief Method to obtain the dock's staging pose. This method should likely
   * be using TF and the dock's pose information to find the staging pose from
   * a static or parameterized staging pose relative to the docking pose
   * @param pose Dock with pose
   * @param frame Dock's frame of pose
   * @return PoseStamped of staging pose in the specified frame
   */
  virtual geometry_msgs::msg::PoseStamped getStagingPose(
    const geometry_msgs::msg::Pose & pose, const std::string & frame);

  /**
   * @brief Method to obtain the refined pose of the dock, usually based on sensors
   * @param pose The initial estimate of the dock pose.
   * @param frame The frame of the initial estimate.
   */
  virtual bool getRefinedPose(geometry_msgs::msg::PoseStamped & pose);

  /**
   * @copydoc opennav_docking_core::ChargingDock::isDocked
   */
  virtual bool isDocked();

  /**
   * @copydoc opennav_docking_core::ChargingDock::isCharging
   */
  virtual bool isCharging();

  /**
   * @copydoc opennav_docking_core::ChargingDock::disableCharging
   */
  virtual bool disableCharging();

  /**
   * @brief Similar to isCharging() but called when undocking.
   */
  virtual bool hasStoppedCharging();

private:
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr state);

  // For testing, have the dock pose be hard coded (maybe add a service to set it? TODO(fergs))
  geometry_msgs::msg::PoseStamped dock_pose_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  bool is_charging_;

  double charging_threshold_;
  double docking_threshold_;
  double staging_x_offset_;

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__SIMPLE_CHARGING_DOCK_HPP_
