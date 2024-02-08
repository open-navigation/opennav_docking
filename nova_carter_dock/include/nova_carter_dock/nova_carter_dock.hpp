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

#ifndef NOVA_CARTER_DOCK__NOVA_CARTER_DOCK_HPP_
#define NOVA_CARTER_DOCK__NOVA_CARTER_DOCK_HPP_

#include <cmath>
#include <string>
#include <memory>

#include "sensor_msgs/msg/battery_state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "opennav_docking_core/charging_dock.hpp"

namespace nova_carter_dock
{

class NovaCarterChargingDock : public opennav_docking_core::ChargingDock
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
   * @brief Method to obtain the dock's staging pose.
   * @param pose Dock with pose
   * @param frame Dock's frame of pose
   * @return PoseStamped of staging pose in the specified frame
   */
  virtual geometry_msgs::msg::PoseStamped getStagingPose(
    const geometry_msgs::msg::Pose & pose, const std::string & frame);

  /**
   * @brief Method to obtain the refined pose of the dock based on sensors
   * @param pose The initial estimate of the dock pose.
   * @param frame The frame of the initial estimate.
   */
  virtual bool getRefinedPose(geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Have we made contact with dock? This can be implemented in a variety
   * of ways: by establishing communications with the dock, by monitoring the
   * the drive motor efforts, etc.
   */
  virtual bool isDocked();

  /**
   * @brief Are we charging? If a charge dock requires any sort of negotiation
   * to begin charging, that should happen inside this function as this function
   * will be called repeatedly within the docking loop.
   *
   * NOTE: this function is expected to return QUICKLY. Blocking here will block
   * the docking controller loop.
   */

  virtual bool isCharging();
  /**
   * @brief Undocking while current is still flowing can damage a charge dock
   * so some charge docks provide the ability to disable charging before the
   * robot physically disconnects. The undocking action will not command the
   * robot to move until this returns true.
   */
  virtual bool disableCharging();

  /**
   * @brief Similar to isCharging() but called when undocking.
   */
  virtual bool hasStoppedCharging();

private:
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr state);

  // For testing, have the dock pose be hard coded (maybe add a service to set it?)
  geometry_msgs::msg::PoseStamped dock_pose_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  bool is_charging_{false};

  double charging_threshold_;
  double staging_x_offset_;

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
};
}  // namespace nova_carter_dock

#endif  // NOVA_CARTER_DOCK__NOVA_CARTER_DOCK_HPP_
