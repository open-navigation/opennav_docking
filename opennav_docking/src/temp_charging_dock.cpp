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

#include <cmath>
#include <string>
#include <memory>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#include "opennav_docking_core/charging_dock.hpp"

namespace opennav_docking
{

class TempChargingDock : public opennav_docking_core::ChargingDock
{
  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & /*parent*/,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf)
  {
    name_ = name;
    tf2_buffer_ = tf;
  }

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup()
  {
  }

  /**
   * @brief Method to active Behavior and any threads involved in execution.
   */
  virtual void activate()
  {
  }

  /**
   * @brief Method to deactive Behavior and any threads involved in execution.
   */
  virtual void deactivate()
  {
  }

  /**
   * @brief Method to obtain the dock's staging pose. This method should likely
   * be using TF and the dock's pose information to find the staging pose from
   * a static or parameterized staging pose relative to the docking pose
   * @param pose Dock with pose
   * @param frame Dock's frame of pose
   * @return PoseStamped of staging pose in the specified frame
   */
  virtual geometry_msgs::msg::PoseStamped getStagingPose(
    const geometry_msgs::msg::Pose & pose, const std::string & frame)
  {
    // This gets called at the start of docking
    // Reset our internally tracked dock pose
    dock_pose_.header.frame_id = frame;
    dock_pose_.pose = pose;

    // Compute the staging pose - robot pointed at dock, but backed up a bit (0.5m)
    const double offset = -0.5;
    const double yaw = tf2::getYaw(dock_pose_.pose.orientation);
    geometry_msgs::msg::PoseStamped staging_pose;
    staging_pose = dock_pose_;
    staging_pose.pose.position.x += cos(yaw) * offset;
    staging_pose.pose.position.y += sin(yaw) * offset;

    return staging_pose;
  }

  /**
   * @brief Method to obtain the refined pose of the dock, usually based on sensors
   * @param pose The initial estimate of the dock pose.
   * @param frame The frame of the initial estimate.
   */
  virtual bool getRefinedPose(geometry_msgs::msg::PoseStamped & pose)
  {
    // Just returned cached pose
    pose = dock_pose_;
    return true;
  }

  /**
   * @brief Are we charging? If a charge dock requires any sort of negotiation
   * to begin charging, that should happen inside this function as this function
   * will be called repeatedly within the docking loop.
   *
   * NOTE: this function is expected to return QUICKLY. Blocking here will block
   * the docking controller loop.
   */
  virtual bool isCharging()
  {
    if (dock_pose_.header.frame_id.empty()) {
      // Dock pose is not yet valid
      return false;
    }

    // Find base pose in target frame
    geometry_msgs::msg::PoseStamped base_pose;
    base_pose.header.stamp = rclcpp::Time(0);
    base_pose.header.frame_id = "base_link";
    base_pose.pose.orientation.w = 1.0;
    try {
      tf2_buffer_->transform(base_pose, base_pose, dock_pose_.header.frame_id);
    } catch (const tf2::TransformException & ex) {
      // TODO(fergs): some sort of error message?
    }

    // If we are close enough, pretend we are charging
    double d = std::hypot(
      base_pose.pose.position.x - dock_pose_.pose.position.x,
      base_pose.pose.position.y - dock_pose_.pose.position.y);
    return d < 0.02;
  }

  /**
   * @brief Undocking while current is still flowing can damage a charge dock
   * so some charge docks provide the ability to disable charging before the
   * robot physically disconnects. The undocking action will not command the
   * robot to move until this returns true.
   */
  virtual bool disableCharging()
  {
    return true;
  }

  /**
   * @brief Similar to isCharging() but called when undocking.
   */
  virtual bool hasStoppedCharging()
  {
    // Nothing to do here - just pretend we've already disabled charging
    return true;
  }

private:
  // For testing, have the dock pose be hard coded (maybe add a service to set it?)
  geometry_msgs::msg::PoseStamped dock_pose_;

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
};

}  // namespace opennav_docking

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(opennav_docking::TempChargingDock, opennav_docking_core::ChargingDock)
