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

#include <string>
#include <memory>

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
    tf_buffer_ = tf;
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
    geometry_msgs::msg::PoseStamped p;
    p.pose = pose;
    p.header.frame_id = frame;
    return p;
  }

  /**
   * @brief Method to obtain the refined pose of the dock, usually based on sensors
   * @param pose The initial estimate of the dock pose.
   * @param frame The frame of the initial estimate.
   */
  virtual bool getRefinedPose(geometry_msgs::msg::PoseStamped & /*pose*/)
  {
    return false;
  }

  /**
   * @brief Method to obtain the target pose for the robot from a refined dock pose.
   */
  virtual geometry_msgs::msg::PoseStamped getTargetPose(
    const geometry_msgs::msg::PoseStamped & dock_pose)
  {
    return dock_pose;
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
    return false;
  }

  /**
   * @brief Similar to isCharging() but called when undocking.
   */
  virtual bool hasStoppedCharging()
  {
    return false;
  }
private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace opennav_docking

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(opennav_docking::TempChargingDock, opennav_docking_core::ChargingDock)
