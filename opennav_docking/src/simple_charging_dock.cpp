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

#include "nav2_util/node_utils.hpp"
#include "opennav_docking/simple_charging_dock.hpp"

namespace opennav_docking
{

void SimpleChargingDock::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf)
{
  name_ = name;
  tf2_buffer_ = tf;

  is_charging_ = false;

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".charging_threshold", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".docking_threshold", rclcpp::ParameterValue(0.02));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".staging_x_offset", rclcpp::ParameterValue(-0.5));

  node->get_parameter(name + ".charging_threshold", charging_threshold_);
  node->get_parameter(name + ".docking_threshold", docking_threshold_);
  node->get_parameter(name + ".staging_x_offset", staging_x_offset_);

  battery_sub_ = node->create_subscription<sensor_msgs::msg::BatteryState>(
    "battery_state", 1,
    std::bind(&SimpleChargingDock::batteryCallback, this, std::placeholders::_1));
}

void SimpleChargingDock::cleanup()
{
}

void SimpleChargingDock::activate()
{
}

void SimpleChargingDock::deactivate()
{
}

geometry_msgs::msg::PoseStamped SimpleChargingDock::getStagingPose(
  const geometry_msgs::msg::Pose & pose, const std::string & frame)
{
  // This gets called at the start of docking
  // Reset our internally tracked dock pose
  dock_pose_.header.frame_id = frame;
  dock_pose_.pose = pose;

  // Compute the staging pose - robot pointed at dock, but backed up a bit
  const double yaw = tf2::getYaw(dock_pose_.pose.orientation);
  geometry_msgs::msg::PoseStamped staging_pose;
  staging_pose = dock_pose_;
  staging_pose.pose.position.x += cos(yaw) * staging_x_offset_;
  staging_pose.pose.position.y += sin(yaw) * staging_x_offset_;

  return staging_pose;
}

bool SimpleChargingDock::getRefinedPose(geometry_msgs::msg::PoseStamped & pose)
{
  // Just returned cached pose
  pose = dock_pose_;
  return true;
}

bool SimpleChargingDock::isDocked()
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
  return d < docking_threshold_;
}

bool SimpleChargingDock::isCharging()
{
  return is_charging_;
}

bool SimpleChargingDock::disableCharging()
{
  return true;
}

bool SimpleChargingDock::hasStoppedCharging()
{
  return !isCharging();
}

void SimpleChargingDock::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr state)
{
  is_charging_ = state->current > charging_threshold_;
}

}  // namespace opennav_docking

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(opennav_docking::SimpleChargingDock, opennav_docking_core::ChargingDock)
