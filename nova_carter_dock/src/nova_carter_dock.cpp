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
#include "nova_carter_dock/nova_carter_dock.hpp"

namespace nova_carter_dock
{

void NovaCarterChargingDock::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf)
{
  name_ = name;
  tf2_buffer_ = tf;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->declare_parameter("charging_threshold", 0.1);
  node->declare_parameter("staging_x_offset", -0.5);
  node->get_parameter("charging_threshold", charging_threshold_);
  node->get_parameter("staging_x_offset", staging_x_offset_);

  battery_sub_ = node->create_subscription<sensor_msgs::msg::BatteryState>(
    "battery_state", 1,
    std::bind(&NovaCarterChargingDock::batteryCallback, this, std::placeholders::_1));
}

void NovaCarterChargingDock::cleanup()
{
  battery_sub_.reset();
}

void NovaCarterChargingDock::activate()
{
}

void NovaCarterChargingDock::deactivate()
{
}

geometry_msgs::msg::PoseStamped NovaCarterChargingDock::getStagingPose(
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

bool NovaCarterChargingDock::getRefinedPose(geometry_msgs::msg::PoseStamped & pose)
{
  // TODO use april tags detection / frame rotate / filter averager
  pose = dock_pose_;
  return true;
}

bool NovaCarterChargingDock::isDocked()
{
  // TODO detect off of motor current, dock handshake, battery spike, etc
  return true;
}

bool NovaCarterChargingDock::isCharging()
{
  return is_charging_;
}

bool NovaCarterChargingDock::disableCharging()
{
  // Handled automatically by robot leaving the dock
  return true;
}

bool NovaCarterChargingDock::hasStoppedCharging()
{
  return !isCharging();
}

void NovaCarterChargingDock::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr state)
{
  is_charging_ = state->current > charging_threshold_;
}

}  // namespace nova_carter_dock

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nova_carter_dock::NovaCarterChargingDock, opennav_docking_core::ChargingDock)
