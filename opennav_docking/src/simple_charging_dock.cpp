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

  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".use_external_detection_pose", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_timeout", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_translation_x", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_translation_y", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_yaw", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_pitch", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_roll", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".charging_threshold", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".docking_threshold", rclcpp::ParameterValue(0.02));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".staging_x_offset", rclcpp::ParameterValue(-0.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".staging_yaw_offset", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, "fixed_frame", rclcpp::ParameterValue("odom"));

  node_->get_parameter(name + ".use_external_detection_pose", use_external_detection_pose_);
  node_->get_parameter(name + ".external_detection_timeout", external_detection_timeout_);
  node_->get_parameter(
    name + ".external_detection_translation_x", external_detection_translation_x_);
  node_->get_parameter(
    name + ".external_detection_translation_y", external_detection_translation_y_);
  double yaw, pitch, roll;
  node_->get_parameter(name + ".external_detection_rotation_yaw", yaw);
  node_->get_parameter(name + ".external_detection_rotation_pitch", pitch);
  node_->get_parameter(name + ".external_detection_rotation_roll", roll);
  external_detection_rotation_.setEuler(yaw, pitch, roll);
  node_->get_parameter(name + ".charging_threshold", charging_threshold_);
  node_->get_parameter(name + ".docking_threshold", docking_threshold_);
  node_->get_parameter(name + ".staging_x_offset", staging_x_offset_);
  node_->get_parameter(name + ".staging_yaw_offset", staging_yaw_offset_);
  node_->get_parameter("fixed_frame", fixed_frame_);

  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "battery_state", 1,
    std::bind(&SimpleChargingDock::batteryCallback, this, std::placeholders::_1));

  if (use_external_detection_pose_) {
    dock_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "detected_dock_pose", 1,
      std::bind(&SimpleChargingDock::dockPoseCallback, this, std::placeholders::_1));

    dock_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("dock_pose", 1);
  }

  staging_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("staging_pose", 1);
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
  if (!use_external_detection_pose_) {
    // This gets called at the start of docking
    // Reset our internally tracked dock pose
    dock_pose_.header.frame_id = frame;
    dock_pose_.pose = pose;
  }

  // Compute the staging pose
  const double yaw = tf2::getYaw(pose.orientation);
  geometry_msgs::msg::PoseStamped staging_pose;
  staging_pose.header.frame_id = frame;
  staging_pose.header.stamp = node_->now();
  staging_pose.pose = pose;
  staging_pose.pose.position.x += cos(yaw) * staging_x_offset_;
  staging_pose.pose.position.y += sin(yaw) * staging_x_offset_;
  // TODO(fergs): add staging_yaw_offset_ to orientation;

  // Publish staging pose for debugging purposes
  staging_pose_pub_->publish(staging_pose);

  return staging_pose;
}

bool SimpleChargingDock::getRefinedPose(geometry_msgs::msg::PoseStamped & pose)
{
  // Validate that external pose is new enough
  if (use_external_detection_pose_) {
    auto timeout = rclcpp::Duration::from_seconds(external_detection_timeout_);
    if (node_->now() - detected_dock_pose_.header.stamp > timeout) {
      return false;
    }

    if (dock_pose_.header.stamp != detected_dock_pose_.header.stamp) {
      dock_pose_ = detected_dock_pose_;
      // New detected_pose to process
      if (dock_pose_.header.frame_id != pose.header.frame_id) {
        try {
          dock_pose_.header.stamp = rclcpp::Time(0);
          tf2_buffer_->transform(dock_pose_, dock_pose_, pose.header.frame_id);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN(node_->get_logger(), "Failed to transform detected dock pose");
          return false;
        }
      }

      // Rotate the just the orientation
      geometry_msgs::msg::PoseStamped just_orientation;
      just_orientation.pose.orientation = tf2::toMsg(external_detection_rotation_);
      geometry_msgs::msg::TransformStamped transform;
      transform.transform.rotation = dock_pose_.pose.orientation;
      tf2::doTransform(just_orientation, just_orientation, transform);

      // Construct dock pose
      dock_pose_.header.stamp = detected_dock_pose_.header.stamp;
      dock_pose_.pose.orientation = just_orientation.pose.orientation;
      const double yaw = tf2::getYaw(dock_pose_.pose.orientation);
      dock_pose_.pose.position.x += cos(yaw) * external_detection_translation_x_ -
        sin(yaw) * external_detection_translation_y_;
      dock_pose_.pose.position.y += sin(yaw) * external_detection_translation_x_ +
        cos(yaw) * external_detection_translation_y_;
      dock_pose_.pose.position.z = 0.0;
    }
  }

  // Publish dock pose for debugging purposes
  dock_pose_pub_->publish(dock_pose_);

  // Return dock pose
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

void SimpleChargingDock::dockPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  detected_dock_pose_ = *pose;
}

}  // namespace opennav_docking

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(opennav_docking::SimpleChargingDock, opennav_docking_core::ChargingDock)
