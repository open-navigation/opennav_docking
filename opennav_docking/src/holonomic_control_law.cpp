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

#include <algorithm>
#include <cmath>

#include "opennav_docking/holonomic_control_law.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace opennav_docking
{

HolonomicControlLaw::HolonomicControlLaw(
  double k_phi, double k_delta, double v_linear_max, double v_angular_max)
: k_phi_(k_phi), k_delta_(k_delta), v_linear_max_(v_linear_max), v_angular_max_(v_angular_max)
{
}

geometry_msgs::msg::Twist HolonomicControlLaw::calculateRegularVelocity(
  const geometry_msgs::msg::Pose & target)
{
  geometry_msgs::msg::Twist cmd;

  const double yaw_error = tf2::getYaw(target.orientation);
  cmd.angular.z = std::clamp(k_delta_ * yaw_error, -v_angular_max_, v_angular_max_);
  cmd.linear.x = std::clamp(k_phi_ * target.position.x, -v_linear_max_, v_linear_max_);
  cmd.linear.y = std::clamp(k_phi_ * target.position.y, -v_linear_max_, v_linear_max_);

  return cmd;
}

geometry_msgs::msg::Pose HolonomicControlLaw::calculateNextPose(
  double dt,
  const geometry_msgs::msg::Pose & target,
  const geometry_msgs::msg::Pose & current)
{
  // Express target position in current robot body frame
  const double dx = target.position.x - current.position.x;
  const double dy = target.position.y - current.position.y;
  const double current_yaw = tf2::getYaw(current.orientation);
  const double target_yaw = tf2::getYaw(target.orientation);

  geometry_msgs::msg::Pose relative_target;
  relative_target.position.x =  dx * std::cos(current_yaw) + dy * std::sin(current_yaw);
  relative_target.position.y = -dx * std::sin(current_yaw) + dy * std::cos(current_yaw);

  // Normalize heading error to [-pi, pi]
  double heading_error = target_yaw - current_yaw;
  while (heading_error >  M_PI) { heading_error -= 2.0 * M_PI; }
  while (heading_error < -M_PI) { heading_error += 2.0 * M_PI; }
  relative_target.orientation.z = std::sin(heading_error / 2.0);
  relative_target.orientation.w = std::cos(heading_error / 2.0);

  // Compute holonomic velocity in body frame
  const geometry_msgs::msg::Twist cmd = calculateRegularVelocity(relative_target);

  // Euler-integrate back into the robot-initial frame
  geometry_msgs::msg::Pose next = current;
  next.position.x +=
    (cmd.linear.x * std::cos(current_yaw) - cmd.linear.y * std::sin(current_yaw)) * dt;
  next.position.y +=
    (cmd.linear.x * std::sin(current_yaw) + cmd.linear.y * std::cos(current_yaw)) * dt;

  const double new_yaw = current_yaw + cmd.angular.z * dt;
  next.orientation.x = 0.0;
  next.orientation.y = 0.0;
  next.orientation.z = std::sin(new_yaw / 2.0);
  next.orientation.w = std::cos(new_yaw / 2.0);

  return next;
}

void HolonomicControlLaw::setGains(double k_phi, double k_delta)
{
  k_phi_ = k_phi;
  k_delta_ = k_delta;
}

void HolonomicControlLaw::setSpeedLimit(double v_linear_max, double v_angular_max)
{
  v_linear_max_ = v_linear_max;
  v_angular_max_ = v_angular_max;
}

}  // namespace opennav_docking
