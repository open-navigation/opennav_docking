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

#include <memory>
#include <string>

#include "angles/angles.h"
#include "opennav_docking/graceful_controller.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace opennav_docking
{

void GracefulController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name, std::shared_ptr<tf2_ros::Buffer>/* tf */)
{
  parent_ = parent;
  name_ = name;
}

void GracefulController::cleanup()
{
}

void GracefulController::activate()
{
  // TODO(fergs): make these parameters
  k_phi_ = 2.0;
  k_delta_ = 1.0;
  beta_ = 0.4;
  lambda_ = 2.0;
  v_linear_min_ = 0.1;
  v_linear_max_ = 0.25;
  v_angular_max_ = 0.75;
  slowdown_radius_ = 0.25;
}

void GracefulController::deactivate()
{
}

bool GracefulController::computeVelocityCommand(
  const geometry_msgs::msg::Pose & target,
  geometry_msgs::msg::Twist & command)
{
  // If the target is behind the robot, we are going backwards
  bool backward = target.position.x < 0;

  // Convert the target to polar coordinates
  double r, phi, delta;
  {
    // Compute the difference between the target and the current pose
    float x = target.position.x;
    float y = target.position.y;
    // Compute the line of sight from the robot to the target
    // Flip it if the robot is moving backwards
    float line_of_sight = backward ? (std::atan2(-y, x) + M_PI) : std::atan2(-y, x);
    // Compute the ego polar coordinates
    r = sqrt(x * x + y * y);
    phi = angles::normalize_angle(tf2::getYaw(target.orientation) + line_of_sight);
    delta = angles::normalize_angle(line_of_sight);
  }

  // Calculate the curvature
  double curvature = calculateCurvature(r, phi, delta);

  // Adjust the linear velocity as a function of the path curvature to
  // slowdown the controller as it approaches its target
  double v = v_linear_max_ / (1.0 + beta_ * std::pow(fabs(curvature), lambda_));

  // Slowdown when the robot is near the target to remove singularity
  v = std::min(v_linear_max_ * (r / slowdown_radius_), v);

  // Set some small v_min when far away from origin to promote faster
  // turning motion when the curvature is very high
  v = std::clamp(v, v_linear_min_, v_linear_max_);

  // Compute the angular velocity
  double w = curvature * v;
  // Bound angular velocity between [-max_angular_vel, max_angular_vel]
  double w_bound = std::clamp(w, -v_angular_max_, v_angular_max_);
  // And linear velocity to follow the curvature
  v = (curvature != 0.0) ? (w_bound / curvature) : v;

  if (backward) {v = -v;}

  // Return the velocity command
  command.linear.x = v;
  command.angular.z = w_bound;
  return true;
}

double GracefulController::calculateCurvature(double r, double phi, double delta)
{
  // Calculate the proportional term of the control law as the product of the gain and the error:
  // difference between the actual steering angle and the virtual control for the slow subsystem
  double prop_term = k_delta_ * (delta - std::atan(-k_phi_ * phi));
  // Calculate the feedback control law for the steering
  double feedback_term = (1.0 + (k_phi_ / (1.0 + std::pow(k_phi_ * phi, 2)))) * sin(delta);
  // Calculate the path curvature
  return -1.0 / r * (prop_term + feedback_term);
}

}  // namespace opennav_docking
