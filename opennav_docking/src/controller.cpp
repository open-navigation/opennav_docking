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

#include "rclcpp/rclcpp.hpp"
#include "opennav_docking/controller.hpp"

namespace opennav_docking
{

Controller::Controller(nav2_util::LifecycleNode * node)
{
  node->declare_parameter("controller.k_phi", 2.0);
  node->declare_parameter("controller.k_delta", 1.0);
  node->declare_parameter("controller.beta", 0.4);
  node->declare_parameter("controller.lambda", 2.0);
  node->declare_parameter("controller.v_linear_min", 0.1);
  node->declare_parameter("controller.v_linear_max", 0.25);
  node->declare_parameter("controller.v_angular_max", 0.75);
  node->declare_parameter("controller.slowdown_radius", 0.25);
}

void Controller::configure(nav2_util::LifecycleNode * node)
{
  double k_phi, k_delta, beta, lambda;
  double slowdown_radius, v_linear_min, v_linear_max, v_angular_max;
  node->get_parameter("controller.k_phi", k_phi);
  node->get_parameter("controller.k_delta", k_delta);
  node->get_parameter("controller.beta", beta);
  node->get_parameter("controller.lambda", lambda);
  node->get_parameter("controller.v_linear_min", v_linear_min);
  node->get_parameter("controller.v_linear_max", v_linear_max);
  node->get_parameter("controller.v_angular_max", v_angular_max);
  node->get_parameter("controller.slowdown_radius", slowdown_radius);
  control_law_ = std::make_unique<nav2_graceful_controller::SmoothControlLaw>(
    k_phi, k_delta, beta, lambda, slowdown_radius, v_linear_min, v_linear_max, v_angular_max);
}

bool Controller::computeVelocityCommand(
  const geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & cmd)
{
  // If the target is behind the robot, we are backwards
  bool backward = pose.position.x < 0;
  if (control_law_) {
    cmd = control_law_->calculateRegularVelocity(pose, backward);
    return true;
  }
  return false;
}

}  // namespace opennav_docking
