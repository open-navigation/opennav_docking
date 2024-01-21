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

#ifndef OPENNAV_DOCKING__GRACEFUL_CONTROLLER_HPP_
#define OPENNAV_DOCKING__GRACEFUL_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "opennav_docking_core/controller.hpp"

namespace opennav_docking
{
/**
 * @class opennav_docking::GracefulController
 * @brief Default control law for approaching a dock target
 */
class GracefulController : public opennav_docking_core::Controller
{
public:
  /**
   * @param parent pointer to user's node
   * @param name The name of this planner
   * @param tf A pointer to a TF buffer
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
   * @brief Compute control velocities.
   * @param target Target pose, in robot-centric frame.
   * @param command Computed command to execute for this update cycle.
   */
  virtual bool computeVelocityCommand(
    const geometry_msgs::msg::Pose & target,
    geometry_msgs::msg::Twist & command);

private:
  double calculateCurvature(double r, double phi, double delta);

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::string name_;

  double k_phi_;
  double k_delta_;
  double beta_;
  double lambda_;
  double v_linear_min_, v_linear_max_;
  double v_angular_max_;
  double slowdown_radius_;
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__GRACEFUL_CONTROLLER_HPP_
