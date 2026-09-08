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

#ifndef OPENNAV_DOCKING__HOLONOMIC_CONTROL_LAW_HPP_
#define OPENNAV_DOCKING__HOLONOMIC_CONTROL_LAW_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace opennav_docking
{

/**
 * @class opennav_docking::HolonomicControlLaw
 * @brief Proportional control law for a holonomic (omnidirectional) base.
 *        Counterpart to nav2_graceful_controller::SmoothControlLaw for
 *        differential-drive docking.
 */
class HolonomicControlLaw
{
public:
  /**
   * @brief Construct a holonomic control law.
   * @param k_phi Proportional gain on the linear position error.
   * @param k_delta Proportional gain on the heading error.
   * @param v_linear_max Maximum linear velocity [m/s].
   * @param v_angular_max Maximum angular velocity [rad/s].
   */
  HolonomicControlLaw(
    double k_phi, double k_delta, double v_linear_max, double v_angular_max);

  /**
   * @brief Compute a velocity command toward the target.
   * @param target Target pose in robot-centric coordinates.
   * @return Velocity command (linear.x/y, angular.z).
   */
  geometry_msgs::msg::Twist calculateRegularVelocity(const geometry_msgs::msg::Pose & target);

  /**
   * @brief Simulate one time step (Euler) for collision trajectory checking.
   * @param dt Time step [s].
   * @param target Target pose in the robot-initial frame.
   * @param current Current robot pose in the robot-initial frame.
   * @return Next robot pose after dt.
   */
  geometry_msgs::msg::Pose calculateNextPose(
    double dt,
    const geometry_msgs::msg::Pose & target,
    const geometry_msgs::msg::Pose & current);

  /** @brief Update the proportional gains. */
  void setGains(double k_phi, double k_delta);

  /** @brief Update the speed limits. */
  void setSpeedLimit(double v_linear_max, double v_angular_max);

protected:
  double k_phi_;
  double k_delta_;
  double v_linear_max_;
  double v_angular_max_;
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__HOLONOMIC_CONTROL_LAW_HPP_
