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

#ifndef OPENNAV_DOCKING__TYPES_HPP_
#define OPENNAV_DOCKING__TYPES_HPP_

#include "opennav_docking_msgs/action/dock_robot.hpp"
#include "opennav_docking_msgs/action/undock_robot.hpp"

typedef opennav_docking_msgs::action::DockRobot DockRobot;
typedef opennav_docking_msgs::action::UndockRobot UndockRobot;

class DockingException : public std::runtime_error
{
public:
  explicit DockingException(const std::string & description)
  : std::runtime_error(description) {}
};

#endif  // OPENNAV_DOCKING__TYPES_HPP_
