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

#ifndef OPENNAV_DOCKING__UTILS_HPP_
#define OPENNAV_DOCKING__UTILS_HPP_

#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "opennav_docking/types.hpp"

namespace utils
{

using rclcpp::ParameterType::PARAMETER_STRING;
using rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
using rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
using nav2_util::geometry_utils::orientationAroundZAxis;

/**
* @brief Parse a yaml file to obtain docks
* @param yaml_file A yaml file path with docks
* @param node Node to use for logging
* @param dock_db Database to populate
*/
inline bool parseDockFile(
  const std::string & yaml_filepath,
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  DockMap & dock_db)
{
  YAML::Node yaml_file = YAML::LoadFile(yaml_filepath);
  if (!yaml_file["docks"]) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Dock database (%s) does not contain 'docks'.", yaml_filepath.c_str());
    return false;
  }

  auto yaml_docks = yaml_file["docks"];
  Dock curr_dock;
  for (const auto & yaml_dock : yaml_docks) {
    std::string dock_name = yaml_dock.first.as<std::string>();
    const YAML::Node & dock_attribs = yaml_dock.second;

    curr_dock.frame = "map";
    if (dock_attribs["frame"]) {
      curr_dock.frame = dock_attribs["frame"].as<std::string>();
    }

    if (!dock_attribs["type"]) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Dock database (%s) entries do not contain 'type'.", yaml_filepath.c_str());
      return false;
    }
    curr_dock.type = dock_attribs["type"].as<std::string>();

    if (!dock_attribs["pose"]) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Dock database (%s) entries do not contain 'pose'.", yaml_filepath.c_str());
      return false;
    }
    std::vector<double> pose_arr = dock_attribs["pose"].as<std::vector<double>>();
    if (pose_arr.size() != 3u) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Dock database (%s) entries do not contain pose of size 3.", yaml_filepath.c_str());
      return false;
    }
    curr_dock.pose.position.x = pose_arr[0];
    curr_dock.pose.position.y = pose_arr[1];
    curr_dock.pose.orientation = orientationAroundZAxis(pose_arr[2]);

    // Insert into dock instance database
    dock_db.emplace(dock_name, curr_dock);
  }

  return true;
}

/**
* @brief Parse a parameter file to obtain docks
* @param docks_param Parameter dock namespaces to parse
* @param node Node to use for logging & getting parameters
* @param dock_db Database to populate
*/
inline bool parseDockParams(
  const std::vector<std::string> & docks_param,
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  DockMap & dock_db)
{
  Dock curr_dock;
  std::vector<double> pose_arr;
  for (const auto & dock_name : docks_param) {
    if (!node->has_parameter(dock_name + ".frame")) {
      node->declare_parameter(dock_name + ".frame", "map");
    }
    node->get_parameter(dock_name + ".frame", curr_dock.frame);

    if (!node->has_parameter(dock_name + ".type")) {
      node->declare_parameter(dock_name + ".type", PARAMETER_STRING);
    }
    if (!node->get_parameter(dock_name + ".type", curr_dock.type)) {
      RCLCPP_ERROR(node->get_logger(), "Dock %s has no dock 'type'.", dock_name.c_str());
      return false;
    }

    pose_arr.clear();
    if (!node->has_parameter(dock_name + ".pose")) {
      node->declare_parameter(dock_name + ".pose", PARAMETER_DOUBLE_ARRAY);
    }
    if (!node->get_parameter(dock_name + ".pose", pose_arr) || pose_arr.size() != 3u) {
      RCLCPP_ERROR(node->get_logger(), "Dock %s has no valid 'pose'.", dock_name.c_str());
      return false;
    }
    curr_dock.pose.position.x = pose_arr[0];
    curr_dock.pose.position.y = pose_arr[1];
    curr_dock.pose.orientation = orientationAroundZAxis(pose_arr[2]);

    // Insert into dock instance database
    dock_db.emplace(dock_name, curr_dock);
  }
  return true;
}

}  // namespace utils

#endif  // OPENNAV_DOCKING__UTILS_HPP_
