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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "opennav_docking/docking_server.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

// Testing unit functions in docking server, smoke/system tests in python file

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

namespace opennav_docking
{

TEST(DockingServerTests, ObjectLifecycle)
{
  auto node = std::make_shared<opennav_docking::DockingServer>();
  node->configure();
  node->activate();
  node->deactivate();
  node->cleanup();
  node->shutdown();
  node.reset();
}

TEST(DockingServerTests, testDynamicParams)
{
  auto node = std::make_shared<opennav_docking::DockingServer>();
  node->configure();
  node->activate();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("controller_frequency", 0.2),
      rclcpp::Parameter("initial_perception_timeout", 1.0),
      rclcpp::Parameter("wait_charge_timeout", 1.2),
      rclcpp::Parameter("undock_linear_tolerance", 0.25),
      rclcpp::Parameter("undock_angular_tolerance", 0.125),
      rclcpp::Parameter("base_frame", std::string("hi")),
      rclcpp::Parameter("fixed_frame", std::string("hi")),
      rclcpp::Parameter("max_retries", 7)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(node->get_parameter("controller_frequency").as_double(), 0.2);
  EXPECT_EQ(node->get_parameter("initial_perception_timeout").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("wait_charge_timeout").as_double(), 1.2);
  EXPECT_EQ(node->get_parameter("undock_linear_tolerance").as_double(), 0.25);
  EXPECT_EQ(node->get_parameter("undock_angular_tolerance").as_double(), 0.125);
  EXPECT_EQ(node->get_parameter("base_frame").as_string(), std::string("hi"));
  EXPECT_EQ(node->get_parameter("fixed_frame").as_string(), std::string("hi"));
  EXPECT_EQ(node->get_parameter("max_retries").as_int(), 7);
}

}  // namespace opennav_docking
