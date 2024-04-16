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

#include <chrono>
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "opennav_dynamic_following/following_server.hpp"
#include "nav2_util/node_thread.hpp"

// Testing unit functions in following server, smoke/system tests in python file

using namespace std::chrono_literals;  // NOLINT

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

namespace opennav_dynamic_following
{

class FollowingServerShim : public FollowingServer
{
public:
  FollowingServerShim()
  : FollowingServer() {}

  // Bypass TF
  virtual geometry_msgs::msg::PoseStamped getRobotPoseInFrame(const std::string &)
  {
    return geometry_msgs::msg::PoseStamped();
  }
};

TEST(FollowingServerTests, ObjectLifecycle)
{
  auto node = std::make_shared<opennav_dynamic_following::FollowingServer>();
  node->configure();
  node->activate();
  node->deactivate();
  node->cleanup();
  node->shutdown();
  node.reset();
}

TEST(FollowingServerTests, testDynamicParams)
{
  auto node = std::make_shared<opennav_dynamic_following::FollowingServer>();
  node->on_configure(rclcpp_lifecycle::State());
  node->on_activate(rclcpp_lifecycle::State());

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("controller_frequency", 0.2),
      rclcpp::Parameter("initial_perception_timeout", 1.0),
      rclcpp::Parameter("object_approach_timeout", 2.0),
      rclcpp::Parameter("transform_tolerance", 3.0),
      rclcpp::Parameter("base_frame", std::string("hi")),
      rclcpp::Parameter("fixed_frame", std::string("hi"))});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(node->get_parameter("controller_frequency").as_double(), 0.2);
  EXPECT_EQ(node->get_parameter("initial_perception_timeout").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("object_approach_timeout").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("transform_tolerance").as_double(), 3.0);
  EXPECT_EQ(node->get_parameter("base_frame").as_string(), std::string("hi"));
  EXPECT_EQ(node->get_parameter("fixed_frame").as_string(), std::string("hi"));
}

TEST(FollowingServerTests, RefinedPoseTest)
{
  auto node = std::make_shared<opennav_dynamic_following::FollowingServer>();
  node->on_configure(rclcpp_lifecycle::State());
  node->on_activate(rclcpp_lifecycle::State());
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "detected_dynamic_pose", rclcpp::QoS(1));

  geometry_msgs::msg::PoseStamped pose;

  // Timestamps are outdated; this is after timeout
  EXPECT_FALSE(node->getRefinedPose(pose));

  geometry_msgs::msg::PoseStamped detected_pose;
  detected_pose.header.stamp = node->now();
  detected_pose.header.frame_id = "my_frame";
  detected_pose.pose.position.x = 0.1;
  detected_pose.pose.position.y = -0.1;
  pub->publish(detected_pose);
  rclcpp::spin_some(node->get_node_base_interface());

  pose.header.frame_id = "my_frame";
  EXPECT_TRUE(node->getRefinedPose(pose));
  EXPECT_NEAR(pose.pose.position.x, 0.1, 0.01);
  EXPECT_NEAR(pose.pose.position.y, -0.1, 0.01);
  node->deactivate();
  node->cleanup();
  node.reset();
}

}  // namespace opennav_dynamic_following
