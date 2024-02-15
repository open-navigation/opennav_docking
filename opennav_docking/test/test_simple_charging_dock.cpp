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
#include "sensor_msgs/msg/battery_state.hpp"
#include "opennav_docking/simple_charging_dock.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

// Testing the simple charging dock plugin

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

namespace opennav_docking
{

TEST(SimpleChargingDockTests, ObjectLifecycle)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));

  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();
  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  // Check initial states
  EXPECT_FALSE(dock->isCharging());
  EXPECT_TRUE(dock->disableCharging());
  EXPECT_TRUE(dock->hasStoppedCharging());

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, BatteryState)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto pub = node->create_publisher<sensor_msgs::msg::BatteryState>(
    "battery_state", rclcpp::QoS(1));
  pub->on_activate();
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));

  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();

  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  // Below threshold
  sensor_msgs::msg::BatteryState msg;
  msg.current = 0.3;
  pub->publish(msg);
  rclcpp::Rate r(2);
  r.sleep();
  rclcpp::spin_some(node->get_node_base_interface());

  EXPECT_FALSE(dock->isCharging());
  EXPECT_TRUE(dock->hasStoppedCharging());

  // Above threshold
  sensor_msgs::msg::BatteryState msg2;
  msg2.current = 0.6;
  pub->publish(msg2);
  rclcpp::Rate r1(2);
  r1.sleep();
  rclcpp::spin_some(node->get_node_base_interface());

  EXPECT_TRUE(dock->isCharging());
  EXPECT_FALSE(dock->hasStoppedCharging());

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, StagingPose)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();

  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  geometry_msgs::msg::Pose pose;
  std::string frame = "my_frame";
  auto staging_pose = dock->getStagingPose(pose, frame);
  EXPECT_NEAR(staging_pose.pose.position.x, -0.5, 0.01);
  EXPECT_NEAR(staging_pose.pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(tf2::getYaw(staging_pose.pose.orientation), 0.0, 0.01);
  EXPECT_EQ(staging_pose.header.frame_id, frame);

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, StagingPoseWithYawOffset)
{
  // Override the parameter default
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      {"my_dock.staging_yaw_offset", 3.14},
    }
  );

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test", options);
  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();

  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  geometry_msgs::msg::Pose pose;
  std::string frame = "my_frame";
  auto staging_pose = dock->getStagingPose(pose, frame);
  // Pose should be the same as default, but pointing in opposite direction
  EXPECT_NEAR(staging_pose.pose.position.x, -0.5, 0.01);
  EXPECT_NEAR(staging_pose.pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(tf2::getYaw(staging_pose.pose.orientation), 3.14, 0.01);
  EXPECT_EQ(staging_pose.header.frame_id, frame);

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, RefinedPoseTest)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "detected_dock_pose", rclcpp::QoS(1));
  pub->on_activate();
  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();

  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  geometry_msgs::msg::PoseStamped pose;

  // Timestamps are outdated; this is after timeout
  EXPECT_FALSE(dock->isDocked());
  EXPECT_FALSE(dock->getRefinedPose(pose));

  geometry_msgs::msg::PoseStamped detected_pose;
  detected_pose.header.stamp = node->now();
  detected_pose.header.frame_id = "my_frame";
  detected_pose.pose.position.x = 0.1;
  detected_pose.pose.position.y = -0.1;
  pub->publish(detected_pose);
  rclcpp::spin_some(node->get_node_base_interface());

  pose.header.frame_id = "my_frame";
  EXPECT_TRUE(dock->getRefinedPose(pose));
  EXPECT_NEAR(pose.pose.position.x, 0.1, 0.01);
  EXPECT_NEAR(pose.pose.position.y, -0.1, 0.01);

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

}  // namespace opennav_docking
