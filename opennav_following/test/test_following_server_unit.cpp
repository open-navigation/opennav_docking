// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "opennav_following/following_exceptions.hpp"
#include "opennav_following/following_server.hpp"
#include "nav2_util/node_thread.hpp"
#include "tf2/utils.h"

// Testing unit functions in following server, smoke/system tests in python file

using namespace std::chrono_literals;  // NOLINT

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

namespace opennav_following
{

using FollowObject = opennav_following_msgs::action::FollowObject;

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

  virtual void doInitialPerception(geometry_msgs::msg::PoseStamped &)
  {
    std::string exception;
    this->get_parameter("exception_to_throw", exception);
    if (exception == "TransformException") {
      throw tf2::TransformException("TransformException");
    } else if (exception == "ObjectNotValid") {
      throw opennav_following::ObjectNotValid("ObjectNotValid");
    } else if (exception == "FailedToDetectObject") {
      throw opennav_following::FailedToDetectObject("FailedToDetectObject");
    } else if (exception == "FailedToControl") {
      throw opennav_following::FailedToControl("FailedToControl");
    } else if (exception == "FollowingException") {
      throw opennav_following::FollowingException("FollowingException");
    } else if (exception == "exception") {
      throw std::exception();
    }
  }

  virtual bool getRefinedPose(geometry_msgs::msg::PoseStamped & pose)
  {
    return FollowingServer::getRefinedPose(pose);
  }

  geometry_msgs::msg::PoseStamped getPoseAtDistance(
    const geometry_msgs::msg::PoseStamped & pose, double distance)
  {
    return FollowingServer::getPoseAtDistance(pose, distance);
  }

  bool isGoalReached(const geometry_msgs::msg::PoseStamped & goal_pose)
  {
    return FollowingServer::isGoalReached(goal_pose);
  }

  geometry_msgs::msg::PoseStamped getFilteredPose()
  {
    return dynamic_pose_;
  }

  void setSkipOrientation(bool skip_orientation)
  {
    skip_orientation_ = skip_orientation;
  }
};

TEST(FollowingServerTests, ObjectLifecycle)
{
  auto node = std::make_shared<opennav_following::FollowingServer>();
  node->configure();
  node->activate();
  node->deactivate();
  node->cleanup();
  node->shutdown();
  node.reset();
}

TEST(FollowingServerTests, ErrorExceptions)
{
  auto node = std::make_shared<FollowingServerShim>();
  auto node_thread = nav2_util::NodeThread(node);
  auto node2 = std::make_shared<rclcpp::Node>("client_node");

  node->on_configure(rclcpp_lifecycle::State());
  node->on_activate(rclcpp_lifecycle::State());

  node->declare_parameter("exception_to_throw", rclcpp::ParameterValue(""));
  node->declare_parameter("follow_action_called", rclcpp::ParameterValue(false));
  node->set_parameter(rclcpp::Parameter("fixed_frame", rclcpp::ParameterValue("test_frame")));

  // Error codes following
  std::vector<std::string> error_ids{
    "TransformException", "ObjectNotValid", "FailedToDetectObject", "FailedToControl",
    "FollowingException", "exception"};
  std::vector<int> error_codes{1002, 1003, 1004, 1005, 1000, 1000};

  // Call action, check error code
  for (unsigned int i = 0; i != error_ids.size(); i++) {
    node->set_parameter(
      rclcpp::Parameter("exception_to_throw", rclcpp::ParameterValue(error_ids[i])));

    auto client = rclcpp_action::create_client<FollowObject>(node2, "follow_object");
    if (!client->wait_for_action_server(1s)) {
      RCLCPP_ERROR(node2->get_logger(), "Action server not available after waiting");
    }
    auto goal_msg = FollowObject::Goal();
    goal_msg.object_pose.header.frame_id = "test_frame";

    auto future_goal_handle = client->async_send_goal(goal_msg);

    if (rclcpp::spin_until_future_complete(
        node2, future_goal_handle, 2s) == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto future_result = client->async_get_result(future_goal_handle.get());
      if (rclcpp::spin_until_future_complete(
          node2, future_result, 5s) == rclcpp::FutureReturnCode::SUCCESS)
      {
        auto result = future_result.get();
        EXPECT_EQ(result.result->error_code, error_codes[i]);
      } else {
        EXPECT_TRUE(false);
      }
    } else {
      EXPECT_TRUE(false);
    }
  }

  // Set follow_action_called to true to simulate robot following object
  node->set_parameter(rclcpp::Parameter("follow_action_called", true));

  node->on_deactivate(rclcpp_lifecycle::State());
  node->on_cleanup(rclcpp_lifecycle::State());
  node->on_shutdown(rclcpp_lifecycle::State());
  node.reset();
}

TEST(FollowingServerTests, GetPoseAtDistance)
{
  auto node = std::make_shared<opennav_following::FollowingServerShim>();
  node->on_configure(rclcpp_lifecycle::State());

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = node->now();
  pose.header.frame_id = "my_frame";
  pose.pose.position.x = 1.0;
  pose.pose.position.y = -1.0;

  auto new_pose = node->getPoseAtDistance(pose, 0.2);
  EXPECT_NEAR(new_pose.pose.position.x, 0.8, 0.01);
  EXPECT_NEAR(new_pose.pose.position.y, -1.0, 0.01);

  node->on_cleanup(rclcpp_lifecycle::State());
  node->on_shutdown(rclcpp_lifecycle::State());
  node.reset();
}

TEST(FollowingServerTests, IsGoalReached)
{
  auto node = std::make_shared<opennav_following::FollowingServerShim>();
  node->on_configure(rclcpp_lifecycle::State());

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = node->now();
  pose.header.frame_id = "my_frame";
  pose.pose.position.x = 1.0;
  pose.pose.position.y = -1.0;

  EXPECT_FALSE(node->isGoalReached(pose));

  // Set the pose below the tolerance
  pose.pose.position.x = 0.1;
  pose.pose.position.y = 0.1;
  EXPECT_TRUE(node->isGoalReached(pose));

  node->on_cleanup(rclcpp_lifecycle::State());
  node->on_shutdown(rclcpp_lifecycle::State());
  node.reset();
}

TEST(FollowingServerTests, RefinedPoseTest)
{
  auto node = std::make_shared<opennav_following::FollowingServerShim>();
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "detected_dynamic_pose", rclcpp::QoS(1));
  pub->on_activate();

  // Set filter coefficient to 0, so no filtering is done
  node->set_parameter(rclcpp::Parameter("filter_coef", rclcpp::ParameterValue(0.0)));

  node->on_configure(rclcpp_lifecycle::State());
  node->on_activate(rclcpp_lifecycle::State());

  geometry_msgs::msg::PoseStamped pose;

  // Timestamps are outdated; this is after timeout
  EXPECT_FALSE(node->getRefinedPose(pose));

  // Set skip orientation to false
  node->setSkipOrientation(false);

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

  auto filtered_pose = node->getFilteredPose();
  EXPECT_NEAR(filtered_pose.pose.position.x, 0.1, 0.01);
  EXPECT_NEAR(filtered_pose.pose.position.y, -0.1, 0.01);
  EXPECT_DOUBLE_EQ(tf2::getYaw(pose.pose.orientation), tf2::getYaw(filtered_pose.pose.orientation));

  // Now, set skip orientation to true
  node->setSkipOrientation(true);

  detected_pose.header.stamp = node->now();
  pub->publish(detected_pose);
  rclcpp::spin_some(node->get_node_base_interface());

  EXPECT_TRUE(node->getRefinedPose(pose));
  EXPECT_NEAR(pose.pose.position.x, 0.1, 0.01);
  EXPECT_NEAR(pose.pose.position.y, -0.1, 0.01);

  filtered_pose = node->getFilteredPose();
  EXPECT_NEAR(filtered_pose.pose.position.x, 0.1, 0.01);
  EXPECT_NEAR(filtered_pose.pose.position.y, -0.1, 0.01);
  EXPECT_NEAR(tf2::getYaw(filtered_pose.pose.orientation), -0.785, 0.01);

  node->on_deactivate(rclcpp_lifecycle::State());
  node->on_cleanup(rclcpp_lifecycle::State());
  node->on_shutdown(rclcpp_lifecycle::State());
  node.reset();
}

TEST(FollowingServerTests, DynamicParams)
{
  auto node = std::make_shared<opennav_following::FollowingServer>();
  node->on_configure(rclcpp_lifecycle::State());
  node->on_activate(rclcpp_lifecycle::State());

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  // Set parameters
  auto results = params->set_parameters_atomically(
    {rclcpp::Parameter("controller_frequency", 1.0),
      rclcpp::Parameter("initial_perception_timeout", 2.0),
      rclcpp::Parameter("detection_timeout", 3.0),
      rclcpp::Parameter("desired_distance", 4.0),
      rclcpp::Parameter("linear_tolerance", 5.0),
      rclcpp::Parameter("angular_tolerance", 6.0),
      rclcpp::Parameter("base_frame", std::string("test_base_frame")),
      rclcpp::Parameter("fixed_frame", std::string("test_fixed_frame")),
      rclcpp::Parameter("skip_orientation", false)});

  // Spin
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Check parameters
  EXPECT_EQ(node->get_parameter("controller_frequency").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("initial_perception_timeout").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("detection_timeout").as_double(), 3.0);
  EXPECT_EQ(node->get_parameter("desired_distance").as_double(), 4.0);
  EXPECT_EQ(node->get_parameter("linear_tolerance").as_double(), 5.0);
  EXPECT_EQ(node->get_parameter("angular_tolerance").as_double(), 6.0);
  EXPECT_EQ(node->get_parameter("base_frame").as_string(), "test_base_frame");
  EXPECT_EQ(node->get_parameter("fixed_frame").as_string(), "test_fixed_frame");
  EXPECT_EQ(node->get_parameter("skip_orientation").as_bool(), false);
}

}  // namespace opennav_following
