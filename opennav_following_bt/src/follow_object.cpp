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

#include <memory>
#include <string>

#include "opennav_following_bt/follow_object.hpp"

namespace opennav_following_bt
{

FollowObjectAction::FollowObjectAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

void FollowObjectAction::on_tick()
{
  // Get core inputs about what to perform
  double max_duration;
  getInput("object_pose", goal_.object_pose);
  getInput("max_duration", max_duration);

  // Populate the input message
  goal_.max_duration = rclcpp::Duration::from_seconds(max_duration);
}

BT::NodeStatus FollowObjectAction::on_success()
{
  setOutput("total_elapsed_time", result_.result->total_elapsed_time);
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FollowObjectAction::on_aborted()
{
  setOutput("total_elapsed_time", result_.result->total_elapsed_time);
  setOutput("error_code_id", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus FollowObjectAction::on_cancelled()
{
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

void FollowObjectAction::halt()
{
  BtActionNode::halt();
}

}  // namespace opennav_following_bt

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<opennav_following_bt::FollowObjectAction>(
        name, "follow_object", config);
    };

  factory.registerBuilder<opennav_following_bt::FollowObjectAction>(
    "FollowObject", builder);
}
