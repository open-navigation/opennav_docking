#!/usr/bin/env python3

# Copyright (c) 2024 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time

from action_msgs.msg import GoalStatus
from opennav_docking_msgs.action import DockRobot
import rclpy
from rclpy.action import ActionClient


class Dock:

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('dock_robot')
        self.is_charging = False

    def action_goal_callback(self, future):
        self.goal_handle = future.result()
        assert self.goal_handle.accepted
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.action_result_callback)

    def action_result_callback(self, future):
        self.action_result.append(future.result())
        print(future.result())

    def action_feedback_callback(self, msg):
        # Force the docking action to run a full recovery loop and then
        # make contact with the dock (based on pose of robot) before
        # we report that the robot is charging
        if msg.feedback.num_retries > 0 and \
                msg.feedback.state == msg.feedback.WAIT_FOR_CHARGE:
            self.is_charging = True

    def dock(self):
        # Create action client
        self.dock_action_client = ActionClient(self.node, DockRobot, 'dock_robot')

        # Test docking action
        self.action_result = []
        self.dock_action_client.wait_for_server(timeout_sec=5.0)
        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = 'test_dock'
        future = self.dock_action_client.send_goal_async(
            goal, feedback_callback=self.action_feedback_callback)
        future.add_done_callback(self.action_goal_callback)

        # Wait until complete
        while len(self.action_result) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

        assert self.action_result[0].status == GoalStatus.STATUS_ABORTED
        assert not self.action_result[0].result.success


if __name__ == '__main__':
    d = Dock()
    d.dock()
