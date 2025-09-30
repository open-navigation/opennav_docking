#!/usr/bin/env python3

# Copyright (c) 2024 Open Navigation LLC
# Copyright (c) 2024 Alberto J. Tudela Roldán
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

from geometry_msgs.msg import PoseStamped
from opennav_following_msgs.action import FollowObject
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient


class FollowObjectTester(Node):

    def __init__(self):
        super().__init__(node_name='follow_object_tester')
        # Create a one-shot timer to call the action server after a short delay (2 seconds)
        self.timer = self.create_timer(2, self.timer_callback)
        # self.subscription  # prevent unused variable warning
        self.follow_action_client = ActionClient(self, FollowObject, 'follow_object')

    def timer_callback(self):
        """Callback for the timer to send the goal to the action server."""
        if self.send_goal():
            self.get_logger().info('Goal sent successfully.')
            self.timer.cancel()  # Cancel the timer after sending the goal
        else:
            self.get_logger().error('Failed to send goal to follow object action server.')

    def action_feedback_callback(self, msg):
        """Prints the feedback message from the action server."""
        if msg.feedback.state == msg.feedback.INITIAL_PERCEPTION:
            self.get_logger().info('Initial perception of the object.')
        elif msg.feedback.state == msg.feedback.CONTROLLING:
            self.get_logger().info('Controlling the robot to follow the object.')
        elif msg.feedback.state == msg.feedback.STOPPING:
            self.get_logger().info('Stopping the robot.')
        elif msg.feedback.state == msg.feedback.RETRY:
            self.get_logger().info('Retrying to follow the object.')

    def send_goal(self):
        """Calls the action server."""
        self.get_logger().info('Calling the follow object action.')
        while not self.follow_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('"FollowObject" action server not available, waiting...')

        goal_msg = FollowObject.Goal()
        future = self.follow_action_client.send_goal_async(
            goal_msg, feedback_callback=self.action_feedback_callback)
        self.goal_handle = future.result()

        return True


def main(args=None):
    rclpy.init(args=args)
    node = FollowObjectTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
