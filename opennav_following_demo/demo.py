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
        self.subscription = self.create_subscription(
            PoseStamped, 'detected_dynamic_pose', self.pose_callback, 10)
        # self.subscription  # prevent unused variable warning
        self.follow_action_client = ActionClient(self, FollowObject, 'follow_object')
        self.first_message_received = False

    def pose_callback(self, msg):
        """Subscribes to the pose topic and calls the action server with the first pose."""
        if not self.first_message_received:
            self.first_message_received = True
            self.send_goal(msg)

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

    def send_goal(self, pose):
        """Sends the pose to the action server."""
        self.get_logger().info('Sending the pose to follow.')
        while not self.follow_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('"FollowObject" action server not available, waiting...')

        goal_msg = FollowObject.Goal()
        goal_msg.object_pose = pose
        orientation_q = pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        # Print the pose in (x,y) format
        self.get_logger().info(
            'Pose received in frame ' + pose.header.frame_id +
            ': x=' + str(pose.pose.position.x) +
            ', y=' + str(pose.pose.position.y)
        )

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
