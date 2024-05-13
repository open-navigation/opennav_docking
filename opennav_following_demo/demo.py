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
        self.follow_client = ActionClient(self, FollowObject, 'follow_object')
        self.first_message_received = False
        self.goal_handle = None

    def pose_callback(self, msg):
        """Subscribes to the pose topic and calls the action server with the first pose."""
        if not self.first_message_received:
            self.first_message_received = True
            self.followObject(msg)

    def followObject(self, pose):
        """Sends the pose to the action server."""
        self.get_logger().info('Sending the pose to follow.')
        while not self.follow_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('"FollowObject" action server not available, waiting...')

        goal_msg = FollowObject.Goal()
        goal_msg.object_pose = pose
        orientation_q = pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        # Print the pose in (x,y) format
        self.get_logger().info('Pose received in frame ' + pose.header.frame_id +
                               ': x=' + str(pose.pose.position.x) + 
                               ', y=' + str(pose.pose.position.y))

        self.get_logger().info('Sending pose ' + str(pose) + '...')
        send_goal_future = self.follow_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            print('Following request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True


def main(args=None):
    rclpy.init(args=args)
    node = FollowObjectTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
