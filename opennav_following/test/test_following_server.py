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

from math import cos, sin
import time
import unittest

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
from nav2_msgs.action import FollowObject
import pytest
import rclpy
from rclpy.action import ActionClient
from tf2_ros import TransformBroadcaster


# This test can be run standalone with:
# python3 -u -m pytest test_following_server.py -s

@pytest.mark.rostest
def generate_test_description():

    return LaunchDescription([
        Node(
            package='opennav_following',
            executable='opennav_following',
            name='following_server',
            parameters=[{'desired_distance': 0.5,
                         'linear_tolerance': 0.05,
                         'angular_tolerance': 0.05,
                         'controller': {
                             'use_collision_detection': False,
                             'transform_tolerance': 0.5,
                         }}],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['following_server']}]
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestFollowingServer(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        # Latest odom -> base_link
        cls.x = 0.0
        cls.y = 0.0
        cls.theta = 0.0
        # Track at distance state
        cls.at_distance = False
        # Latest command velocity
        cls.command = Twist()
        cls.node = rclpy.create_node('test_following_server')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def command_velocity_callback(self, msg):
        self.node.get_logger().info('Command: %f %f' % (msg.twist.linear.x, msg.twist.angular.z))
        self.command = msg.twist

    def timer_callback(self):
        # Propagate command
        period = 0.05
        self.x += cos(self.theta) * self.command.linear.x * period
        self.y += sin(self.theta) * self.command.linear.x * period
        self.theta += self.command.angular.z * period
        # Need to publish updated TF
        self.publish()

    def publish(self):
        # Publish base->odom transform
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = sin(self.theta / 2.0)
        t.transform.rotation.w = cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)
        # Publish a fixed object pose at 1.75m in front of the robot
        p = PoseStamped()
        p.header.stamp = self.node.get_clock().now().to_msg()
        p.header.frame_id = 'odom'
        p.pose.position.x = 1.75
        p.pose.position.y = 0.0
        self.object_pose_pub.publish(p)

    def action_feedback_callback(self, msg):
        # Set at_distance flag when the robot is stopping
        if msg.feedback.state == msg.feedback.STOPPING:
            self.at_distance = True

    def test_following_server(self):
        # Publish TF for odometry
        self.tf_broadcaster = TransformBroadcaster(self.node)

        # Create a timer to run "control loop" at 20hz
        self.timer = self.node.create_timer(0.05, self.timer_callback)

        # Create action client
        self.follow_action_client = ActionClient(self.node, FollowObject, 'follow_object')

        # Subscribe to command velocity
        self.node.create_subscription(
            TwistStamped,
            'cmd_vel',
            self.command_velocity_callback,
            10
        )

        # Create publisher for the object pose used in the perception
        self.object_pose_pub = self.node.create_publisher(
            PoseStamped,
            'detected_dynamic_pose',
            10
        )

        # Spin once so that TF is published
        rclpy.spin_once(self.node, timeout_sec=0.1)

        # Test follow action with an object at 1.75m in front of the robot
        self.action_result = []
        self.follow_action_client.wait_for_server(timeout_sec=5.0)
        goal = FollowObject.Goal()
        goal.object_pose.header.frame_id = 'odom'
        goal.object_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.object_pose.pose.position.x = 1.75
        goal.object_pose.pose.position.y = 0.0
        goal.max_duration = rclpy.time.Duration(seconds=5.0).to_msg()
        future = self.follow_action_client.send_goal_async(
            goal, feedback_callback=self.action_feedback_callback)
        rclpy.spin_until_future_complete(self.node, future)
        self.goal_handle = future.result()
        assert self.goal_handle.accepted
        result_future_original = self.goal_handle.get_result_async()

        # Run for 2 seconds
        for i in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

        # Send another goal to preempt the first
        future = self.follow_action_client.send_goal_async(
            goal, feedback_callback=self.action_feedback_callback)
        rclpy.spin_until_future_complete(self.node, future)
        self.goal_handle = future.result()
        assert self.goal_handle.accepted
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        self.action_result.append(result_future.result())

        rclpy.spin_until_future_complete(self.node, result_future_original)
        self.action_result.append(result_future_original.result())

        # First is aborted due to preemption
        self.assertEqual(self.action_result[0].status, GoalStatus.STATUS_ABORTED)
        self.assertTrue(self.action_result[0].result, FollowObject.Result.NONE)
        self.assertFalse(self.at_distance)

        self.node.get_logger().info('Goal preempted')

        # Run for 0.5 seconds
        for i in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

        # Second is aborted due to preemption during main loop (takes down all actions)
        self.assertEqual(self.action_result[1].status, GoalStatus.STATUS_ABORTED)
        self.assertTrue(self.action_result[0].result, FollowObject.Result.NONE)
        self.assertFalse(self.at_distance)

        # Resend the goal
        self.node.get_logger().info('Sending goal again')
        future = self.follow_action_client.send_goal_async(
            goal, feedback_callback=self.action_feedback_callback)
        rclpy.spin_until_future_complete(self.node, future)
        self.goal_handle = future.result()
        assert self.goal_handle.accepted
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        self.action_result.append(result_future.result())

        self.assertEqual(self.action_result[2].status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(self.action_result[0].result, FollowObject.Result.TIMEOUT)
        self.assertTrue(self.at_distance)
