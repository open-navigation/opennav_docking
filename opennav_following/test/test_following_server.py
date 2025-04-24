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
from opennav_following_msgs.action import FollowObject
import pytest
import rclpy
from rclpy.action import ActionClient
from tf2_ros import TransformBroadcaster

# This test can be run standalone with:
# python3 -u -m pytest test_following_server.py -s

# If python3-flaky is installed, you can run the test multiple times to
# try to identify flaky ness.
# python3 -u -m pytest --force-flaky --min-passes 3 --max-runs 5 -s -v test_following_server.py


@pytest.mark.rostest
# @pytest.mark.flaky
# @pytest.mark.flaky(max_runs=5, min_passes=3)
def generate_test_description() -> LaunchDescription:

    return LaunchDescription([
        # SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        # SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        Node(
            package='opennav_following',
            executable='opennav_following',
            name='following_server',
            parameters=[{'desired_distance': 0.5,
                         'detection_timeout': 0.1,
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
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        # Create a ROS node for tests
        # Latest odom -> base_link
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # Track states
        self.at_distance = False
        self.retry_state = False
        # Latest command velocity
        self.command = Twist()
        self.node = rclpy.create_node('test_following_server')

    def tearDown(self) -> None:
        self.node.destroy_node()

    def command_velocity_callback(self, msg: TwistStamped) -> None:
        self.node.get_logger().info(f'Command: {msg.twist.linear.x:f} {msg.twist.angular.z:f}')
        self.command = msg.twist

    def timer_callback(self) -> None:
        # Propagate command
        period = 0.05
        self.x += cos(self.theta) * self.command.linear.x * period
        self.y += sin(self.theta) * self.command.linear.x * period
        self.theta += self.command.angular.z * period
        # Need to publish updated TF
        self.publish()

    def publish(self) -> None:
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
        # Stop the object from moving when the robot reaches it, to force a recovery
        if not self.at_distance:
            self.object_pose_pub.publish(p)

    def action_feedback_callback(self, msg: FollowObject.Feedback) -> None:
        # Force the following action to run a full recovery loop when
        # the robot is at distance
        if msg.feedback.state == msg.feedback.STOPPING:
            self.at_distance = True
        elif msg.feedback.state == msg.feedback.RETRY:
            self.at_distance = False
            self.retry_state = True

    def test_following_server(self) -> None:
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

        # Publish transform
        self.publish()

        # Run for 1 seconds to allow tf to propagate
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

        # Test follow action with an object at 1.75m in front of the robot
        self.action_result = []
        assert self.follow_action_client.wait_for_server(timeout_sec=5.0), \
            'follow_object service not available'

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
        assert self.goal_handle is not None, 'goal_handle should not be None'
        assert self.goal_handle.accepted, 'goal_handle not accepted'
        result_future_original = self.goal_handle.get_result_async()

        # Run for 2 seconds
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

        # Send another goal to preempt the first
        future = self.follow_action_client.send_goal_async(
            goal, feedback_callback=self.action_feedback_callback)
        rclpy.spin_until_future_complete(self.node, future)
        self.goal_handle = future.result()
        assert self.goal_handle is not None, 'goal_handle should not be None'
        assert self.goal_handle.accepted, 'goal_handle not accepted'
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        self.action_result.append(result_future.result())

        rclpy.spin_until_future_complete(self.node, result_future_original)
        self.action_result.append(result_future_original.result())

        # First is aborted due to preemption
        self.assertIsNotNone(self.action_result[0])
        if self.action_result[0] is not None:
            self.assertEqual(self.action_result[0].status, GoalStatus.STATUS_ABORTED)
            self.assertTrue(self.action_result[0].result, FollowObject.Result.NONE)
            self.assertFalse(self.at_distance)

        self.node.get_logger().info('Goal preempted')

        # Run for 0.5 seconds
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

        # Second is aborted due to preemption during main loop (takes down all actions)
        self.assertIsNotNone(self.action_result[1])
        if self.action_result[1] is not None:
            self.assertEqual(self.action_result[1].status, GoalStatus.STATUS_ABORTED)
            self.assertTrue(self.action_result[1].result, FollowObject.Result.NONE)
            self.assertFalse(self.at_distance)

        # Resend the goal
        self.node.get_logger().info('Sending goal again')
        future = self.follow_action_client.send_goal_async(
            goal, feedback_callback=self.action_feedback_callback)
        rclpy.spin_until_future_complete(self.node, future)
        self.goal_handle = future.result()
        assert self.goal_handle is not None, 'goal_handle should not be None'
        assert self.goal_handle.accepted, 'goal_handle not accepted'
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        self.action_result.append(result_future.result())

        self.assertIsNotNone(self.action_result[2])
        if self.action_result[2] is not None:
            self.assertEqual(self.action_result[2].status, GoalStatus.STATUS_SUCCEEDED)
            self.assertTrue(self.action_result[2].result, FollowObject.Result.TIMEOUT)
            self.assertEqual(self.action_result[2].result.num_retries, 2)
            self.assertTrue(self.at_distance)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_info: launch_testing.ProcInfoHandler) -> None:
        # Check that all processes in the launch exit with code 0
        launch_testing.asserts.assertExitCodes(proc_info)
