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

from math import acos, cos, sin
import unittest

from geometry_msgs.msg import TransformStamped, Twist
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
from nav2_msgs.action import NavigateToPose
from opennav_docking_msgs.action import DockRobot, UndockRobot
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer
from tf2_ros import TransformBroadcaster


@pytest.mark.rostest
def generate_test_description():

    return LaunchDescription([
        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            parameters=[{'dock_plugins': ['temp_dock_plugin'],
                         'temp_dock_plugin': {'plugin': 'opennav_docking::TempChargingDock'},
                         'docks': ['test_dock'],
                         'test_dock': {
                            'type': 'temp_dock_plugin',
                            'frame': 'odom',
                            'pose': [1.0, 0.0, 0.0]
                         }}],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['docking_server']}]
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestDockingServer(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        # Latest odom -> base_link
        cls.x = 0.0
        cls.y = 0.0
        cls.theta = 0.0
        # Latest command velocity
        cls.command = Twist()
        cls.node = rclpy.create_node('test_docking_server')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def command_velocity_callback(self, msg):
        self.node.get_logger().info('Command: %f %f' % (msg.linear.x, msg.angular.z))
        self.command = msg

    def timer_callback(self):
        # Propagate command
        period = 0.05
        self.x += cos(self.theta) * self.command.linear.x * period
        self.y += sin(self.theta) * self.command.linear.x * period
        self.theta += self.command.angular.z * period
        self.publish_tf()

    def publish_tf(self):
        # Publish transform
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = sin(self.theta / 2.0)
        t.transform.rotation.w = cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

    def dock_robot_goal_callback(self, future):
        goal_handle = future.result()
        assert goal_handle.accepted
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.dock_robot_result_callback)

    def dock_robot_result_callback(self, future):
        self.dock_result = future.result().status
        print(future.result())

    def nav_execute_callback(self, goal_handle):
        goal = goal_handle.request
        self.x = goal.pose.pose.position.x - 0.05
        self.y = goal.pose.pose.position.y + 0.05
        self.theta = 2.0 * acos(goal.pose.pose.orientation.w)
        self.node.get_logger().info('Navigating to %f %f %f' % (self.x, self.y, self.theta))
        goal_handle.succeed()
        self.publish_tf()

        result = NavigateToPose.Result()
        result.error_code = 0
        return result

    def test_docking_server(self):
        # Publish TF for odometry
        self.tf_broadcaster = TransformBroadcaster(self.node)

        # Create a timer to run "control loop" at 20hz
        self.timer = self.node.create_timer(0.05, self.timer_callback)

        # Create action client
        self.dock_action_client = ActionClient(self.node, DockRobot, 'dock_robot')
        self.undock_action_client = ActionClient(self.node, UndockRobot, 'undock_robot')

        # Subscribe to command velocity
        self.node.create_subscription(
            Twist,
            'cmd_vel',
            self.command_velocity_callback,
            10
        )

        # Mock out navigation server (just teleport the robot)
        self.action_server = ActionServer(
            self.node,
            NavigateToPose,
            'navigate_to_pose',
            self.nav_execute_callback)

        # Spin once so that TF is published
        rclpy.spin_once(self.node, timeout_sec=0.1)

        # Test docking action
        self.dock_result = None
        self.dock_action_client.wait_for_server(timeout_sec=5.0)
        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = 'test_dock'
        future = self.dock_action_client.send_goal_async(goal)
        future.add_done_callback(self.dock_robot_goal_callback)

        while self.dock_result is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Test undocking action
