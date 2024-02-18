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

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

def generate_launch_description():
    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        remappings=[
            ('image', '/hawk_front/left/image_raw'),        # /owl_front/left/image_raw
            ('camera_info', '/hawk_front/left/camerainfo')  # /owl_front/left/camerainfo
        ],
        parameters=[{'size': 0.1524,  # 6 inches
                     'max_tags': 4,
                     'tile_size': 4}])

    apriltag_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            apriltag_node,
        ],
        output='screen'
    )

    dock_pose_publisher = Node(
        package='nova_carter_dock',
        executable='dock_pose_publisher',
        name='dock_pose_publisher',
        parameters=[{'use_first_detection': False}],
    )

    return launch.LaunchDescription([apriltag_container, dock_pose_publisher])
