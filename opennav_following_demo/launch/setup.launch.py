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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_dir = os.path.join(get_package_share_directory('opennav_following_demo'), 'params')

    params_file = os.path.join(params_dir, 'following_demo.yaml')

    following_server = Node(
        package='opennav_following',
        executable='opennav_following',
        name='following_server',
        output='screen',
        parameters=[params_file]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_following',
        output='screen',
        parameters=[{'autostart': True}, {'use_sim_time': False},
                    {'node_names': ['following_server']}],
    )

    return LaunchDescription([
        following_server,
        lifecycle_manager
    ])
