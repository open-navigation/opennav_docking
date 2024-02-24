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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    carter_navigation_launch_dir = os.path.join(
        get_package_share_directory('carter_navigation'), 'launch')
    nova_carter_dock_launch_dir = os.path.join(
        get_package_share_directory('nova_carter_docking'), 'launch')
    nova_carter_dock_params_dir = os.path.join(
        get_package_share_directory('nova_carter_docking'), 'params')

    params_file = default_value=os.path.join(nova_carter_dock_params_dir, 'nova_carter_docking.yaml')

    robot_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            carter_navigation_launch_dir, 'teleop.launch.py')),
        launch_arguments={
            'launch_hawks': 'True',      # stereo
            'launch_owls': 'False',      # fisheye
            'launch_rplidars': 'False',  # 2D
            'launch_hesai': 'False',     # 3D
            'launch_segway': 'True'      # base
        }.items(),
    )

    dock_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nova_carter_dock_launch_dir, 'isaac_apriltag_detection_pipeline.launch.py'))
    )

    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[params_file],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': ['docking_server']}],
    )

    return LaunchDescription([
        robot_base_launch,
        dock_detection_launch,
        docking_server,
        lifecycle_manager
    ])
