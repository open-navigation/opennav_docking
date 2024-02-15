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

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            parameters=[{'controller_frequency': 50.0,
                         'controller': {'k_phi': 3.0,
                                        'k_delta': 2.0,
                                        'v_linear_min': 0.15,
                                        'v_linear_max': 0.15},
                         'wait_charge_timeout': 10.0,
                         'dock_plugins': ['simple_dock_plugin'],
                         'simple_dock_plugin': {'plugin': 'opennav_docking::SimpleChargingDock',
                                                # This is relatively high, because we the "dock"
                                                # pose is actually set quite far in front of the
                                                # robot. If the base_link gets very close to the
                                                # dock pose, the control law can generate large
                                                # rotational velocities.
                                                'docking_threshold': 0.3,
                                                'staging_x_offset': -0.85,
                                                # We lose marker when we get close at the very end
                                                'external_detection_timeout': 2.0,
                                                'use_external_detection_pose': True,
                                                'external_detection_translation_x': -0.1,
                                                'external_detection_translation_y': 0.0,
                                                'external_detection_rotation_roll': -1.57,
                                                'external_detection_rotation_pitch': 1.57,
                                                'external_detection_rotation_yaw': 0.0,
                                                'filter_coef': 0.1},
                         'docks': ['test_dock'],
                         'test_dock': {'type': 'simple_dock_plugin',
                                       'frame': 'map',
                                       'pose': [0.552521, -0.173559, -0.221314]}}],
            # When testing sketchy code, it is helpful to keep robot from moving
            # remappings=[('cmd_vel', 'crap')],
            output='screen',
        ),

        Node(
            package='image_proc',
            executable='track_marker_node',
            name='dock_marker_tracker_node',
            parameters=[{'marker_id': 0, 'marker_size': 0.159}],
            remappings=[('tracked_pose', 'detected_dock_pose'),
                        ('image', 'head_camera/rgb/image_raw')],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['docking_server']}]),
    ])
