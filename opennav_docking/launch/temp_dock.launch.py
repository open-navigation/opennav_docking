#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    autostart = LaunchConfiguration('autostart')
    lifecycle_nodes = ['docking_server']

    return LaunchDescription([
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            parameters=[{'dock_plugins': ['temp_dock_plugin'],
                         'temp_dock_plugin': {'plugin': 'opennav_docking::TempChargingDock'},
                         'docks': ['test_dock'],
                         'test_dock': {'type': 'temp_dock_plugin', 'frame': 'test_frame', 'pose': [0.0, 0.0, 0.0]}}],
            output='screen',
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),
    ])
