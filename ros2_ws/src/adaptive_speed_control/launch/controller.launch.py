#!/usr/bin/env python3
"""
Stand-alone launch file for the Adaptive Speed Controller and Navigator.
Use this when Gazebo or physical robot is already running.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_adaptive = get_package_share_directory('adaptive_speed_control')
    params_path = os.path.join(pkg_adaptive, 'params', 'controller_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    start_navigator = LaunchConfiguration('start_navigator', default='true')
    nav_mode = LaunchConfiguration('mode', default='adaptive')

    adaptive_controller_node = Node(
        package='adaptive_speed_control',
        executable='adaptive_speed_controller',
        name='adaptive_speed_controller',
        output='screen',
        parameters=[params_path, {'use_sim_time': use_sim_time}],
    )

    navigator_node = Node(
        package='adaptive_speed_control',
        executable='clutter_navigator',
        name='clutter_navigator',
        output='screen',
        parameters=[params_path, {'mode': nav_mode, 'use_sim_time': use_sim_time}],
        condition=IfCondition(start_navigator)
    )

    telemetry_hud_node = Node(
        package='adaptive_speed_control',
        executable='speed_telemetry_hud',
        name='speed_telemetry_hud',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('start_navigator', default_value='true'))
    ld.add_action(DeclareLaunchArgument('mode', default_value='adaptive'))

    ld.add_action(adaptive_controller_node)
    ld.add_action(navigator_node)
    ld.add_action(telemetry_hud_node)

    return ld
