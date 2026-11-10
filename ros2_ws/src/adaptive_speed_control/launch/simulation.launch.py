#!/usr/bin/env python3
"""
Complete Simulation Launch file for Adaptive Speed Control in Gazebo Harmonic.
Spawns TurtleBot3 into the cluttered arena world, launches ROS-Gz bridge,
the Adaptive Speed Controller, and Autonomous Navigator.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_adaptive = get_package_share_directory('adaptive_speed_control')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Default model Burger if not already set in environment
    if 'TURTLEBOT3_MODEL' not in os.environ:
        os.environ['TURTLEBOT3_MODEL'] = 'burger'

    # Launch Configurations
    model = LaunchConfiguration('model', default='burger')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-7.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    start_navigator = LaunchConfiguration('start_navigator', default='true')
    nav_mode = LaunchConfiguration('mode', default='adaptive')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    default_world = os.path.join(pkg_adaptive, 'worlds', 'cluttered_arena.world')
    world = LaunchConfiguration('world', default=default_world)

    params_path = os.path.join(pkg_adaptive, 'params', 'controller_params.yaml')
    rviz_config_path = os.path.join(pkg_adaptive, 'rviz', 'adaptive_speed.rviz')

    # Set GZ_SIM_RESOURCE_PATH to include TurtleBot3 models
    set_env_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_tb3_gazebo, 'models')
    )
    set_tb3_model_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', model)

    # 1. Gazebo Sim (Single unified process: physics engine + 3D GUI window)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[
            ('gz_args', ['-r -v2 --render-engine-gui ogre ', world]),
            ('on_exit_shutdown', 'true')
        ]
    )

    # 3. Robot State Publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. Spawn Robot & Parameter Bridge
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # 5. Adaptive Speed Controller Node
    adaptive_controller_node = Node(
        package='adaptive_speed_control',
        executable='adaptive_speed_controller',
        name='adaptive_speed_controller',
        output='screen',
        parameters=[params_path, {'use_sim_time': use_sim_time}],
    )

    # 6. Autonomous Clutter Navigator Node
    navigator_node = Node(
        package='adaptive_speed_control',
        executable='clutter_navigator',
        name='clutter_navigator',
        output='screen',
        parameters=[params_path, {'mode': nav_mode, 'use_sim_time': use_sim_time}],
        condition=IfCondition(start_navigator)
    )

    # 7. Speed Telemetry HUD (Live console dashboard)
    telemetry_hud_node = Node(
        package='adaptive_speed_control',
        executable='speed_telemetry_hud',
        name='speed_telemetry_hud',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 8. RViz2 (Optional visualizer)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    ld = LaunchDescription()

    # Declare Arguments
    ld.add_action(DeclareLaunchArgument('model', default_value='burger', description='TurtleBot3 model: burger or waffle'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use Gazebo simulation clock'))
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='-7.0', description='Initial robot X coordinate'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='0.0', description='Initial robot Y coordinate'))
    ld.add_action(DeclareLaunchArgument('start_navigator', default_value='true', description='Start autonomous navigation node'))
    ld.add_action(DeclareLaunchArgument('mode', default_value='adaptive', description='Navigation mode: adaptive or fixed'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true', description='Open RViz2 visualization'))
    ld.add_action(DeclareLaunchArgument('world', default_value=default_world, description='World SDF file path'))

    # Environment
    ld.add_action(set_tb3_model_env)
    ld.add_action(set_env_models)

    # Simulation and Robot nodes
    ld.add_action(gz_sim)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)

    # Speed Controller and Navigator
    ld.add_action(adaptive_controller_node)
    ld.add_action(navigator_node)
    ld.add_action(telemetry_hud_node)
    ld.add_action(rviz_node)

    return ld
