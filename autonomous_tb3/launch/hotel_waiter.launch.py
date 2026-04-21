#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#
# Migrated from ROS 2 Humble (Gazebo Classic) to ROS 2 Jazzy (Gazebo Harmonic)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_autonomous_tb3 = get_package_share_directory('autonomous_tb3')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    hotel_path = os.path.join(pkg_autonomous_tb3, 'world', 'hotel', 'model.sdf')
    table_path = os.path.join(pkg_autonomous_tb3, 'models', 'table', 'model.sdf')
    # actor_path = os.path.join(pkg_autonomous_tb3, 'models', 'actor', 'model.sdf')  # unused
    config_dir = os.path.join(pkg_autonomous_tb3, 'config')
    map_file = os.path.join(config_dir, 'hotel_map.yaml')
    params_file = os.path.join(config_dir, 'tb3_nav_params.yaml')
    nav_config = os.path.join(config_dir, 'tb3_nav.rviz')
    world_path = os.path.join(pkg_autonomous_tb3, 'world', 'hotel', 'empty_world.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-4.5')
    y_pose = LaunchConfiguration('y_pose', default='0.51')

    # ── Gazebo Harmonic: headless server ────────────────────────────────────
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world_path],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ── Gazebo Harmonic: GUI client ─────────────────────────────────────────
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 '}.items()
    )

    # ── Robot state publisher (TB3 Jazzy – unchanged interface) ────────────
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ── Spawn TurtleBot3 (TB3 Jazzy launch – includes ros_gz_bridge) ───────
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # ── Expose custom model directory to Gazebo Harmonic ────────────────────
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_autonomous_tb3, 'models')
    )

    # ── Spawn hotel building and furniture via ros_gz_sim create ───────────
    hotel_spawner = Node(
        package='autonomous_tb3',
        output='screen',
        executable='sdf_spawner',
        name='hotel_spawner',
        arguments=[hotel_path, 'hotel', '0.0', '0.0']
    )

    table_spawner_1 = Node(
        package='autonomous_tb3',
        output='screen',
        executable='sdf_spawner',
        name='table_spawner_1',
        arguments=[table_path, 'table_1', '-0.6', '3.99']
    )

    table_spawner_2 = Node(
        package='autonomous_tb3',
        output='screen',
        executable='sdf_spawner',
        name='table_spawner_2',
        arguments=[table_path, 'table_2', '4.52', '3.99']
    )

    table_spawner_3 = Node(
        package='autonomous_tb3',
        output='screen',
        executable='sdf_spawner',
        name='table_spawner_3',
        arguments=[table_path, 'table_3', '4.53', '-3.17']
    )

    table_spawner_4 = Node(
        package='autonomous_tb3',
        output='screen',
        executable='sdf_spawner',
        name='table_spawner_4',
        arguments=[table_path, 'table_4', '-0.6', '-3.09']
    )

    # ── Nav2 stack ──────────────────────────────────────────────────────────
    hotel_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('nav2_bringup'), '/launch', '/bringup_launch.py']
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'use_collision_monitor': 'False'
        }.items(),
    )

    # ── RViz2 ───────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        output='screen',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d', nav_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()

    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    
    delayed_spawn = TimerAction(
        period=15.0,
        actions=[spawn_turtlebot_cmd]
    )
    ld.add_action(delayed_spawn)
    
    ld.add_action(hotel_spawner)
    ld.add_action(table_spawner_1)
    ld.add_action(table_spawner_2)
    ld.add_action(table_spawner_3)
    ld.add_action(table_spawner_4)
    ld.add_action(rviz)
    ld.add_action(hotel_nav)

    return ld