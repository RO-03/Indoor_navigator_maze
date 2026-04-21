#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#
# Migrated from ROS 2 Humble (Gazebo Classic) to ROS 2 Jazzy (Gazebo Harmonic)
#
# Key change vs original: we no longer use bringup_launch.py / navigation_launch.py
# because Jazzy hardcodes docking_server in the lifecycle list with no flag to skip it.
# Instead we launch localization_launch.py (amcl + map_server) and start each nav2
# navigation node individually so we control exactly which lifecycle nodes are managed.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_autonomous_tb3  = get_package_share_directory('autonomous_tb3')
    pkg_ros_gz_sim      = get_package_share_directory('ros_gz_sim')

    maze_path   = os.path.join(pkg_autonomous_tb3, 'world', 'maze', 'model.sdf')
    config_dir  = os.path.join(pkg_autonomous_tb3, 'config')
    map_file    = os.path.join(config_dir, 'maze.yaml')
    params_file = os.path.join(config_dir, 'tb3_nav_params.yaml')
    rviz_config = os.path.join(config_dir, 'tb3_nav.rviz')

    empty_world = 'empty.sdf'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose       = LaunchConfiguration('x_pose', default='-5.2')
    y_pose       = LaunchConfiguration('y_pose', default='-6.7')

    # ── Gazebo Harmonic: physics server ─────────────────────────────────────
    # CRITICAL: -z 1000 limits physics to 1000 Hz update rate.
    # With max_step_size=0.001s → RTF = 0.001 × 1000 = 1.0 (real-time).
    # Without -z, Gazebo runs at ~5-6x real-time (empty.sdf uses type='ignored'
    # physics which ignores the real_time_factor XML tag). At 5x RTF, sim_time
    # races far ahead of wall time → AMCL TF timestamps become stale → map TF
    # never appears in the global_costmap's buffer → navigation never activates.
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v2 -z 1000 ', empty_world],
            'on_exit_shutdown': 'true',
        }.items()
    )

    # ── Gazebo Harmonic: GUI client ─────────────────────────────────────────
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 '}.items()
    )

    # ── Robot state publisher ────────────────────────────────────────────────
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ── Spawn TurtleBot3 ─────────────────────────────────────────────────────
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()
    )

    # ── Expose custom model directory to Gazebo Harmonic ────────────────────
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_autonomous_tb3, 'models')
    )

    # ── Spawn the maze model ─────────────────────────────────────────────────
    maze_spawner = Node(
        package='autonomous_tb3',
        executable='sdf_spawner',
        name='maze_spawner',
        output='screen',
        arguments=[maze_path, 'maze', '0.0', '0.0']
    )

    # ── Localization stack: direct node launches (no localization_launch.py) ─
    # Launching directly avoids RewrittenYaml substitution issues that caused
    # map_server to load "map.yaml" (relative) instead of the absolute path.
    # yaml_filename is passed explicitly as a node parameter override.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file,
                    {'yaml_filename': map_file,
                     'use_sim_time': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'autostart': True,
                     'node_names': ['map_server', 'amcl'],
                     'use_sim_time': True}],
    )

    # ── Nav2 navigation nodes (individual launches, no docking/route server) ─
    # Jazzy's navigation_launch.py hardcodes docking_server in lifecycle_nodes.
    # We bypass it by launching each node ourselves and registering only the
    # 7 nodes required for TurtleBot3 maze navigation.
    nav_lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'bt_navigator',
        'waypoint_follower',
    ]

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'),
                    ('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'),
                    ('cmd_vel', 'cmd_vel_nav')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'),
                    ('cmd_vel',          'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'cmd_vel')],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart':   True,
            'node_names':  nav_lifecycle_nodes,
            'use_sim_time': True,
        }],
    )

    # ── RViz2 ───────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── Assemble launch description ──────────────────────────────────────────
    ld = LaunchDescription()

    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)

    # t=20s  – spawn robot + maze after Gazebo is ready
    ld.add_action(TimerAction(period=20.0, actions=[spawn_turtlebot_cmd]))
    ld.add_action(TimerAction(period=20.0, actions=[maze_spawner]))

    # t=25s  – start LOCALIZATION only (map_server + amcl)
    # AMCL needs time to spin up and publish the map→odom TF before
    # planner_server/global_costmap tries to look it up.
    ld.add_action(TimerAction(
        period=25.0,
        actions=[map_server, amcl, lifecycle_manager_localization]
    ))

    # t=28s  – start amcl_initializer
    # This node broadcasts map→odom TF at 5 Hz and publishes /initialpose
    # at 1 Hz so the map frame definitely exists when navigation starts.
    # It stops sending /initialpose once /amcl_pose confirms AMCL is alive,
    # but keeps the TF ticking so AMCL's lookups also succeed.
    amcl_init = Node(
        package='autonomous_tb3',
        executable='amcl_initializer',
        name='amcl_initializer',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    ld.add_action(TimerAction(period=28.0, actions=[amcl_init]))

    # t=35s  – start NAVIGATION nodes + RViz
    # 10-second gap (25→35) gives AMCL enough time to become active and
    # amcl_initializer enough time (28→35 = 7 s) to seed the map→odom TF,
    # preventing "Invalid frame ID map" TF errors on startup.
    ld.add_action(TimerAction(
        period=35.0,
        actions=[
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            lifecycle_manager_navigation,
            rviz,
        ]
    ))

    return ld