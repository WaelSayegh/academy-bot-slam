#!/usr/bin/env python3
"""Bring up simulation, AMCL, Nav2, and the Phase 1 courier system."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = get_package_share_directory('acadbot_courier')
    default_params = f'{package_share}/config/courier.yaml'
    default_locations = f'{package_share}/config/locations.yaml'

    localization = LaunchConfiguration('localization')
    nav2_delay = LaunchConfiguration('nav2_delay')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    courier_params_file = LaunchConfiguration('courier_params_file')
    locations_file = LaunchConfiguration('locations_file')

    # Verified against the repository's real autonomy.launch.py on 2026-08-16.
    # Re-check these arguments after pulling future acadbot_bringup changes.
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('acadbot_bringup'),
                'launch',
                'autonomy.launch.py',
            ])
        ),
        launch_arguments={
            'localization': localization,
            'nav2_delay': nav2_delay,
            'headless': headless,
            'rviz': rviz,
            # Map reception (0.6, 4.2) equals Gazebo (-2.4, 2.2).
            'x': spawn_x,
            'y': spawn_y,
            'yaw': spawn_yaw,
        }.items(),
    )

    initial_pose = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='acadbot_courier',
                executable='initial_pose_publisher',
                name='initial_pose_publisher',
                output='screen',
                parameters=[locations_file, {'use_sim_time': True}],
            )
        ],
    )

    # Nav2 itself is delayed by 12 seconds in the verified bringup, so starting
    # the courier at 15 seconds avoids racing its action and costmap services.
    courier = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='acadbot_courier',
                executable='courier_node',
                name='courier_node',
                output='screen',
                parameters=[locations_file, courier_params_file, {'use_sim_time': True}],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('localization', default_value='amcl'),
        DeclareLaunchArgument('nav2_delay', default_value='12.0'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('spawn_x', default_value='-2.4'),
        DeclareLaunchArgument('spawn_y', default_value='2.2'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('courier_params_file', default_value=default_params),
        DeclareLaunchArgument('locations_file', default_value=default_locations),
        autonomy,
        initial_pose,
        courier,
    ])
