#!/usr/bin/env python3
"""courier.launch.py — one command for the full courier demo.

Brings up simulation + AMCL + Nav2 + RViz (via acadbot_bringup's
autonomy.launch.py) and the courier node together:

    ros2 launch acadbot_courier courier.launch.py

Arguments:
    headless:=true               Gazebo server only — no GUI, no GPU needed
    rviz:=false                  skip RViz2
    nav2_delay:=<seconds>        wait before starting Nav2 (default 12)
    courier_params_file:=<path>  override the courier node's parameters file
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('acadbot_bringup')
    pkg_courier = get_package_share_directory('acadbot_courier')

    default_params = os.path.join(pkg_courier, 'config', 'courier.yaml')

    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'autonomy.launch.py')),
        launch_arguments={
            'localization': 'amcl',
            'headless': LaunchConfiguration('headless'),
            'rviz': LaunchConfiguration('rviz'),
            'nav2_delay': LaunchConfiguration('nav2_delay'),
        }.items())

    courier = Node(
        package='acadbot_courier',
        executable='courier_node',
        name='courier_node',
        output='screen',
        parameters=[LaunchConfiguration('courier_params_file')],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo server-only (no GUI, no GPU required).'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2. Set false on a machine with no display.'),
        DeclareLaunchArgument(
            'nav2_delay', default_value='12.0',
            description='Seconds to wait for localization before starting Nav2.'),
        DeclareLaunchArgument(
            'courier_params_file', default_value=default_params,
            description="Courier node's parameters file."),
        autonomy,
        courier,
    ])
