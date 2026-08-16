#!/usr/bin/env python3
"""One command for: simulation + Nav2 + AMCL
localization on the academy map + the courier_manager node. Run with
`ros2 launch acadbot_bringup courier.launch.py`."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('acadbot_bringup')
    pkg_courier = get_package_share_directory('acadbot_courier_pipeline')

    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'autonomy.launch.py')),
        launch_arguments={
            'localization': LaunchConfiguration('localization'),
            'nav2_delay': LaunchConfiguration('nav2_delay'),
            'headless': LaunchConfiguration('headless'),
            'rviz': LaunchConfiguration('rviz'),
        }.items())

    courier = Node(
        package='acadbot_courier_pipeline',
        executable='courier_manager',
        name='courier_manager',
        output='screen',
        parameters=[os.path.join(pkg_courier, 'config', 'courier.yaml')],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'localization', default_value='amcl',
            description='slam or amcl. Courier demo runs on the saved academy map.'),
        DeclareLaunchArgument('nav2_delay', default_value='12.0'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo server-only (no GUI, no GPU required).'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2. Set false on a machine with no display.'),
        autonomy,
        courier,
    ])
