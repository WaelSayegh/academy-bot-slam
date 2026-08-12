#!/usr/bin/env python3
"""localization.launch.py — standalone AMCL localization for AcadBot.

Brings up the simulation, map_server + AMCL (via nav2_bringup) on a
previously saved map, RViz, and the covariance-based localization_monitor
node that reports x/y/yaw and whether AMCL has converged. Does NOT start the
rest of the Nav2 stack (planner/controller/behaviors) - see
acadbot_navigation/navigation.launch.py for that.

Typical usage:
    ros2 launch acadbot_localization localization.launch.py
    # In RViz: "2D Pose Estimate" to seed AMCL, then watch the
    # localization_monitor log lines converge (sigma drops below
    # converged_sigma).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_localization = get_package_share_directory('acadbot_localization')
    pkg_navigation = get_package_share_directory('acadbot_navigation')
    pkg_description = get_package_share_directory('acadbot_description')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_gazebo = get_package_share_directory('acadbot_gazebo')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    monitor_params_file = LaunchConfiguration('monitor_params_file')
    use_rviz = LaunchConfiguration('rviz')
    headless = LaunchConfiguration('headless')

    default_map = os.path.join(pkg_navigation, 'maps', 'academy_map.yaml')
    default_params = os.path.join(pkg_navigation, 'config', 'nav2_params.yaml')
    default_monitor_params = os.path.join(
        pkg_localization, 'config', 'localization_monitor.yaml')
    default_rviz_config = os.path.join(pkg_description, 'rviz', 'nav2.rviz')

    # The simulated robot - same pattern as acadbot_bringup/mapping.launch.py's
    # 'sim' block. Starts first so /clock exists before AMCL needs it.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'headless': headless,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # map_server + AMCL + their lifecycle manager - the same block
    # acadbot_navigation/navigation.launch.py includes for localization:=amcl.
    amcl_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
    )

    localization_monitor = Node(
        package='acadbot_localization',
        executable='localization_monitor',
        name='localization_monitor',
        output='screen',
        parameters=[monitor_params_file, {'use_sim_time': use_sim_time}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Full path to the map yaml saved by map_saver_cli.'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Params file containing the amcl: block.'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'monitor_params_file', default_value=default_monitor_params,
            description='Params for the localization_monitor node.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz with the nav2 config to set the initial pose.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo server-only (no GUI, no GPU needed).'),

        sim,
        amcl_localization,
        localization_monitor,
        rviz,
    ])
