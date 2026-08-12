#!/usr/bin/env python3
"""Launch simulation, saved-map localization, RViz, and the AMCL monitor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory("acadbot_gazebo")
    pkg_navigation = get_package_share_directory("acadbot_navigation")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")
    pkg_description = get_package_share_directory("acadbot_description")
    pkg_localization = get_package_share_directory("acadbot_localization")

    default_map = os.path.join(
        pkg_navigation, "maps", "academy_map.yaml"
    )
    params_file = os.path.join(
        pkg_navigation, "config", "nav2_params.yaml"
    )
    rviz_config = os.path.join(
        pkg_description, "rviz", "nav2.rviz"
    )
    monitor_config = os.path.join(
        pkg_localization, "config", "localization_monitor.yaml"
    )

    map_yaml = LaunchConfiguration("map")
    headless = LaunchConfiguration("headless")
    rviz_enabled = LaunchConfiguration("rviz")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_gazebo, "launch", "simulation.launch.py"
            )
        ),
        launch_arguments={
            "headless": headless,
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_nav2_bringup,
                "launch",
                "localization_launch.py",
            )
        ),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": "true",
            "params_file": params_file,
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(rviz_enabled),
    )

    monitor = Node(
        package="acadbot_localization",
        executable="localization_monitor",
        name="localization_monitor",
        parameters=[monitor_config],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value=default_map,
            description="Occupancy-grid .yaml to localise against.",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo without its graphical interface.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz2.",
        ),
        simulation,
        localization,
        rviz,
        monitor,
    ])
