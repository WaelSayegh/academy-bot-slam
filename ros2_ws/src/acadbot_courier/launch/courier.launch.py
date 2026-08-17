#!/usr/bin/env python3
"""courier.launch.py — complete AcadBot Courier system bringup.

Starts the existing AcadBot autonomy stack, including simulation,
localization, Nav2, and RViz, then starts the courier mission server
with its YAML configuration.

Typical usage:
    ros2 launch acadbot_courier courier.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # -----------------------------------------------------------------------
    # Resolve installed package directories.
    # -----------------------------------------------------------------------
    bringup_share = get_package_share_directory("acadbot_bringup")
    courier_share = get_package_share_directory("acadbot_courier")

    autonomy_launch = os.path.join(
        bringup_share,
        "launch",
        "autonomy.launch.py"
    )

    courier_config = os.path.join(
        courier_share,
        "config",
        "courier.yaml"
    )

    # -----------------------------------------------------------------------
    # Start the existing full autonomy stack.
    #
    # autonomy.launch.py already brings up:
    #   - Gazebo simulation
    #   - AMCL localization
    #   - Nav2
    #   - RViz
    # -----------------------------------------------------------------------
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(autonomy_launch),
        launch_arguments={
            "localization": "amcl",
        }.items(),
    )

    # -----------------------------------------------------------------------
    # Start the AcadBot courier mission server using its YAML configuration.
    # -----------------------------------------------------------------------
    courier_server = Node(
        package="acadbot_courier",
        executable="courier_server",
        name="courier_server",
        output="screen",
        parameters=[courier_config],
    )

    return LaunchDescription([
        autonomy,
        courier_server,
    ])