#!/usr/bin/env python3
"""navigation.launch.py — bring up Nav2 for AcadBot (Session 4).

Two localization options (pick one with the `localization` argument):
  * slam   (default): slam_toolbox in localization mode provides map->odom
  * amcl            : the classic AMCL + map_server with a saved .yaml map

Typical full-stack startup:
    ros2 launch acadbot_gazebo simulation.launch.py
    ros2 launch acadbot_navigation navigation.launch.py
    ros2 launch acadbot_control patrol.launch.py     # the C++ mission node
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            GroupAction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('acadbot_navigation')
    pkg_slam = get_package_share_directory('acadbot_slam')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    map_yaml = LaunchConfiguration('map')

    default_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg_nav, 'maps', 'academy_map_v2.yaml')

    # --- Localization via slam_toolbox (default) ---
    use_slam = PythonExpression(["'", localization, "' == 'slam'"])
    use_amcl = PythonExpression(["'", localization, "' == 'amcl'"])

    slam_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'localization_slam.launch.py')),
        condition=IfCondition(use_slam),
    )

    # --- Localization via AMCL + map_server ---
    amcl_localization = GroupAction(
        condition=IfCondition(use_amcl),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')),
                launch_arguments={
                    'map': map_yaml,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                }.items(),
            ),
        ],
    )

    # --- The Nav2 navigation stack (planner, controller, behaviors, BT, ...) ---
    # Our own bringup rather than nav2_bringup/navigation_launch.py: that one
    # also starts docking_server and route_server, which this course does not
    # use and does not configure — and a server that fails to configure makes
    # the lifecycle manager abort, leaving every other server INACTIVE.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'nav2_stack.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
    )

    # Nav2's costmaps refuse to configure until map->odom exists, and the
    # lifecycle manager aborts the whole bringup if a transition times out.
    # Localization needs a few seconds first (slam_toolbox has to deserialize
    # the pose graph, which is not instant on a big map or a busy laptop), so
    # hold the stack back rather than racing it. Raise nav2_delay if you still
    # see "Failed to change state for node: controller_server".
    delayed_nav2 = TimerAction(
        period=LaunchConfiguration('nav2_delay'), actions=[nav2])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'localization', default_value='amcl',
            description="'amcl' (particle filter on the .yaml/.pgm grid map, "
                        "the default since Session 3) or 'slam' (slam_toolbox "
                        "localization mode, which needs a serialized "
                        ".posegraph saved alongside the map)"),
        DeclareLaunchArgument(
            'nav2_delay', default_value='12.0',
            description='Seconds to wait for localization before starting Nav2.'),

        slam_localization,
        amcl_localization,
        delayed_nav2,
    ])
