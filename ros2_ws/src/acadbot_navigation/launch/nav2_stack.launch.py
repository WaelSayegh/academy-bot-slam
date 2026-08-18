#!/usr/bin/env python3
"""nav2_stack.launch.py — the Nav2 servers AcadBot actually uses.

This replaces `nav2_bringup/navigation_launch.py`. That file is fine, but its
node list is hard-coded and includes `docking_server` (and `route_server`),
neither of which this course uses and neither of which is configured in our
`nav2_params.yaml`. A server that fails to configure makes
`lifecycle_manager_navigation` abort the whole bringup, which leaves the
planner, controller, behavior server and BT navigator stuck in INACTIVE — and
then every goal comes back "Goal rejected by Nav2".

Starting exactly the servers we teach also means the running stack matches the
Session 4 architecture slide one-for-one:

    bt_navigator -> planner_server -> controller_server -> velocity_smoother
                 -> behavior_server (recoveries)   -> collision_monitor

Arguments:
    params_file:=<path>   Nav2 parameter file (default: our nav2_params.yaml)
    use_sim_time:=true    run on /clock
    autostart:=true       configure + activate everything on startup
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

# Order matters: the lifecycle manager brings these up in sequence.
SERVERS = [
    ('nav2_controller', 'controller_server', 'controller_server', True),
    ('nav2_smoother', 'smoother_server', 'smoother_server', False),
    ('nav2_planner', 'planner_server', 'planner_server', False),
    ('nav2_behaviors', 'behavior_server', 'behavior_server', True),
    ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', False),
    ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', False),
    ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', True),
    ('nav2_collision_monitor', 'collision_monitor', 'collision_monitor', False),
]


def generate_launch_description():
    pkg_nav = get_package_share_directory('acadbot_navigation')
    default_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    # Nav2 nodes must use the relative tf topics or they miss the tree.
    tf_remap = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    nodes = []
    for pkg, exe, name, nav_cmd_vel in SERVERS:
        remaps = list(tf_remap)
        if nav_cmd_vel:
            # Everything upstream of the collision monitor writes cmd_vel_nav;
            # the monitor is the only node that publishes the real /cmd_vel.
            remaps.append(('cmd_vel', 'cmd_vel_nav'))
        nodes.append(Node(
            package=pkg, executable=exe, name=name, output='screen',
            parameters=[params_file],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remaps,
        ))

    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'autostart': autostart,
                     # courier_manager is an rclcpp_lifecycle::LifecycleNode too, but it's
                     # launched separately (as a plain Node(...)) by courier.launch.py, not
                     # started from SERVERS here — only its name needs to be known to this
                     # lifecycle manager so it gets configured/activated in sequence.
                     # Deliberate coupling (TASKS.md Phase 1): launches that include this
                     # file without also running courier_manager (e.g. the S4 patrol demo)
                     # will now hang waiting for a lifecycle service that never appears.
                     'node_names': [s[2] for s in SERVERS] + ['courier_manager']}],
    ))

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        GroupAction([SetParameter('use_sim_time', use_sim_time)] + nodes),
    ])
