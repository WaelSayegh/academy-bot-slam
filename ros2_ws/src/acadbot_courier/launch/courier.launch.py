#!/usr/bin/env python3
"""One command for the whole courier demo: simulation, localization, the pose
seeder, Nav2, RViz and the courier itself.

    ros2 launch acadbot_courier courier.launch.py

Arguments:
    mission:=fsm|bt   which mission engine runs (default fsm)
    headless:=true    Gazebo server only -- no GUI, no GPU needed
    rviz:=false       skip RViz2
    map:=<path>       localize on a different saved map
    initial_x/initial_y/initial_yaw   where the robot starts, in the map frame
    nav2_delay:=<s>   seconds to wait for localization before starting Nav2
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_courier = get_package_share_directory('acadbot_courier')
    pkg_gazebo = get_package_share_directory('acadbot_gazebo')
    pkg_nav = get_package_share_directory('acadbot_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    courier_params = os.path.join(pkg_courier, 'config', 'courier.yaml')
    rviz_config = os.path.join(pkg_courier, 'rviz', 'courier.rviz')
    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg_nav, 'maps', 'academy_map.yaml')

    headless = LaunchConfiguration('headless')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'simulation.launch.py')),
        launch_arguments={'headless': headless}.items(),
    )

    # Started with the simulation, not after it: AMCL's TF buffer only fills
    # from the moment the node starts.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
            'params_file': nav2_params,
        }.items(),
    )

    # A node rather than a timed `topic pub`, because AMCL silently ignores a
    # pose that arrives before it is active and the seeder can check and retry.
    seeder = Node(
        package='acadbot_courier',
        executable='initial_pose_seeder',
        name='initial_pose_seeder',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_yaw': LaunchConfiguration('initial_yaw'),
        }],
    )

    # The course's own Nav2 bringup and parameters, unchanged.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'nav2_stack.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params,
        }.items(),
    )
    delayed_nav2 = TimerAction(
        period=LaunchConfiguration('nav2_delay'), actions=[nav2])

    # Exactly one of the two engines runs; both claim the same service and
    # action names.
    mission = LaunchConfiguration('mission')
    use_fsm = PythonExpression(["'", mission, "' == 'fsm'"])
    use_bt = PythonExpression(["'", mission, "' == 'bt'"])

    courier_fsm = Node(
        package='acadbot_courier',
        executable='courier_server',
        name='courier_server',
        output='screen',
        parameters=[courier_params],
        condition=IfCondition(use_fsm),
    )

    courier_bt = Node(
        package='acadbot_courier',
        executable='courier_bt_server',
        name='courier_bt_server',
        output='screen',
        parameters=[courier_params],
        remappings=[('~/locations', '/courier_server/locations')],
        condition=IfCondition(use_bt),
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('mission', default_value='fsm',
                              description="Mission engine: 'fsm' (State Machine) or 'bt' (Behavior Tree)."),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo server-only (no GUI, no GPU required).'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2. Set false on a machine with no display.'),
        DeclareLaunchArgument(
            'initial_x', default_value='0.0',
            description="AMCL's starting guess, in the map frame. The default "
                        'is where the robot spawns: map = gazebo + (3, 2), and '
                        'simulation.launch.py spawns it at gazebo (-3, -2).'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'nav2_delay', default_value='15.0',
            description='Seconds to wait for localization before starting Nav2.'),
        sim,
        localization,
        seeder,
        delayed_nav2,
        courier_fsm,
        courier_bt,
        rviz,
    ])
