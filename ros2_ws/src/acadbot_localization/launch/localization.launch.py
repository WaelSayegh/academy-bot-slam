import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    acadbot_gazebo_dir = get_package_share_directory('acadbot_gazebo')
    acadbot_navigation_dir = get_package_share_directory('acadbot_navigation')
    acadbot_description_dir = get_package_share_directory('acadbot_description')
    acadbot_localization_dir = get_package_share_directory('acadbot_localization')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(acadbot_navigation_dir, 'maps', 'academy_map.yaml')
    default_params = os.path.join(acadbot_navigation_dir, 'config', 'nav2_params.yaml')
    default_rviz_config = os.path.join(acadbot_description_dir, 'rviz', 'nav2.rviz')
    default_monitor_config = os.path.join(acadbot_localization_dir, 'config', 'localization_monitor.yaml')

    map_arg = DeclareLaunchArgument('map', default_value=default_map)
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(acadbot_gazebo_dir, 'launch', 'simulation.launch.py')),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items())

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
            'params_file': default_params,
        }.items())

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', default_rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen')

    monitor_node = Node(
        package='acadbot_localization', executable='localization_monitor',
        name='localization_monitor', output='screen',
        parameters=[default_monitor_config])

    return LaunchDescription([
        map_arg, headless_arg, rviz_arg,
        simulation, localization, rviz_node, monitor_node,
    ])
