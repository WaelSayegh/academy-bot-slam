"""courier.launch.py - one command for the full AcadBot Courier demo.

    ros2 launch acadbot_courier courier.launch.py
    ros2 launch acadbot_courier courier.launch.py headless:=true     # no Gazebo GUI
    ros2 launch acadbot_courier courier.launch.py rviz:=false       # skip RViz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('acadbot_bringup')
    pkg_courier = get_package_share_directory('acadbot_courier')

    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'autonomy.launch.py')
        ),
        launch_arguments={
            'localization': LaunchConfiguration('localization'),
            'nav2_delay': LaunchConfiguration('nav2_delay'),
            'headless': LaunchConfiguration('headless'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    courier = Node(
        package='acadbot_courier',
        executable='courier_server',
        name='courier_server',
        output='screen',
        parameters=[os.path.join(pkg_courier, 'config', 'courier.yaml')],
    )

    initialpose_msg = (
        "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0},"
        "orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,"
        "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.06853891945200942]}}"
    )

    auto_initialpose = TimerAction(
        period=LaunchConfiguration('initialpose_delay'),
        condition=IfCondition(PythonExpression([LaunchConfiguration('initialpose_delay'), " != 0.0"])),
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/initialpose',
                 'geometry_msgs/msg/PoseWithCovarianceStamped',
                 initialpose_msg],
            output='log'
        )]
    )

    return LaunchDescription([
        DeclareLaunchArgument('localization', default_value='amcl'),
        DeclareLaunchArgument('nav2_delay', default_value='12.0'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('initialpose_delay', default_value='15.0'),
        autonomy,
        courier,
        auto_initialpose,
    ])