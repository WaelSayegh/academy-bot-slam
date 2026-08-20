"""
courier.launch.py — requirement 9: one command brings up everything needed
to demo the courier: simulation + AMCL localization + Nav2 + the dispatcher
node. Reuses acadbot_bringup's existing autonomy.launch.py rather than
re-declaring sim/Nav2/localization from scratch.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('acadbot_bringup')
    courier_share = get_package_share_directory('acadbot_courier')

    # Reuse the existing `autonomy.launch.py` from `acadbot_bringup` which
    # brings up simulation, Nav2, AMCL (or SLAM depending on args), and all
    # required infrastructure. We forward the `localization=amcl` argument to
    # request AMCL-based localization here (map_server + amcl lifecycle nodes).
    autonomy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'autonomy.launch.py')
        ),
        launch_arguments={'localization': 'amcl'}.items(),
    )

    # Start the dispatcher node which reads `courier_params.yaml` for
    # location definitions and retry/timeouts. Parameters are passed by
    # filename (rclcpp will load the YAML into the node's parameter server).
    # The Node's lifetime is managed by the launch system; when the launch
    # exits the Node is torn down cleanly (RAII semantics via launch).
    dispatcher_node = Node(
        package='acadbot_courier',
        executable='dispatcher',
        name='dispatcher',
        output='screen',
        parameters=[
            os.path.join(courier_share, 'config', 'courier_params.yaml')
        ],
    )

    return LaunchDescription([
        autonomy_launch,
        dispatcher_node,
    ])