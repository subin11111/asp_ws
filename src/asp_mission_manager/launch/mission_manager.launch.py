from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('asp_mission_manager')
    params = os.path.join(pkg_dir, 'config', 'mission_manager_params.yaml')

    return LaunchDescription([
        Node(
            package='asp_mission_manager',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen',
            parameters=[params],
        )
    ])
