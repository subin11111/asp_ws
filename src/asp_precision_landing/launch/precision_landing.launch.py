from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('asp_precision_landing')
    params = os.path.join(pkg_dir, 'config', 'precision_landing_params.yaml')

    return LaunchDescription([
        Node(
            package='asp_precision_landing',
            executable='precision_landing_node',
            name='precision_landing_node',
            output='screen',
            parameters=[params],
        )
    ])
