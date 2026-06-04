from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('asp_ugv_control')
    params = os.path.join(pkg_dir, 'config', 'rendezvous_params.yaml')
    path_csv = os.path.join(pkg_dir, 'path', 'mission3_rendezvous_senior.csv')

    return LaunchDescription([
        Node(
            package='asp_ugv_control',
            executable='ugv_rendezvous_node',
            name='ugv_rendezvous_node',
            output='screen',
            parameters=[
                params,
                {'path_csv': path_csv},
            ],
        )
    ])
