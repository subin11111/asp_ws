from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('asp_uav_control')
    params = os.path.join(pkg_dir, 'config', 'uav_exploration_params.yaml')
    path_csv = os.path.join(pkg_dir, 'path', 'uav_path_generated.csv')

    return LaunchDescription([
        Node(
            package='asp_uav_control',
            executable='uav_exploration_node',
            name='uav_exploration_node',
            output='screen',
            parameters=[
                params,
                {'path_csv': path_csv},
                {'allow_generated_path_runtime': True},
                {'runtime_path_must_contain': ''},
            ],
        )
    ])
