from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('asp_uav_control')
    params = os.path.join(pkg_dir, 'config', 'uav_exploration_params.yaml')
    path_csv = os.path.join(pkg_dir, 'path', 'uav_path_mission2_senior.csv')

    return LaunchDescription([
        Node(
            package='asp_uav_control',
            executable='uav_exploration_node',
            name='uav_exploration_node',
            output='screen',
            parameters=[
                params,
                {'path_csv': path_csv},
                {'dynamic_safe_prefix': True},
                {'force_takeoff_before_path': True},
                {'use_mission2_latched_origin': True},
                {'require_mission2_latched_origin': True},
                {'start_on_launch': False},
            ],
        )
    ])
