from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('asp_ugv_control')

    params = os.path.join(pkg_dir, 'config', 'path_follower_params.yaml')
    mission_csv = os.path.join(pkg_dir, 'path', 'mission.csv')

    return LaunchDescription([
        Node(
            package='asp_ugv_control',
            executable='ugv_path_follower_node',
            name='ugv_path_follower_node',
            output='screen',
            parameters=[
                params,
                {'mission_csv': mission_csv},
                {'cmd_vel_topic': '/command/ugv_cmd_vel'},
                {'map_frame': 'map'},
                {'base_frame': 'X1_asp/base_link'},
            ],
        )
    ])
