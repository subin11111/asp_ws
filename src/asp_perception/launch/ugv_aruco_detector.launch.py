from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('asp_perception')
    params = os.path.join(pkg_dir, 'config', 'aruco_detector_params.yaml')

    return LaunchDescription([
        Node(
            package='asp_perception',
            executable='ugv_aruco_detector_node',
            name='ugv_aruco_detector_node',
            output='screen',
            parameters=[params],
        )
    ])
