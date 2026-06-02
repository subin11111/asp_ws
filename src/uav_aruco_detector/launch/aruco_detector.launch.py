from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('uav_aruco_detector')
    params_file = os.path.join(pkg_share, 'config', 'aruco_detector.yaml')

    return LaunchDescription([
        Node(
            package='uav_aruco_detector',
            executable='aruco_detector_node',
            name='aruco_detector',
            output='screen',
            parameters=[params_file],
            remappings=[
                (
                    '/uav/camera/image_raw',
                    '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image'
                ),
                (
                    '/uav/camera/camera_info',
                    '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info'
                ),
            ]
        )
    ])
