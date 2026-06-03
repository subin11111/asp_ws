from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    mission_pkg = get_package_share_directory('asp_mission_manager')
    uav_pkg = get_package_share_directory('asp_uav_control')
    perception_pkg = get_package_share_directory('asp_perception')
    ugv_pkg = get_package_share_directory('asp_ugv_control')
    landing_pkg = get_package_share_directory('asp_precision_landing')

    mission_params = os.path.join(mission_pkg, 'config', 'mission_manager_params.yaml')
    uav_params = os.path.join(uav_pkg, 'config', 'uav_exploration_params.yaml')
    uav_path = os.path.join(uav_pkg, 'path', 'uav_path_mission2_senior.csv')
    perception_params = os.path.join(perception_pkg, 'config', 'uav_aruco_detector_params.yaml')
    rendezvous_params = os.path.join(ugv_pkg, 'config', 'rendezvous_params.yaml')
    rendezvous_path = os.path.join(ugv_pkg, 'path', 'rendezvous.csv')
    landing_params = os.path.join(landing_pkg, 'config', 'precision_landing_params.yaml')

    return LaunchDescription([
        Node(
            package='asp_mission_manager',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen',
            parameters=[mission_params],
        ),
        Node(
            package='asp_perception',
            executable='uav_aruco_detector_node',
            name='uav_aruco_detector_node',
            output='screen',
            parameters=[perception_params],
        ),
        Node(
            package='asp_uav_control',
            executable='uav_exploration_node',
            name='uav_exploration_node',
            output='screen',
            parameters=[
                uav_params,
                {'path_csv': uav_path},
                {'dynamic_safe_prefix': True},
                {'force_takeoff_before_path': True},
                {'use_mission2_latched_origin': True},
                {'require_mission2_latched_origin': True},
                {'start_on_launch': False},
            ],
        ),
        Node(
            package='asp_ugv_control',
            executable='ugv_rendezvous_node',
            name='ugv_rendezvous_node',
            output='screen',
            parameters=[
                rendezvous_params,
                {'path_csv': rendezvous_path},
            ],
        ),
        Node(
            package='asp_precision_landing',
            executable='precision_landing_node',
            name='precision_landing_node',
            output='screen',
            parameters=[landing_params],
        ),
    ])
