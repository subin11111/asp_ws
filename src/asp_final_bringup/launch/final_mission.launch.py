from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from pathlib import Path


def share(package):
    return Path(get_package_share_directory(package))


def generate_launch_description():
    ugv_params = share("asp_final_ugv") / "config" / "ugv_params.yaml"
    uav_params = share("asp_final_uav") / "config" / "uav_params.yaml"
    perception_params = share("asp_final_perception") / "config" / "perception_params.yaml"
    gazebo_bridge_launch = share("asp_final_gazebo_bridge") / "launch" / "gazebo_bridge.launch.py"

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
                name="asp_final_micro_xrce_agent",
                output="screen",
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo_bridge_launch))),
            Node(
                package="asp_final_px4_bridge",
                executable="px4_offboard_bridge",
                name="asp_final_px4_offboard_bridge",
                output="screen",
            ),
            Node(
                package="asp_final_mission",
                executable="mission_supervisor",
                name="asp_final_mission_supervisor",
                output="screen",
            ),
            Node(
                package="asp_final_ugv",
                executable="ugv_path_follower",
                name="asp_final_ugv_path_follower",
                parameters=[str(ugv_params)],
                output="screen",
            ),
            Node(
                package="asp_final_uav",
                executable="uav_mission_node",
                name="asp_final_uav_mission_node",
                parameters=[str(uav_params)],
                output="screen",
            ),
            Node(
                package="asp_final_perception",
                executable="aruco_detector",
                name="asp_final_uav_aruco_detector",
                parameters=[str(perception_params), {"mode": "uav"}],
                output="screen",
            ),
            Node(
                package="asp_final_perception",
                executable="aruco_detector",
                name="asp_final_landing_aruco_detector",
                parameters=[str(perception_params), {"mode": "landing"}],
                output="screen",
            ),
            Node(
                package="asp_final_perception",
                executable="detected_marker_csv",
                name="asp_final_detected_marker_csv",
                output="screen",
            ),
        ]
    )
