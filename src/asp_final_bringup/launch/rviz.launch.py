from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    config = Path(get_package_share_directory("asp_final_bringup")) / "config" / "asp_final_mission.rviz"
    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="asp_final_rviz",
                arguments=["-d", str(config)],
                output="screen",
            ),
        ]
    )
