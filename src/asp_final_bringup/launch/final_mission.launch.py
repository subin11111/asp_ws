from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
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
    detected_marker_csv_path = "/home/sunny/asp_ws/docs/asp_final_detected_markers.csv"
    rendezvous_goal_x = LaunchConfiguration("rendezvous_goal_x")
    rendezvous_goal_y = LaunchConfiguration("rendezvous_goal_y")
    rendezvous_goal_target_speed = LaunchConfiguration("rendezvous_goal_target_speed")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rendezvous_goal_x", default_value=".nan"),
            DeclareLaunchArgument("rendezvous_goal_y", default_value=".nan"),
            DeclareLaunchArgument("rendezvous_goal_target_speed", default_value=".nan"),
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
                parameters=[
                    str(ugv_params),
                    {
                        "rendezvous_goal_x": rendezvous_goal_x,
                        "rendezvous_goal_y": rendezvous_goal_y,
                        "rendezvous_goal_target_speed": rendezvous_goal_target_speed,
                    },
                ],
                output="screen",
            ),
            Node(
                package="asp_final_uav",
                executable="uav_mission_node",
                name="asp_final_uav_mission_node",
                parameters=[
                    str(uav_params),
                    {
                        "rendezvous_goal_x": rendezvous_goal_x,
                        "rendezvous_goal_y": rendezvous_goal_y,
                    },
                ],
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
                parameters=[{"csv_path": detected_marker_csv_path}],
                output="screen",
            ),
            Node(
                package="asp_final_bringup",
                executable="final_visualization",
                name="asp_final_visualization",
                output="screen",
            ),
            Node(
                package="gazebo_env_setup",
                executable="mission_timer_node",
                name="mission_timer_node",
                output="screen",
            ),
        ]
    )
