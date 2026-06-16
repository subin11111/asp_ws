from launch import LaunchDescription
from launch_ros.actions import Node


def bridge(topic, ros_type, gz_type, direction="@"):
    return f"{topic}@{ros_type}{direction}{gz_type}"


def generate_launch_description():
    bridge_args = [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/model/X1_asp/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        "/model/X1_asp/pose_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        "/model/x500_gimbal_0/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        "/model/x500_gimbal_0/pose_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        bridge("/model/X1_asp/cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist", "]"),
        bridge(
            "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image",
            "sensor_msgs/msg/Image",
            "gz.msgs.Image",
            "[",
        ),
        bridge(
            "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info",
            "sensor_msgs/msg/CameraInfo",
            "gz.msgs.CameraInfo",
            "[",
        ),
        bridge(
            "/world/default/model/X1_asp/link/base_link/sensor/camera_front/image",
            "sensor_msgs/msg/Image",
            "gz.msgs.Image",
            "[",
        ),
        bridge(
            "/world/default/model/X1_asp/link/base_link/sensor/camera_front/camera_info",
            "sensor_msgs/msg/CameraInfo",
            "gz.msgs.CameraInfo",
            "[",
        ),
        bridge(
            "/world/default/model/X1_asp/link/base_link/sensor/gpu_lidar/scan/points",
            "sensor_msgs/msg/PointCloud2",
            "gz.msgs.PointCloudPacked",
            "[",
        ),
    ]
    return LaunchDescription(
        [
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="asp_final_parameter_bridge",
                arguments=bridge_args,
                output="screen",
            ),
            Node(
                package="asp_final_gazebo_bridge",
                executable="ugv_cmd_vel_relay",
                name="asp_final_ugv_cmd_vel_relay",
                output="screen",
            ),
            Node(
                package="asp_final_gazebo_bridge",
                executable="final_pose_tf_broadcaster",
                name="asp_final_pose_tf_broadcaster",
                parameters=[
                    {
                        "map_frame": "map",
                        "ugv_base_frame": "X1_asp/base_link",
                        "uav_base_frame": "x500_gimbal_0/base_link",
                    }
                ],
                output="screen",
            ),
        ]
    )
