# bridge_and_tf.launch.py
#
# Gazebo pose topics bridge + pose_tf_broadcaster 실행
#
# 추가된 pose bridge:
#   /model/X1_asp/pose
#   /model/X1_asp/pose_static
#   /model/x500_gimbal_0/pose
#   /model/x500_gimbal_0/pose_static
#
# static TF publisher for old model name x500_depth_0 intentionally removed.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_pose_bridge',
        output='screen',
        arguments=[
            '/model/X1_asp/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/model/X1_asp/pose_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/model/x500_gimbal_0/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/model/x500_gimbal_0/pose_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
    )

    pose_tf_broadcaster = Node(
        package='gazebo_env_setup',
        executable='pose_tf_broadcaster',
        name='pose_tf_broadcaster',
        output='screen',
    )

    return LaunchDescription([
        pose_bridge,
        pose_tf_broadcaster,
    ])
