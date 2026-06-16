from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        '/model/X1_asp/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/model/X1_asp/pose_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/model/x500_gimbal_0/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/model/x500_gimbal_0/pose_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image'
        '@sensor_msgs/msg/Image[gz.msgs.Image',
        '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info'
        '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/world/default/model/X1_asp/link/base_link/sensor/camera_front/image'
        '@sensor_msgs/msg/Image[gz.msgs.Image',
        '/world/default/model/X1_asp/link/base_link/sensor/camera_front/camera_info'
        '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/world/default/model/X1_asp/link/base_link/sensor/gpu_lidar/scan/points'
        '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        '/world/default/model/x500_gimbal_0/link/base_link/sensor/imu_sensor/imu'
        '@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera_imu/imu'
        '@sensor_msgs/msg/Imu[gz.msgs.IMU',
    ]

    parameter_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        output='screen',
        arguments=bridge_args,
        parameters=[{'use_sim_time': True}],
    )

    pose_tf_broadcaster = Node(
        package='gazebo_env_setup',
        executable='pose_tf_broadcaster',
        name='pose_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # world -> orb_map (place the model in the world frame)
    tf_world_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_base',
        output='screen',
        arguments=[
            '0', '0', '0',   # x y z
            '-1.57079632679', '0', '-1.57079632679',   # yaw pitch roll
            'world',
            'map',
        ],
    )

    return LaunchDescription([
        parameter_bridge_node,
        pose_tf_broadcaster,
        tf_world_base,
    ])
