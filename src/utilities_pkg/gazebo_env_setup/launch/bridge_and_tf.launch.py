from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        '/rgb_camera@sensor_msgs/msg/Image@gz.msgs.Image',
        '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        '/drone_0_pcl_render_node/depth@sensor_msgs/msg/Image@gz.msgs.Image',
        '/drone_0_pcl_render_node/depth/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        '/world/default/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'
        '@sensor_msgs/msg/Imu@gz.msgs.IMU',
    ]

    parameter_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        output='screen',
        arguments=bridge_args,
        parameters=[{'use_sim_time': True}],
    )
    

    # IMU: base_link와 동일 위치로 가정 (offset 0)
    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_imu',
        output='screen',
        arguments=[
            '0', '0', '0',          # x y z
            '0', '0', '0',          # roll pitch yaw
            'base_link',            # parent
            'x500_depth_0/base_link/imu_sensor',  # child
        ],
    )

    # base_link -> OakD-Lite camera_link (SDF에서 .12 .03 .242로 설정)
    tf_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_link',
        output='screen',
        arguments=[
            '0.12', '0.03', '0.242',   # x y z (camera_link relative to base_link) (from SDF)
            '0', '0', '0',             # roll pitch yaw (from SDF)
            'base_link',
            'x500_depth_0/camera_link',
        ],
    )

    # Depth 카메라 (StereoOV7251) – SDF에서 sensor pose is relative to camera_link
    tf_depth = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_depth',
        output='screen',
        arguments=[
            '0.01233', '-0.03', '0.01878',   # x y z (sensor relative to camera_link) (from SDF)
            '0', '0', '0',                     # roll pitch yaw (from SDF)
            'x500_depth_0/camera_link',
            'x500_depth_0/camera_link/StereoOV7251',
        ],
    )

    # RGB 카메라 (IMX214) – SDF sensor pose relative to camera_link
    tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_rgb',
        output='screen',
        arguments=[
            '0.01233', '-0.03', '0.01878',  # x y z (sensor relative to camera_link) (from SDF)
            '0', '0', '0',                   # roll pitch yaw (from SDF)
            'x500_depth_0/camera_link',
            'x500_depth_0/camera_link/IMX214',
        ],
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
        tf_imu,
        tf_camera_link,
        tf_camera,
        tf_depth,
        tf_world_base,
    ])
