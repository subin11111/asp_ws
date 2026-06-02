from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ugv_cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ugv_cmd_vel_bridge',
        output='screen',
        arguments=[
            '/model/X1_asp/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        remappings=[
            ('/model/X1_asp/cmd_vel', '/command/ugv_cmd_vel'),
        ],
    )

    return LaunchDescription([
        ugv_cmd_vel_bridge,
    ])
