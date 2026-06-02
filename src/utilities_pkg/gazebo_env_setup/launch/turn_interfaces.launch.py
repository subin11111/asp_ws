# turn_interfaces.launch.py
#
# 기존 turn_interfaces.launch.py에서 bridge_and_tf.launch.py를 포함해서
# pose bridge와 pose_tf_broadcaster가 반드시 같이 실행되게 하는 버전.
#
# 기존 파일에 다른 bridge/node가 있으면 아래 IncludeLaunchDescription 부분만
# 기존 launch에 추가해도 된다.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('gazebo_env_setup')

    bridge_and_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'bridge_and_tf.launch.py')
        )
    )

    return LaunchDescription([
        bridge_and_tf,
    ])
