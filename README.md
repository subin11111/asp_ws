# ASP ROS2 Workspace

## 목적

이 문서는 `jsb` 브랜치 기준으로 ASP ROS2 workspace에서 변경한 제어 구조와 실행 방법을 정리한다. 원래 브랜치/원본 코드 대비 UAV와 UGV 명령 토픽이 섞일 수 있는 문제를 분리하고, UGV 전용 keyboard control과 Gazebo bridge를 추가한 내용을 설명한다.

## 프로젝트 개요

이 저장소는 ASP 자율주행 시스템 플랫폼 실험을 위한 ROS2 workspace이다. PX4 SITL + Gazebo 환경과 연동하여 UAV/UGV 제어 실험을 수행한다.

기존 제공 패키지를 기반으로 UGV 전용 keyboard control과 bridge를 추가했다. 최종 목표는 UAV 제어 명령과 UGV 제어 명령을 서로 다른 ROS2 topic으로 분리하여 두 차량을 독립적으로 운용하는 것이다.

## 기존 문제 구조

기존 `keyboard_control_node`는 `/command/twist`를 publish했고, 이 토픽은 UAV offboard control에 사용되었다.

```text
keyboard_control_node
  -> /command/twist
  -> offboard_control
  -> PX4 UAV control
```

UGV를 임시로 움직이기 위해 다음 bridge를 사용했을 때 문제가 발생했다.

```bash
ros2 run ros_gz_bridge parameter_bridge \
  "/model/X1_asp/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist" \
  --ros-args -r /model/X1_asp/cmd_vel:=/command/twist
```

이 구조에서는 UAV와 UGV가 같은 `/command/twist`를 공유한다.

```text
/command/twist
  +- UAV offboard_control
  +- UGV Gazebo DiffDrive
```

따라서 UGV keyboard 입력이 UAV offboard control로 전달될 수 있고, UAV 명령과 UGV 명령이 섞일 위험이 있다.

## 최종 수정 구조

UAV와 UGV 명령 토픽을 다음과 같이 분리했다.

```text
[UAV Control]
keyboard_control_node
  -> /command/twist
  -> offboard_control
  -> /fmu/in/trajectory_setpoint
  -> x500_gimbal_0

[UGV Control]
ugv_keyboard_control_node
  -> /command/ugv_cmd_vel
  -> ros_gz_bridge
  -> /model/X1_asp/cmd_vel
  -> X1_asp
```

정리하면 다음과 같다.

```text
/command/twist       = UAV 전용
/command/ugv_cmd_vel = UGV 전용
```

## 변경 사항

| File | Type | Description |
| --- | --- | --- |
| `utilities_pkg/src/ugv_keyboard_control_node.cpp` | Added | UGV 전용 keyboard control node |
| `utilities_pkg/CMakeLists.txt` | Modified | `ugv_keyboard_control_node` executable 및 install target 추가 |
| `gazebo_env_setup/launch/ugv_bridge.launch.py` | Added | `/command/ugv_cmd_vel` -> `/model/X1_asp/cmd_vel` bridge |
| `utilities_pkg/package.xml` | Modified | `std_msgs` dependency 반영 |
| `gazebo_env_setup/package.xml` | Modified | `std_msgs`, `tf2_msgs` dependency 반영 |
| `gazebo_env_setup/CMakeLists.txt` | Modified | `std_msgs`, `tf2_msgs` `find_package` 및 target dependency 반영 |

## UGV Keyboard Node

* Node name: `ugv_keyboard_control_node`
* Publish topic: `/command/ugv_cmd_vel`
* Message type: `geometry_msgs/msg/Twist`
* Publish rate: 20 Hz
* Control fields: `linear.x`, `angular.z`
* Differential drive 차량이므로 `linear.y`는 사용하지 않음

키 매핑은 다음과 같다.

```text
w : forward
x : backward
a : turn left
d : turn right
s or Space : stop
h : help
Ctrl+C : quit
```

## UGV Bridge

* Launch file: `gazebo_env_setup/launch/ugv_bridge.launch.py`
* ROS2 topic: `/command/ugv_cmd_vel`
* Gazebo topic: `/model/X1_asp/cmd_vel`
* Bridge direction: ROS2 -> Gazebo
* Bridge expression:

```text
/model/X1_asp/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist
```

## 빌드 방법

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

colcon build --packages-select utilities_pkg gazebo_env_setup

source install/setup.bash
```

성공 기준은 다음과 같다.

```text
Summary: 2 packages finished
```

## 실행 방법

### UGV 실험

Terminal 1: PX4/Gazebo 실행

```bash
cd ~/PX4-Autopilot_ASP
make px4_sitl gz_x500_gimbal
```

Terminal 2: UGV bridge 실행

```bash
px4humble
source ~/ros2_ws/install/setup.bash
ros2 launch gazebo_env_setup ugv_bridge.launch.py
```

Terminal 3: UGV keyboard 실행

```bash
px4humble
source ~/ros2_ws/install/setup.bash
ros2 run utilities_pkg ugv_keyboard_control_node
```

Terminal 4: UGV command topic 확인

```bash
px4humble
source ~/ros2_ws/install/setup.bash
ros2 topic echo /command/ugv_cmd_vel
```

### UAV 실험

Terminal 1: PX4/Gazebo 실행

```bash
cd ~/PX4-Autopilot_ASP
make px4_sitl gz_x500_gimbal
```

Terminal 2: QGroundControl 실행

```bash
chmod +x QGroundControl.AppImage
./QGroundControl.AppImage
```

Terminal 3: UAV interface 실행

```bash
px4humble
source ~/ros2_ws/install/setup.bash
ros2 launch gazebo_env_setup turn_interfaces.launch.py
```

Terminal 4: UAV keyboard 실행

```bash
px4humble
source ~/ros2_ws/install/setup.bash
ros2 run utilities_pkg keyboard_control_node
```

## 검증 방법

UGV topic을 확인한다.

```bash
ros2 topic list | grep ugv
```

UAV command topic을 확인한다.

```bash
ros2 topic list | grep /command/twist
```

UGV keyboard만 눌렀을 때 다음 topic에는 값이 나와야 한다.

```bash
ros2 topic echo /command/ugv_cmd_vel
```

반대로 다음 topic에는 값이 나오지 않아야 한다.

```bash
ros2 topic echo /command/twist
```

이 결과가 확인되면 UGV keyboard control이 UAV용 `/command/twist`와 분리되어 동작하는 것이다.

## 사용 금지 명령

다음 임시 bridge 명령은 더 이상 사용하지 않는다.

```bash
ros2 run ros_gz_bridge parameter_bridge \
  "/model/X1_asp/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist" \
  --ros-args -r /model/X1_asp/cmd_vel:=/command/twist
```

이 명령은 `/command/twist`가 UAV와 UGV에 동시에 연결되어 명령이 섞일 수 있으므로 사용하지 않는다.

## 변경 기록: RViz Camera Bridge 수정

작성 시각: `2026-06-02 21:43:09 KST`

RViz에서 `UAV_Image`, `UGV_Image`, `Marker Detected` 패널에 `No Image`가 표시되는 문제를 확인했다. 핵심 원인은 현재 Gazebo 모델 topic은 `X1_asp`, `x500_gimbal_0` 기준으로 생성되어 있는데, bridge 설정에는 예전 camera topic이 남아 있어 image topic이 ROS2로 전달되지 않는 점이었다.

이번 수정에서는 `gazebo_env_setup/launch/bridge_and_tf.launch.py`의 `bridge_args`를 현재 실제 Gazebo topic 기준으로 갱신했다.

```text
/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image
/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info
/world/default/model/X1_asp/link/base_link/sensor/camera_front/image
/world/default/model/X1_asp/link/base_link/sensor/camera_front/camera_info
/world/default/model/X1_asp/link/base_link/sensor/gpu_lidar/scan/points
/world/default/model/x500_gimbal_0/link/base_link/sensor/imu_sensor/imu
/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera_imu/imu
```

또한 `gazebo_env_setup/config/asp_final_proj.rviz`에서 RViz display QoS를 Gazebo bridge topic과 맞추기 위해 다음 display의 Reliability Policy를 `Best Effort`로 변경했다.

```text
UAV_Image
UGV_Image
GPU_LiDAR
```

RViz display topic은 다음 값을 사용한다.

```text
UAV_Image: /world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image
UGV_Image: /world/default/model/X1_asp/link/base_link/sensor/camera_front/image
GPU_LiDAR: /world/default/model/X1_asp/link/base_link/sensor/gpu_lidar/scan/points
```

빌드는 다음 명령으로 확인했다.

```bash
colcon build --packages-select gazebo_env_setup
```

제어 코드, keyboard node, UGV bridge, PX4 파일은 수정하지 않았다.

## 변경 기록: Gazebo Pose TF Bridge 수정

작성 시각: `2026-06-02 22:24:49 KST`

RViz에서 camera image는 표시되지만 TF tree에 실제 모델 frame인 `X1_asp/base_link`, `x500_gimbal_0/base_link`가 보이지 않는 문제를 확인했다. 원인은 Gazebo의 pose topic이 ROS2로 bridge되지 않았고, `pose_tf_broadcaster`도 `bridge_and_tf.launch.py`에서 실행되지 않는 구조였기 때문이다.

이번 수정에서는 `gazebo_env_setup/launch/bridge_and_tf.launch.py`에 다음 Gazebo pose topic bridge를 추가했다.

```text
/model/X1_asp/pose
/model/X1_asp/pose_static
/model/x500_gimbal_0/pose
/model/x500_gimbal_0/pose_static
```

같은 launch 파일에서 `pose_tf_broadcaster` node도 함께 실행되도록 추가했다.

```text
gazebo_env_setup/pose_tf_broadcaster
```

기존 launch에 남아 있던 `x500_depth_0` 기준 static TF publisher들은 현재 모델명과 맞지 않아 제거했다. 현재 모델 기준은 다음과 같다.

```text
X1_asp
x500_gimbal_0
```

`gazebo_env_setup/src/pose_tf_broadcaster.cpp`는 다음 topic을 모두 구독하도록 수정했다.

```text
/model/X1_asp/pose
/model/X1_asp/pose_static
/model/x500_gimbal_0/pose
/model/x500_gimbal_0/pose_static
```

수신한 transform의 parent frame이 `default`이면 `map`으로 바꾸고, child frame은 Gazebo가 제공하는 이름을 그대로 유지한다. 따라서 RViz/tf2에서 다음 frame을 확인할 수 있어야 한다.

```text
map
X1_asp/base_link
x500_gimbal_0/base_link
```

빌드는 다음 명령으로 확인했다.

```bash
colcon build --packages-select gazebo_env_setup
```

실행 중인 `turn_interfaces.launch.py`는 재시작해야 새 pose bridge와 `pose_tf_broadcaster`가 적용된다.

```bash
ros2 launch gazebo_env_setup turn_interfaces.launch.py
ros2 run tf2_ros tf2_echo map X1_asp/base_link
ros2 run tf2_ros tf2_echo map x500_gimbal_0/base_link
```

제어 코드, keyboard node, UGV bridge, image bridge topic, PX4/default.sdf는 수정하지 않았다.
