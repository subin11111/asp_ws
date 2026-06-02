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
