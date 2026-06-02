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

## 변경 기록: Mission1 UGV Path Follower 추가

작성 시각: `2026-06-03 00:12:53 KST`

Mission1 구현 시작을 위해 새 패키지 `asp_ugv_control`을 추가했다. 목표는 UGV `X1_asp`가 `mission.csv` waypoint를 저속으로 따라가고, `mission_type=2` waypoint에 도착하면 Mission2 시작 지점으로 판단하여 정지하는 것이다.

전체 명령 흐름은 다음과 같다.

```text
ugv_path_follower_node
  -> /command/ugv_cmd_vel
  -> ugv_cmd_vel_bridge
  -> /model/X1_asp/cmd_vel
  -> Gazebo X1_asp
```

새 패키지 구조는 다음과 같다.

```text
src/asp_ugv_control/
├── CMakeLists.txt
├── package.xml
├── config/path_follower_params.yaml
├── docs/MISSION1_UGV_PATH_FOLLOWER.md
├── launch/ugv_path_follower.launch.py
├── path/mission.csv
└── src/ugv_path_follower_node.cpp
```

`ugv_path_follower_node`의 주요 동작은 다음과 같다.

```text
node name: ugv_path_follower_node
output topic: /command/ugv_cmd_vel
TF: map -> X1_asp/base_link
state topic: /ugv/state
event topic: /ugv/mission_event
CSV format: x,y,mission_type,target_speed
```

`mission_type=2` waypoint에 도착하면 zero Twist를 publish하고 정지한다. 이때 `/ugv/state`는 `STOPPED`, `/ugv/mission_event`는 `MISSION2_START_REACHED`를 publish한다.

Mission1 path follower는 다음 제어 개념으로 구성했다.

```text
TF2 기반 현재 pose 조회
waypoint까지 거리 계산
target heading과 현재 yaw의 heading error 계산
waypoint 도달 시 다음 waypoint로 전환
target marker publish
mission_type=2를 Mission2 연결 지점으로 해석
```

현재 workspace 구조에 맞게 다음 제약을 적용했다.

```text
/model/X1_asp/cmd_vel 직접 publish 금지
/command/twist 사용 금지
/command/ugv_cmd_vel만 publish
UAV takeoff/rendezvous topic 연동 제거
Mission1 완료 이벤트를 /ugv/mission_event로 publish
```

빌드와 실행 파일 등록은 다음 명령으로 확인했다.

```bash
colcon build --packages-select asp_ugv_control
ros2 pkg executables asp_ugv_control
```

정상 출력:

```text
asp_ugv_control ugv_path_follower_node
```

실행 순서는 다음과 같다.

```bash
ros2 launch gazebo_env_setup turn_interfaces.launch.py
ros2 launch gazebo_env_setup ugv_bridge.launch.py
ros2 launch asp_ugv_control ugv_path_follower.launch.py
```

검증 명령은 다음과 같다.

```bash
ros2 topic echo /command/ugv_cmd_vel
ros2 topic echo /ugv/state
ros2 topic echo /ugv/mission_event
ros2 run tf2_ros tf2_echo map X1_asp/base_link
```

기존 `utilities_pkg`, UAV keyboard node, UGV keyboard node, `offboard_control.cpp`, `gazebo_env_setup` launch/config, PX4/default.sdf는 수정하지 않았다.

## 변경 기록: Mission2 Trigger FSM 추가

작성 시각: `2026-06-03 01:11:52 KST`

Mission2 진입 조건을 ArUco marker 검출이 아니라 UGV가 Mission1 종료 지점에 도착했는지 여부로 정리했다. 이에 따라 Mission2 Trigger의 주 publisher는 새 `mission_manager_node`가 담당하도록 구성했다.

최종 Mission2 trigger 흐름은 다음과 같다.

```text
ugv_path_follower_node
  -> /ugv/mission_event: MISSION2_START_REACHED
  -> mission_manager_node
  -> /mission/mission2_trigger: true
  -> /mission/state: UAV_TAKEOFF_READY
```

새 패키지 `asp_mission_manager`를 추가했다.

```text
src/asp_mission_manager/
├── asp_mission_manager/mission_manager_node.py
├── config/mission_manager_params.yaml
├── docs/MISSION_FSM.md
├── launch/mission_manager.launch.py
├── package.xml
├── setup.cfg
└── setup.py
```

`mission_manager_node`의 FSM 상태는 다음과 같다.

```text
INIT
READY
MISSION1_RUNNING
MISSION2_TRIGGERED
UAV_TAKEOFF_READY
UAV_TAKEOFF_REQUESTED
```

주요 topic은 다음과 같다.

```text
Subscriptions:
/mission/start
/mission/reset
/ugv/state
/ugv/mission_event
/perception/mission2_trigger

Publishers:
/mission/state
/mission/status
/mission/mission2_trigger
/command/takeoff
```

`/ugv/mission_event`에서 `MISSION2_START_REACHED`를 받으면 `MISSION1_RUNNING` 또는 `READY` 상태에서 `MISSION2_TRIGGERED`로 전환하고, `/mission/mission2_trigger`에 `true`를 한 번 publish한다. 이후 상태는 `UAV_TAKEOFF_READY`가 된다.

`auto_publish_takeoff`는 기본값 `false`로 두었다. 따라서 현재 단계에서는 Mission2 Trigger와 FSM 상태 전환만 확인하고, UAV takeoff 명령 자동 발행은 나중에 활성화한다.

또한 새 패키지 `asp_perception`을 추가했다. 이 패키지는 현재 Mission2 진입 판단의 주체가 아니라, 나중에 marker exploration 또는 precision landing에 사용할 perception 기능으로 유지한다.

```text
src/asp_perception/
├── CMakeLists.txt
├── config/aruco_detector_params.yaml
├── docs/MISSION2_TRIGGER_ARUCO.md
├── launch/ugv_aruco_detector.launch.py
├── package.xml
└── src/ugv_aruco_detector_node.cpp
```

`asp_perception`은 기본값에서 `/mission/mission2_trigger`를 publish하지 않는다. 대신 marker ID `0` 검출 시 `/perception/mission2_trigger`만 publish한다. mission manager는 이 값을 status에 기록만 하고, 현재 FSM 전이는 UGV 위치 도착 event로만 수행한다.

RViz의 Marker Detected display topic은 다음으로 변경했다.

```text
/perception/aruco/annotated
```

빌드는 다음 명령으로 확인했다.

```bash
colcon build --packages-select asp_mission_manager asp_perception
```

정상 실행 파일은 다음과 같다.

```text
asp_mission_manager mission_manager_node
asp_perception ugv_aruco_detector_node
```

Mission manager 단독 검증 명령은 다음과 같다.

```bash
ros2 launch asp_mission_manager mission_manager.launch.py
ros2 topic echo /mission/state
ros2 topic echo /mission/status
ros2 topic echo /mission/mission2_trigger
ros2 topic pub --once /ugv/mission_event std_msgs/msg/String "{data: MISSION2_START_REACHED}"
```

기존 UAV/UGV 제어 코드, keyboard node, bridge launch, PX4/default.sdf는 수정하지 않았다.

## 변경 기록: Mission2 이후 UAV Exploration 준비

작성 시각: `2026-06-03 02:33:00 KST`

Mission1 UGV path follower와 mission manager를 확장하여 Mission1 종료 이후 Mission2 준비 상태와 UAV exploration 준비 흐름을 연결했다. 현재까지 확인된 안정 동작 범위는 다음과 같다.

```text
Mission1 UGV waypoint 주행
  -> mission_type=2 waypoint 도착
  -> /ugv/mission_event: MISSION2_START_REACHED
  -> mission_manager_node
  -> /mission/mission2_trigger: true
  -> /mission/state: UAV_TAKEOFF_READY
  -> Mission2 준비 이륙 동작
```

아래 화면은 Mission1 이후 Mission2 준비 단계에서 UGV 근처에 UAV가 이륙해 있는 Gazebo 확인 화면이다.

![Mission1 이후 Mission2 준비 이륙 확인](docs/images/mission1_takeoff_ready_2026-06-03.png)

이번 변경으로 `asp_mission_manager` FSM에 UAV exploration 준비 상태를 추가했다.

```text
UAV_EXPLORATION_READY
UAV_EXPLORATION_RUNNING
UAV_EXPLORATION_COMPLETE
```

Mission manager는 `/mission/state`와 `/uav/exploration_start`를 통해 `asp_uav_control`의 exploration node와 연결된다. UAV exploration node는 launch만으로 UAV를 움직이지 않도록 구성했다.

```text
start_on_launch: false
start_on_mission_state: true
required_start_state: UAV_EXPLORATION_READY
```

따라서 `/mission/state`가 `UAV_EXPLORATION_READY`가 되거나, 수동으로 `/uav/exploration_start true`가 publish될 때만 `/command/pose` waypoint 명령을 publish한다. IDLE 상태에서는 `/command/pose`를 publish하지 않는다.

새 패키지 `asp_uav_control`을 추가했다.

```text
src/asp_uav_control/
├── asp_uav_control/uav_exploration_node.py
├── config/uav_exploration_params.yaml
├── docs/MISSION4_UAV_EXPLORATION.md
├── launch/uav_exploration.launch.py
└── path/uav_path.csv
```

UAV exploration node의 주요 topic은 다음과 같다.

```text
Subscriptions:
/mission/state
/uav/exploration_start
/perception/uav/marker_detections

Publishers:
/command/pose
/gimbal_pitch_degree
/uav/exploration_state
/uav/exploration_event
/mission/uav_exploration_complete
```

UGV와 UAV waypoint CSV도 현재 패키지 형식에 맞게 정리했다.

```text
UGV 원본 CSV:
/home/subin/Autonomous-System-Platform-final-project/ugv_controller/path/mission.csv

현재 UGV CSV:
src/asp_ugv_control/path/mission.csv
format: x,y,mission_type,target_speed

UAV 원본 CSV:
/home/subin/Autonomous-System-Platform-final-project/uav_controller/path/uav_path.csv

현재 UAV CSV:
src/asp_uav_control/path/uav_path.csv
format: x,y,z,yaw_deg,gimbal_pitch_deg,hold_sec,tag
```

다만 현재 waypoint CSV는 우리 Gazebo map과 완전히 맞는 좌표로 검증된 상태가 아니다. 따라서 다음 단계에서 실제 `map -> X1_asp/base_link`, `map -> x500_gimbal_0/base_link` TF와 marker 배치 기준으로 UGV/UAV waypoint를 다시 측정하고 보정해야 한다.

`asp_perception`에는 UAV ArUco detector node도 추가했다.

```text
src/asp_perception/src/uav_aruco_detector_node.cpp
src/asp_perception/launch/uav_aruco_detector.launch.py
src/asp_perception/config/uav_aruco_detector_params.yaml
src/asp_perception/docs/UAV_ARUCO_DETECTOR.md
```

UAV ArUco detector는 UAV camera image와 camera_info를 구독하고 `/perception/uav/marker_*` 계열 topic을 publish하도록 구성했다. 다만 아직 실제 Gazebo 실행에서 ArUco marker detection node가 정상 검출되는지는 확인하지 못했다. 현재 확인 완료 범위는 Mission1 이후 Mission2 준비 및 이륙 동작까지이다.

RViz의 Marker Detected display topic은 UAV perception 결과 확인을 위해 다음 topic으로 맞췄다.

```text
/perception/uav/aruco/annotated
```

빌드는 다음 명령으로 확인했다.

```bash
colcon build --packages-select asp_uav_control asp_ugv_control asp_mission_manager asp_perception gazebo_env_setup
```

추가로 `mission_logs/`는 runtime detection log가 생성되는 위치이므로 `.gitignore`에 포함했다.

## 변경 기록: UAV Exploration 좌표계 보정 및 성공 확인

작성 시각: `2026-06-03 04:15:44 KST`

UAV exploration 실행 중 UAV가 waypoint를 따라가기는 하지만, 예상한 marker 방향이 아니라 나무와 장애물 쪽으로 이동하는 문제가 있었다. 처음에는 waypoint CSV 좌표가 현재 Gazebo map과 맞지 않는 문제로 의심했지만, marker pose 기반으로 생성한 `uav_path_generated.csv`와 `uav_path_safe.csv`의 marker 좌표가 Gazebo marker spawn pose와 일치하는 것을 확인했다.

확인된 핵심 원인은 CSV가 아니라 좌표계 변환 방식이었다.

```text
/command/pose
  = ROS/Gazebo map ENU 절대좌표

PX4 TrajectorySetpoint
  = PX4 local NED 상대좌표
```

기존 `offboard_control`은 `/command/pose`의 map ENU 절대좌표를 그대로 ENU -> NED 축 변환만 수행했다.

```text
map ENU target: x=-120.26, y=35.85, z=5.40
기존 변환 결과: x=35.85, y=-120.26, z=-5.40
```

이 값은 PX4 local origin 기준 상대좌표가 아니라 Gazebo map 절대좌표를 축만 바꾼 값이어서, UAV가 현재 위치 기준 상승 명령이 아닌 먼 위치로 이동하는 명령처럼 해석될 수 있었다.

이번 수정에서는 `/command/pose` position setpoint에만 map origin offset을 적용했다. 첫 `/command/pose` 수신 시 `map -> x500_gimbal_0/base_link` TF를 lookup하고, 그 위치를 PX4 local 변환용 origin으로 잡는다. 이후 target map ENU에서 origin map ENU를 뺀 local ENU를 만든 뒤 기존 ENU -> NED 변환을 수행한다.

```text
target map ENU = (-120.26, 35.85, 5.40)
origin map ENU = (-120.26, 35.85, 0.40)
local ENU      = (0.00, 0.00, 5.00)
PX4 NED        = (0.00, 0.00, -5.00)
```

이 구조로 `takeoff_climb`가 현재 위치에서 수직 상승하는 명령으로 해석되는 것을 확인했고, UAV exploration이 정상 방향으로 진행되는 것을 확인했다.

추가한 `offboard_control` parameter는 다음과 같다.

```text
use_map_origin_offset: true
auto_set_map_origin_on_first_pose: true
map_origin_x: 0.0
map_origin_y: 0.0
map_origin_z: 0.0
map_frame: map
base_frame: x500_gimbal_0/base_link
publish_debug_setpoints: true
```

좌표 변환 확인을 위해 다음 debug topic을 추가했다.

```text
/debug/offboard/input_pose_enu
/debug/offboard/local_pose_enu
/debug/offboard/setpoint_pose_ned
/debug/offboard/frame_report
```

`/command/twist` manual control은 기존 속도 제어 흐름을 유지하고, map origin offset은 `/command/pose` 기반 position setpoint에만 적용했다. OFFBOARD/ARM runtime recovery, disarm altitude guard, gimbal pitch command도 유지했다.

UAV exploration node에서는 Mission2 Trigger 시점의 실제 UAV 위치를 기준으로 safe prefix를 동적으로 생성하도록 정리했다. UAV는 Mission1 동안 UGV 위에 실려 이동하므로 exploration 시작 위치는 launch 시점이 아니라 `/uav/exploration_start true` 또는 `/mission/state == UAV_EXPLORATION_READY`를 받은 순간의 `map -> x500_gimbal_0/base_link` TF이다.

```text
start signal
  -> current UAV TF lookup
  -> takeoff_climb
  -> safe_altitude
  -> transition_001, transition_002, ...
  -> scan waypoint 후보
```

`uav_path.csv`는 전체 비행 경로가 아니라 marker scan waypoint 후보 목록으로 취급한다. `dynamic_safe_prefix: true`일 때 runtime에서 앞부분에 safe prefix가 자동으로 붙는다. launch 직후에는 여전히 `/command/pose`를 publish하지 않고, start signal 이후에만 waypoint를 publish한다.

경로 생성 및 검증 보조 도구도 추가했다.

```text
tools/path_tools/extract_marker_poses.py
tools/path_tools/generate_uav_path_from_markers.py
tools/path_tools/generate_safe_uav_path.py
tools/path_tools/README_path_generation.md
```

생성 파일은 다음과 같다.

```text
tools/path_tools/marker_poses.csv
src/asp_uav_control/path/uav_path_generated.csv
src/asp_uav_control/path/uav_path_safe.csv
```

`uav_path_generated.csv`는 marker spawn pose 기반 관측 후보이고, `uav_path_safe.csv`는 시작 위치 기준 safe prefix를 앞에 붙인 검증용 후보이다. 기존 `uav_path.csv`는 자동으로 덮어쓰지 않는다.

marker detection 쪽에서는 `/perception/uav/marker_detections`를 `marker_id` key가 있는 구조화된 문자열로 publish하도록 보강했다. exploration node는 `marker_id` 또는 `id` key만 marker ID로 파싱하고, 좌표나 timestamp 숫자는 marker ID로 취급하지 않는다.

이번 시행착오에서 확인한 순서는 다음과 같다.

```text
1. 기존 CSV 좌표가 현재 map과 맞지 않을 가능성을 확인
2. Gazebo marker spawn pose에서 waypoint 후보 자동 생성
3. generated path와 safe path를 별도 CSV로 생성
4. launch가 실제 어떤 CSV를 읽는지 로그와 generated launch로 확인
5. Mission2 Trigger 시점 TF 기준 dynamic safe prefix 생성으로 변경
6. /command/pose는 정상 map ENU 절대좌표임을 확인
7. offboard_control이 PX4 local NED 상대좌표가 아니라 절대좌표를 축 변환만 하고 있던 문제 확인
8. map origin offset 적용 후 local ENU -> PX4 NED 변환으로 보정
9. UAV exploration 정상 방향 진행 확인
```

검증에 사용한 주요 명령은 다음과 같다.

```bash
colcon build --packages-select asp_uav_control asp_perception px4_ros_com
colcon build --packages-select px4_ros_com
ros2 topic echo /debug/offboard/input_pose_enu
ros2 topic echo /debug/offboard/local_pose_enu
ros2 topic echo /debug/offboard/setpoint_pose_ned
ros2 topic echo /debug/offboard/frame_report
ros2 topic echo /uav/exploration_event
ros2 topic echo /command/pose
```

runtime log가 저장되는 `mission_logs/`와 diagnostic log 디렉토리는 git에 포함하지 않도록 `.gitignore`에 정리했다.
