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

## 변경 기록: Final mission_timer_node launch 연결 및 조기 disarm 방지

작성 시각: `2026-06-17 KST`

Final mission 실행 중 `mission_timer_node`가 launch에 포함되지 않아 시간이 측정되지 않거나,
반대로 너무 이르게 `/command/disarm`를 받아 조기 종료되는 문제를 정리했다.

이번 변경의 핵심은 다음 두 가지다.

```text
1. final_mission.launch.py에 기존 mission_timer_node를 직접 추가
2. mission_timer.cpp는 수정하지 않고, PX4 bridge 쪽 조기 disarm만 방지
```

### launch 연결

기존 `gazebo_env_setup/src/mission_timer.cpp`는 그대로 유지하고, 아래 launch에 node를 추가했다.

```text
src/asp_final_bringup/launch/final_mission.launch.py
```

추가된 node는 다음과 같다.

```text
package: gazebo_env_setup
executable: mission_timer_node
name: mission_timer_node
```

이제 final mission launch를 실행하면 mission timer도 함께 뜬다.

```bash
ros2 launch asp_final_bringup final_mission.launch.py
```

### 조기 종료 원인

최근 로그를 확인한 결과, timer가 `Mission4` 종료가 아니라 Mission2 시작 직후 약 25초 만에 끝난 원인은
`mission_timer.cpp` 자체가 아니라 `asp_final_px4_offboard_bridge`가 너무 이르게 `/command/disarm`를
publish했기 때문이었다.

실제 로그 순서는 다음과 같았다.

```text
1. mission_timer_node: Mission started
2. PX4 bridge: Mission2 takeoff origin latch
3. PX4 bridge: OFFBOARD/ARM accepted
4. PX4 bridge: Requested PX4 DISARM and published /command/disarm
5. mission_timer_node: Mission finished
```

`mission_timer.cpp`는 `/command/disarm`와 UGV/UAV 거리 조건으로 종료를 판단하므로, 이 조기 disarm 신호가
들어가면 Mission4 이전에도 타이머가 끝날 수 있었다.

### 해결 방식

`mission_timer.cpp`는 수정하지 않았다. 대신 `asp_final_px4_offboard_bridge`에서 실제 landing 요청이
들어온 뒤에만 auto-disarm이 동작하도록 조건을 좁혔다.

즉 다음과 같이 동작하도록 정리했다.

```text
Mission1~3: /command/disarm publish 금지
Mission4 landing 요청 이후: landed 상태가 확인되면 disarm 허용
```

이렇게 하면 기존 `mission_timer_node`의 종료 기준은 그대로 유지하면서도, 실질적으로 Mission4 이후에만
timer가 끝나도록 유도할 수 있다.

### 변경 파일

```text
README.md
src/asp_final_bringup/launch/final_mission.launch.py
src/asp_final_px4_bridge/asp_final_px4_bridge/px4_offboard_bridge.py
```

### 반영 방법

launch와 PX4 bridge 변경이 install에 반영되도록 다음 빌드를 수행한다.

```bash
cd /home/sunny/asp_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select asp_final_bringup asp_final_px4_bridge
source install/setup.bash
```

---

## 변경 기록: Final RViz 시각화, rendezvous 파라미터화, mission timer 종료 신호

작성 시각: `2026-06-11 KST`

Final mission 시험 운용 중 다음 세 가지 운영 편의 기능을 추가했다.

```text
1. RViz에서 UGV/UAV waypoint와 ArUco 검출 위치를 색상별로 확인
2. UGV Mission3 최종 rendezvous waypoint를 ros2 param set으로 변경
3. PX4 자동 disarm 시 시험용 mission_timer_node가 보는 /command/disarm도 함께 publish
```

### RViz 시각화

새 node `asp_final_visualization`을 추가했다. 이 node는 final mission CSV와 perception 결과를 RViz용
`visualization_msgs/MarkerArray`로 변환한다.

```text
src/asp_final_bringup/asp_final_bringup/final_visualization.py
```

publish topic은 다음과 같다.

```text
/asp_final/visualization/ugv_waypoints
/asp_final/visualization/uav_waypoints
/asp_final/visualization/detected_aruco_markers
```

색상 구분은 다음 기준이다.

```text
UGV Mission1 carrier path: cyan/blue
UGV Mission3 rendezvous path: orange
UAV Mission2 waypoint path: purple/light blue
Detected ArUco marker pose: green sphere + label
```

ArUco image 확인은 기존 detector가 publish하는 annotated image를 사용한다.

```text
/asp_final/perception/uav/aruco/annotated
```

RViz config와 launch를 추가했다.

```text
src/asp_final_bringup/config/asp_final_mission.rviz
src/asp_final_bringup/launch/rviz.launch.py
```

실행 순서는 다음과 같다.

```bash
px4humble
ros2 launch asp_final_bringup final_mission.launch.py
```

다른 터미널에서 RViz를 실행한다.

```bash
px4humble
ros2 launch asp_final_bringup rviz.launch.py
```

`final_mission.launch.py`에는 `asp_final_visualization` node를 같이 실행하도록 추가했다.
따라서 mission launch가 떠 있으면 RViz launch는 RViz window만 띄우면 된다.

### UGV rendezvous 최종 waypoint 파라미터화

시험 중 UGV의 Mission3 최종 rendezvous point를 CSV 수정 없이 바꿀 수 있도록
`asp_final_ugv_path_follower`에 다음 parameter를 추가했다.

```text
rendezvous_goal_x
rendezvous_goal_y
rendezvous_goal_target_speed
```

기본값은 `.nan`이며, 이 경우 CSV 마지막 waypoint를 그대로 사용한다.

실행 중 최종 rendezvous point를 바꾸는 예시는 다음과 같다.

```bash
ros2 param set /asp_final_ugv_path_follower rendezvous_goal_x -57.949
ros2 param set /asp_final_ugv_path_follower rendezvous_goal_y 101.780
```

필요하면 마지막 waypoint의 target speed도 바꿀 수 있다.

```bash
ros2 param set /asp_final_ugv_path_follower rendezvous_goal_target_speed 1.0
```

이 변경은 CSV 파일을 다시 쓰지 않는다. node memory 안의 `self.paths["rendezvous"][-1]`만 갱신한다.

### mission_timer_node 종료 신호 호환

시험에서는 `gazebo_env_setup/src/mission_timer.cpp`의 `mission_timer_node`가 시간을 측정한다.
이 node는 미션 종료를 `/command/disarm` 수신과 UGV/UAV 거리 조건으로 판단한다.

```text
subscribe: /command/disarm
finish condition: distance(map->X1_asp, map->x500_gimbal_0) <= 1.0m
```

반면 final mission stack은 landing 완료 시 `/asp_final/uav/land`를 사용하고, PX4 bridge가
`VEHICLE_CMD_COMPONENT_ARM_DISARM`을 직접 publish한다. 이 때문에 착륙과 PX4 disarm이 진행되어도
`mission_timer_node`가 기대하는 `/command/disarm`이 나오지 않는 문제가 있었다.

이번 변경에서는 `asp_final_px4_offboard_bridge`가 자동 disarm을 요청하는 순간 다음 두 동작을 함께 수행한다.

```text
1. /fmu/in/vehicle_command 로 PX4 disarm command publish
2. /command/disarm std_msgs/Bool true publish
```

따라서 `mission_timer_node`는 기존 코드 수정 없이 종료 신호를 받을 수 있다.

### 변경 파일

```text
README.md
src/asp_final_bringup/asp_final_bringup/__init__.py
src/asp_final_bringup/asp_final_bringup/final_visualization.py
src/asp_final_bringup/config/asp_final_mission.rviz
src/asp_final_bringup/launch/final_mission.launch.py
src/asp_final_bringup/launch/rviz.launch.py
src/asp_final_bringup/package.xml
src/asp_final_bringup/setup.py
src/asp_final_px4_bridge/asp_final_px4_bridge/px4_offboard_bridge.py
src/asp_final_ugv/asp_final_ugv/ugv_path_follower.py
src/asp_final_ugv/config/ugv_params.yaml
src/asp_final_ugv/package.xml
```

### 검증

문법 검사는 다음 명령으로 확인했다.

```bash
python3 -m py_compile \
  src/asp_final_bringup/asp_final_bringup/final_visualization.py \
  src/asp_final_bringup/launch/final_mission.launch.py \
  src/asp_final_bringup/launch/rviz.launch.py \
  src/asp_final_px4_bridge/asp_final_px4_bridge/px4_offboard_bridge.py \
  src/asp_final_ugv/asp_final_ugv/ugv_path_follower.py
```

빌드는 다음 명령으로 확인했다.

```bash
colcon build --packages-select \
  asp_final_bringup asp_final_px4_bridge asp_final_ugv
```

---

## 변경 기록: Final UAV Mission2 전체 검출 및 gimbal pitch 보정

작성 시각: `2026-06-07 01:53:15 KST`

Mission2에서 marker 일부가 끝까지 검출되지 않는 문제를 다시 점검했다. 이번 점검에서는 전체 CSV 기준으로
marker `0~9`를 모두 볼 수 있도록 waypoint 진행 조건과 gimbal pitch 명령 전달을 조정하고, 검출 CSV에
최초/최종 시각과 횟수를 남기도록 보강했다.

### Waypoint 진행 조건 조정

기존 설정에서는 waypoint에 거의 도착해도 `z` 오차와 도착 직후 marker 대기 정책 때문에 다음 waypoint로
넘어가는 타이밍이 불안정했다. 특히 어떤 marker는 이미 카메라 시야에 들어왔는데도 오래 머물지 못하거나,
반대로 도착 판정이 너무 늦어 다음 marker 탐색 시간이 줄었다.

이번 변경에서는 코드 로직은 유지하고 parameter만 조정해 marker waypoint에서 더 확실히 도착한 뒤 진행하도록 했다.

```text
waypoint_xy_tolerance_m: 2.5
waypoint_z_tolerance_m: 3.0
ignore_yaw_for_waypoint_reached: true
waypoint_stuck_timeout_sec: 18.0
max_same_waypoint_sec: 20.0
continue_on_marker_timeout: false
marker_wait_timeout_sec: 8.0
do_not_block_waypoint_progress_on_marker: false
```

이 설정으로 waypoint에 먼저 충분히 접근하고, marker가 보이면 hold를 유지한 뒤 다음 waypoint로 넘어가도록 했다.
Mission2 transition corridor와 wall/roof detect waypoint를 다시 도는 상황도 줄였다.

### Gimbal pitch 명령 보정

기존 `asp_final_uav_mission_node`는 `Float32`로 `/asp_final/uav/gimbal_pitch_deg`를 publish하지만,
PX4 bridge 쪽에서 이를 실제 gimbal manager command로 전달하지 않아 camera가 waypoint CSV의
pitch 의도를 반영하지 못할 가능성이 있었다.

이번 변경에서는 `/asp_final/uav/gimbal_pitch_deg` 값을 PX4 gimbal manager 명령으로 전달하도록 바꿨다.
PX4 bridge는 첫 gimbal 명령 시 `VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE`를 한 번 보내고, 이후
각 pitch 값을 `VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW`로 publish한다.

```text
/asp_final/uav/gimbal_pitch_deg
  -> asp_final_px4_offboard_bridge
  -> /fmu/in/vehicle_command
```

주요 원인은 waypoint 위치보다 gimbal pitch 명령 전달 방식일 가능성이 컸고, 이 부분을 수정했다.

### 검출 CSV 시각 정보 추가

기존 `detected_marker_csv.py`는 marker별 최종 map 좌표만 저장했다. 이번 변경에서는 CSV 저장 node가
marker별 최초 검출 시각, 마지막 검출 시각, 검출 횟수를 함께 저장하도록 했다.

```text
first_seen_sec
last_seen_sec
detection_count
```

또한 최초 검출 시점은 로그에도 남긴다.

```text
first detected UAV marker <id> at <sec>s
```

이제 다음 실행부터는 marker가 처음 보인 시점과 waypoint 이동 로그를 비교해, 실제 검출 이후에도 오래 머문
marker가 어떤 것인지 바로 확인할 수 있다.

### 최신 검출 결과

최신 Mission2 실행에서는 marker `0~9`가 모두 한 번 이상 검출되었다. 저장 CSV 기준 결과는 다음과 같다.

```text
0, 1, 2, 3, 4, 5, 6, 7, 8, 9 detected
```

marker별 좌표 비교 결과는 다음과 같다.

```text
ID 0: dxy=0.47m, dz=0.10m, d3=0.48m
ID 1: dxy=5.35m, dz=6.42m, d3=8.36m
ID 2: dxy=0.81m, dz=-0.17m, d3=0.83m
ID 3: dxy=0.59m, dz=1.11m, d3=1.25m
ID 4: dxy=0.50m, dz=0.17m, d3=0.52m
ID 5: dxy=1.51m, dz=-0.89m, d3=1.75m
ID 6: dxy=1.09m, dz=1.02m, d3=1.49m
ID 7: dxy=0.81m, dz=-0.45m, d3=0.93m
ID 8: dxy=0.67m, dz=0.18m, d3=0.70m
ID 9: dxy=0.50m, dz=-0.31m, d3=0.59m
```

대부분의 marker는 실제 위치와 잘 맞았다. 다만 `1`번 marker는 최신 실행에서 좌표 오차가 크게 튀었고,
검출 좌표가 실제 `1`번보다 실제 `8`번 위치에 더 가까웠다. 따라서 전체 검출은 성공했지만 `1`번 marker의
pose 안정성은 추가 확인이 필요하다.

최종 변경 파일은 다음과 같다.

```text
src/asp_final_perception/asp_final_perception/detected_marker_csv.py
src/asp_final_px4_bridge/asp_final_px4_bridge/px4_offboard_bridge.py
src/asp_final_uav/config/uav_params.yaml
src/asp_final_uav/path/mission2_uav_waypoints.csv
```

검증은 다음 명령으로 확인했다.

```bash
python3 -m py_compile \
  src/asp_final_perception/asp_final_perception/detected_marker_csv.py \
  src/asp_final_px4_bridge/asp_final_px4_bridge/px4_offboard_bridge.py

colcon build --packages-select \
  asp_final_perception asp_final_px4_bridge
```

---

## 변경 기록: Final UAV marker 좌표 저장 및 Mission2 검출 안정화

작성 시각: `2026-06-07 00:35:39 KST`

Mission2에서 UAV가 marker를 실제로 보고 있는지, 그리고 그 결과를 Mission2 runtime 분석에 사용할 수 있는지
확인하기 위해 ArUco detector, marker CSV 저장, UAV path runtime 파라미터를 함께 정리했다.

이번 변경에서는 ArUco detector가 `vision_msgs/Detection3DArray`를 publish하도록 바꾸고, UAV marker detection
node가 marker map pose를 CSV로 남기며, Mission2 runtime path가 generated path가 아닌 senior path만
사용하도록 guard를 강화했다.

### ArUco detector 구조 정리

`asp_final_perception/aruco_detector.py`는 다음 topic을 publish한다.

```text
/asp_final/perception/uav/marker_detections
/asp_final/perception/uav/marker_id
/asp_final/perception/uav/aruco/annotated
/asp_final/perception/landing/marker_detections
/asp_final/perception/landing/marker_id
```

UAV mode에서는 marker pose를 `vision_msgs/Detection3DArray`로 publish하고, 각 detection의
`hypothesis.class_id`에 marker ID를 문자열로 저장한다. landing mode도 동일한 구조를 사용한다.

또한 detector는 다음 parameter를 사용한다.

```text
image_topic
camera_info_topic
map_frame
camera_frame
dictionary
marker_size_m
allowed_marker_ids
```

Mission2용 UAV detector는 `allowed_marker_ids: 0~9`, landing detector는 `allowed_marker_ids: 10`
로 제한했다.

### detected_marker_csv 추가

새 node `asp_final_detected_marker_csv`를 추가했다. 이 node는 `/asp_final/perception/uav/marker_detections`
를 구독하고, marker별 마지막 map 좌표를 CSV로 저장한다.

저장 형식은 다음과 같다.

```csv
marker_id,map_x,map_y,map_z
```

기본 CSV 경로는 다음과 같다.

```text
/home/desktop1/ros2_ws/mission_logs/asp_final_detected_markers.csv
```

같은 marker가 여러 번 검출되면 최신 좌표로 덮어쓴다. 종료 시점에 한 번만 CSV를 기록하므로 실행 중에는 메모리에
유지되고, node 종료 시 파일로 flush된다.

### Mission2 runtime path guard 강화

Mission2에서 generated path가 잘못 들어가면 marker scan waypoint가 wall 후보를 과도하게 순회하거나
현재 world와 맞지 않는 좌표를 사용하게 된다. 이를 막기 위해 `asp_final_uav` 쪽 runtime path guard를 정리했다.

주요 parameter는 다음과 같다.

```text
allow_generated_path_runtime: false
runtime_path_must_contain: senior
waypoint_xy_tolerance_m: 2.0
waypoint_z_tolerance_m: 2.5
ignore_yaw_for_waypoint_reached: true
min_waypoint_separation: 4.0
continue_on_marker_timeout: true
marker_wait_timeout_sec: 3.0
```

`uav_mission_node`는 launch/runtime path 이름이 `senior`를 포함하지 않으면 path를 거부하도록 유지했다.
이렇게 하면 Mission2 실운용에서는 `mission2_uav_waypoints.csv`나 generated/debug CSV가 실수로 들어가지 않는다.

### Mission2 검출 결과 확인 절차

검증 절차는 다음과 같다.

```bash
ros2 launch asp_final_bringup final_mission.launch.py
ros2 topic echo /asp_final/perception/uav/marker_id
ros2 topic echo /asp_final/perception/uav/marker_detections
```

실행 후 CSV를 확인한다.

```bash
cat /home/desktop1/ros2_ws/mission_logs/asp_final_detected_markers.csv
```

이 값과 실제 marker world pose를 비교해 Mission2 waypoint가 marker 근처를 제대로 통과하는지 확인할 수 있다.

### 최신 실행 관찰

최신 실행에서는 UAV가 여러 marker를 검출하는 것은 확인했지만, 일부 marker는 여전히 Mission2 runtime 안에서
확실히 잡히지 않았다. 특히 roof marker 일부는 waypoint 도착 판정과 gimbal 방향의 영향으로 놓칠 가능성이 있었다.

따라서 다음 단계는

```text
1. detected_marker_csv 결과와 실제 marker pose를 비교
2. 놓친 marker의 waypoint/hang time/gimbal pitch를 다시 조정
3. Mission2 전체 0~9 marker 검출 성공 여부를 재검증
```

최종 변경 파일은 다음과 같다.

```text
src/asp_final_perception/asp_final_perception/aruco_detector.py
src/asp_final_perception/asp_final_perception/detected_marker_csv.py
src/asp_final_perception/config/perception_params.yaml
src/asp_final_perception/package.xml
src/asp_final_perception/setup.py
src/asp_final_uav/config/uav_params.yaml
```

검증은 다음 명령으로 확인했다.

```bash
python3 -m py_compile \
  src/asp_final_perception/asp_final_perception/aruco_detector.py \
  src/asp_final_perception/asp_final_perception/detected_marker_csv.py

colcon build --packages-select \
  asp_final_perception asp_final_uav
```

---

## 변경 기록: Final UAV 착륙 yaw 정렬 및 landing 유지 보강

작성 시각: `2026-06-05 01:29:44 KST`

Mission4 착륙 단계에서 UAV가 UGV landing marker 위에 접근할 때 yaw가 맞지 않거나, 착륙 직전 target yaw가
불안정하게 바뀌는 문제를 확인했다. landing target xy만 맞추면 vehicle heading이 어긋나도 내려가므로,
UGV heading 기준으로 착륙 yaw를 유지하도록 보강했다.

이번 변경에서는 해당 TF의 yaw도 함께 읽어 landing command pose에 반영한다.

### landing yaw 계산 방식

기존 `current_ugv_landing_xy()`는 `(x, y)`만 반환했다. 이를 `current_ugv_landing_pose()`로 바꾸어
UGV landing frame 또는 UGV base frame에서 `(x, y, yaw)`를 함께 읽도록 수정했다.

우선순위는 다음과 같다.

```text
1. map -> X1_asp/aruco_marker_10_link
2. map -> X1_asp/base_link
```

lookup에 성공하면 해당 yaw를 `landing_yaw`로 사용하고, 실패하면 현재 UAV yaw를 유지한다.

### landing command pose 반영

landing phase에서 target xy를 계산한 뒤 command pose를 publish할 때 yaw에도 `landing_yaw`를 넣는다.

```text
self.cmd_pose_pub.publish(self.pose_msg(target_x, target_y, target_z, landing_yaw))
```

이렇게 하면 UAV가 UGV 위에 접근하면서 heading도 함께 정렬된다.

### landing 유지 보강

착륙 시작 후 `landing_land_started`가 설정되면 `landing_touchdown_setpoint_m` 높이를 유지하며 계속 같은 target을
publish한다. 이때 yaw도 마지막 계산값이 아니라 매 tick의 `landing_yaw`를 계속 사용하므로, UGV yaw 변화가
있으면 착륙 heading도 따라간다.

이 변경은 takeoff/explore 로직에는 영향이 없고 `phase == "landing"` 안에서만 적용된다.

최종 변경 파일은 다음과 같다.

```text
src/asp_final_uav/asp_final_uav/uav_mission_node.py
```

검증은 다음 명령으로 확인했다.

```bash
python3 -m py_compile \
  src/asp_final_uav/asp_final_uav/uav_mission_node.py

colcon build --packages-select asp_final_uav
```

---

## 변경 기록: Final Mission2/3 완료 조건 및 착륙 전환 정리

작성 시각: `2026-06-05 00:33:04 KST`

Final mission 전체 흐름을 점검하면서 Mission2 UAV exploration, Mission3 UGV rendezvous, Mission4 precision landing
사이 완료 조건과 전환 타이밍을 다시 정리했다. 목표는 Mission2와 Mission3가 병렬로 진행되되, 둘 다 끝나기 전에는
착륙 단계로 넘어가지 않도록 하는 것이다.

### mission supervisor 병렬 완료 조건

`asp_final_mission_supervisor`는 다음 조건에서만 Mission4로 넘어가도록 정리했다.

```text
state == MISSION2_3_PARALLEL
and mission2_done == true
and rendezvous_done == true
```

`mission2_done`는 다음 두 경로 중 하나로 true가 된다.

```text
/asp_final/uav/mission2_complete == true
/asp_final/uav/exploration_state in {"mission2_complete", "complete"}
```

`rendezvous_done`는 다음 두 경로 중 하나로 true가 된다.

```text
/asp_final/ugv/rendezvous_reached == true
/asp_final/ugv/state == "MISSION3_COMPLETE"
```

위 두 조건이 모두 만족될 때만 `/asp_final/landing/start`를 publish하고 `MISSION4_LANDING`으로 전환한다.

### UAV landing phase 정리

`asp_final_uav_mission_node`의 landing phase는 다음 순서로 동작한다.

```text
1. UGV landing pose 또는 마지막 유효 xy를 target으로 사용
2. landing detection이 유효하면 marker map_x/map_y로 target 갱신
3. xy error가 크면 approach altitude 유지
4. xy error가 충분히 작아지면 descent 진행
5. ready_to_land 또는 timed_out_on_target이면 /asp_final/uav/land publish
6. 동시에 /asp_final/landing/complete true publish
```

이 변경은 `phase == "landing"` 안에서만 적용되며, takeoff/explore waypoint publish 로직은 건드리지 않는다.

### UGV rendezvous 완료 후 정지

Mission3 완료 시 UGV가 rendezvous 최종 waypoint에 도달하면 zero Twist를 계속 publish하며 `MISSION3_COMPLETE`
상태를 유지하도록 했다. 이로써 UAV landing phase 동안 UGV가 불필요하게 다시 움직이지 않는다.

### final launch 실행 구성

`final_mission.launch.py`는 다음 node를 함께 실행하도록 유지했다.

```text
asp_final_px4_offboard_bridge
asp_final_mission_supervisor
asp_final_ugv_path_follower
asp_final_uav_mission_node
asp_final_uav_aruco_detector
asp_final_landing_aruco_detector
asp_final_detected_marker_csv
```

최종 변경 파일은 다음과 같다.

```text
src/asp_final_mission/asp_final_mission/mission_supervisor.py
src/asp_final_uav/asp_final_uav/uav_mission_node.py
src/asp_final_ugv/asp_final_ugv/ugv_path_follower.py
src/asp_final_bringup/launch/final_mission.launch.py
```

검증은 다음 명령으로 확인했다.

```bash
python3 -m py_compile \
  src/asp_final_mission/asp_final_mission/mission_supervisor.py \
  src/asp_final_uav/asp_final_uav/uav_mission_node.py \
  src/asp_final_ugv/asp_final_ugv/ugv_path_follower.py \
  src/asp_final_bringup/launch/final_mission.launch.py

colcon build --packages-select \
  asp_final_mission asp_final_uav asp_final_ugv asp_final_bringup
```

---

## 변경 기록: Final Mission2 UAV 안전 경로 및 빠른 이륙 보정

작성 시각: `2026-06-04 16:05:14 KST`

Final mission 전체 흐름에 맞게 Mission2 UAV runtime path와 PX4 offboard bridge를 다시 조정했다. 목표는
Mission1으로 UGV 위에서 이동한 뒤 Mission2 시작 시점에 UAV가 안정적으로 현재 위치 기준으로 이륙하고,
정해둔 senior waypoint path를 빠르게 따라가도록 만드는 것이었다.

이번 수정에서는 Mission2 runtime path를 senior CSV 기준으로 고정하고, PX4 bridge가 Mission2 origin과
PX4 local anchor를 함께 사용해 빠르게 climb 하도록 보정했다.

### UAV runtime path 고정

`final_mission.launch.py`와 `uav_params.yaml` 기준 Mission2에서 사용하는 path를 senior CSV 기준으로 유지했다.
generated path는 runtime에서 사용하지 않도록 했다.

```text
mission2_uav_waypoints.csv
```

transition corridor는 launch/runtime에서 자동으로 생성하되, start 위치와 첫 scan waypoint 사이를 직선 segment로
분할하도록 유지했다.

### PX4 빠른 이륙 feedforward

`asp_final_px4_offboard_bridge`에 climb feedforward parameter를 추가했다.

```text
fast_climb_velocity_feedforward_mps
fast_climb_acceleration_feedforward_mps2
fast_climb_error_threshold_m
```

현재 map z와 target map z 차이가 threshold보다 크면 z축 velocity/acceleration feedforward를 함께
TrajectorySetpoint에 넣는다. 이로써 Mission2 시작 직후 상승 응답이 빨라진다.

### Mission2 origin과 PX4 local anchor

브리지는 `/asp_final/uav/mission2_takeoff_origin`을 받으면 map anchor를 저장하고, 동시에 유효한
`/fmu/out/vehicle_local_position`이 있으면 PX4 local NED anchor도 함께 latch한다.

map target pose -> PX4 local setpoint 변환식은 다음과 같다.

```text
delta_map = target_map - map_anchor
delta_ned = ENU_TO_NED(delta_map)
target_px4 = px4_anchor + delta_ned
```

이 구조로 Mission2 시작 위치가 world spawn이 아니라 UGV 위 현재 위치여도 안정적으로 이륙한다.

### launch/runtime guard

Mission2 path 실행 전에 필요한 조건은 다음과 같다.

```text
has_map_anchor == true
has_px4_anchor == true
preoffboard_setpoint_count 충족
```

이 조건이 만족되기 전에는 PX4 offboard/arm 요청을 반복하지 않는다.

최종 변경 파일은 다음과 같다.

```text
src/asp_final_bringup/launch/final_mission.launch.py
src/asp_final_px4_bridge/asp_final_px4_bridge/px4_offboard_bridge.py
src/asp_final_uav/config/uav_params.yaml
src/asp_final_uav/path/mission2_uav_waypoints.csv
```

검증은 다음 명령으로 확인했다.

```bash
python3 -m py_compile \
  src/asp_final_bringup/launch/final_mission.launch.py \
  src/asp_final_px4_bridge/asp_final_px4_bridge/px4_offboard_bridge.py

colcon build --packages-select \
  asp_final_bringup asp_final_px4_bridge asp_final_uav
```

---

## 변경 기록: Mission2/3 Runtime 경로 가드 및 UGV 전용 지연

작성 시각: `2026-06-04 02:06:45 KST`

Mission2/3 병렬 구간에서 runtime path가 잘못 선택되거나 UGV rendezvous가 너무 빨리 시작되는 문제를 막기 위해
UAV runtime path guard와 UGV rendezvous 시작 지연을 추가했다.

### UAV runtime path guard

Mission2에서 generated/debug path가 실수로 들어오면 실제 world와 맞지 않는 waypoint로 비행할 수 있다.
이를 막기 위해 runtime path 이름 검사와 generated path 차단을 추가했다.

```text
allow_generated_path_runtime: false
runtime_path_must_contain: senior
```

파일 이름이 `generated`를 포함하거나 `senior` token을 포함하지 않으면 runtime path를 거부한다.
거부 시 다음 event를 publish한다.

```text
ERROR:RUNTIME_PATH_REJECTED_GENERATED:<basename>
ERROR:RUNTIME_PATH_REJECTED_MISSING_TOKEN:<basename>:required=<token>
```

정상 수용 시에는 다음 event를 publish한다.

```text
RUNTIME_PATH_ACCEPTED:<basename>
```

### static prefix 제거 및 waypoint 근접 중복 제거

Mission2 latched origin 기반 runtime에서는 CSV 안의 spawn/takeoff prefix가 그대로 들어가면 현재 위치와 맞지 않는다.
따라서 scan waypoint load 단계에서 다음 tag를 제거한다.

```text
takeoff_climb
safe_altitude
transition_*
*spawn*
```

또한 연속 waypoint가 너무 가까우면 뒤 waypoint를 제거하고 hold_sec만 앞 waypoint에 합친다.

```text
CLOSE_WAYPOINT_REMOVED:<tag>:kept=<previous>:distance=<d>
```

### UGV rendezvous 시작 지연

Mission2 trigger 직후 UGV가 너무 이르게 움직이면 UAV Mission2 takeoff/origin과 충돌 가능성이 있었다.
따라서 Mission3 rendezvous 시작 시 fixed delay를 둘 수 있도록 parameter를 추가했다.

```text
rendezvous_start_delay_sec
```

delay 동안은 zero Twist를 유지하고, 이후 Mission3 motion을 시작한다.

### 진단 도구

추가/수정한 진단 도구는 다음과 같다.

```text
/uav/exploration_state
/uav/exploration_event
/ugv/state
/ugv/mission_event
```

최종 변경 파일은 다음과 같다.

```text
src/asp_uav_control/asp_uav_control/uav_exploration_node.py
src/asp_uav_control/config/uav_exploration_params.yaml
src/asp_ugv_control/asp_ugv_control/ugv_rendezvous_node.py
src/asp_ugv_control/config/rendezvous_params.yaml
```

---

## 변경 기록: Mission2 PX4 Local Anchor 이륙 보정

작성 시각: `2026-06-03 14:29:09 KST`

Mission2 시작 시점에 UAV가 world origin 기준이 아니라 UGV 위 현재 위치에서 이륙하도록 PX4 local anchor 변환을 보정했다.

이번 수정에서는 Mission2 origin latch 시점에 두 기준을 동시에 저장하도록 변경했다.

```text
1. Mission2 map ENU anchor
2. Mission2 PX4 local NED anchor
```

`offboard_control.cpp`의 주요 변경 사항은 다음과 같다.

```text
Mission2 origin message 수신 시 map anchor 저장
/fmu/out/vehicle_local_position 유효 시 px4 local anchor 저장
target map pose를 map anchor 기준 delta로 변환
delta ENU -> delta NED 변환 후 px4 local anchor에 더함
```

이렇게 하면 Mission2 start 시점의 실제 UAV 위치가 PX4 local frame 기준 어디에 있든, map waypoint와의 상대 차이만으로
정확한 setpoint를 만들 수 있다.

최종 변경 파일은 다음과 같다.

```text
src/utilities_pkg/px4_ros_com/src/examples/offboard/offboard_control.cpp
```

---

## 변경 기록: UAV Exploration 좌표계 보정 및 성공 확인

작성 시각: `2026-06-03 04:15:44 KST`

Mission2 UAV exploration에서 `/command/pose`가 map 좌표 그대로 PX4 local setpoint로 들어가 비행이 어긋나는 문제를
수정하고, 실제 Mission2 trigger 시점 기준 dynamic safe prefix 생성 방식으로 바꾸어 탐색 성공을 확인했다.

이번 수정에서는 `/command/pose` position setpoint에만 map origin offset을 적용했다. 첫 `/command/pose` 수신 시
`map -> x500_gimbal_0/base_link` TF를 lookup하고, 그 위치를 PX4 local 변환용 origin으로 잡는다. 이후 target map ENU에서
origin map ENU를 뺀 local ENU를 만든 뒤 기존 ENU -> NED 변환을 수행한다.

```text
target_local_enu = target_map_enu - mission2_origin_map_enu
target_ned = ENU_TO_NED(target_local_enu)
```

점검 결과는 다음 순서로 확인했다.

```text
1. senior CSV와 generated CSV 분리 확인
2. Mission2 start event publish 확인
3. first /command/pose와 Mission2 origin 비교
4. launch가 실제 어떤 CSV를 읽는지 로그와 generated launch로 확인
5. Mission2 Trigger 시점 TF 기준 dynamic safe prefix 생성으로 변경
```

최종 변경 파일은 다음과 같다.

```text
src/asp_uav_control/asp_uav_control/uav_exploration_node.py
src/asp_uav_control/config/uav_exploration_params.yaml
src/asp_uav_control/launch/uav_exploration_safe.launch.py
```

---

## 변경 기록: Mission2 이후 UAV Exploration 준비

작성 시각: `2026-06-03 02:33:00 KST`

Mission1 이후 Mission2 준비 단계에서 UAV exploration start 이전에 현재 위치 기준 상승이 먼저 되도록 FSM과 UAV path 준비를 조정했다.

![Mission1 이후 Mission2 준비 이륙 확인](docs/images/mission1_takeoff_ready_2026-06-03.png)

이번 변경으로 `asp_mission_manager` FSM에 UAV exploration 준비 상태를 추가했다.

```text
UAV_TAKEOFF_READY
UAV_TAKEOFF_REQUESTED
UAV_EXPLORATION_READY
```

Mission2 start 직후 바로 CSV 첫 waypoint로 가지 않고, 먼저 현재 위치 기준 takeoff climb를 수행한 뒤 exploration으로 넘어가도록 했다.

최종 변경 파일은 다음과 같다.

```text
src/asp_mission_manager/asp_mission_manager/mission_manager_node.py
src/asp_uav_control/asp_uav_control/uav_exploration_node.py
```

---

## 변경 기록: Mission2 Trigger FSM 추가

작성 시각: `2026-06-03 01:11:52 KST`

Mission2 Trigger의 주 publisher를 `mission_manager_node`로 두는 FSM을 추가했다. UGV path follower가 Mission1 종료 지점에
도착하면 `/ugv/mission_event`에 `MISSION2_START_REACHED`를 publish하고, mission manager는 이를 받아 Mission2와 Mission3를
병렬로 시작한다.

RViz의 Marker Detected display topic은 다음으로 변경했다.

```text
/perception/uav/marker_detections
```

기존 UAV/UGV 제어 코드, keyboard node, bridge launch, PX4/default.sdf는 수정하지 않았다.

---

## 변경 기록: Mission1 UGV Path Follower 추가

작성 시각: `2026-06-03 00:12:53 KST`

Mission1 구현 시작을 위해 새 패키지 `asp_ugv_control`을 추가했다. 목표는 UGV `X1_asp`가 `mission.csv` waypoint를 저속으로 따라가고,
`mission_type=2` waypoint에 도착하면 Mission2 시작 지점으로 판단하여 정지하는 것이다.

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
Pure pursuit에 가까운 heading 제어
waypoint별 target_speed 적용
도착 허용 반경 내 waypoint advance
mission_type=2에서 stop and event publish
```

기존 `utilities_pkg`, UAV keyboard node, UGV keyboard node, `offboard_control.cpp`, `gazebo_env_setup` launch/config, PX4/default.sdf는 수정하지 않았다.

---

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

---

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
