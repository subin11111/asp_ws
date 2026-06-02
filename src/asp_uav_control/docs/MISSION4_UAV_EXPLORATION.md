# Mission4 UAV Exploration

## 목표

UAV가 waypoint 기반 vantage point 경로를 비행하면서 카메라가 marker 후보 영역을 충분히 관찰하도록 한다. marker 위치가 지붕이나 건물 옆면에서 조금 바뀌어도, 여러 waypoint와 hold time으로 camera FOV 안에 들어올 가능성을 높인다.

## 전체 구조

```text
mission_manager_node
  -> /uav/exploration_start
  -> uav_exploration_node
  -> /command/pose
  -> offboard_control
  -> PX4 UAV
```

`uav_exploration_node`는 launch만으로 exploration을 시작하지 않는다. 기본 설정은
`start_on_launch: false`이고, IDLE 상태에서는 `/command/pose`를 publish하지 않는다.
따라서 UAV는 아래 두 조건 중 하나가 들어온 뒤에만 waypoint 명령을 publish한다.

* `/mission/state`가 `UAV_EXPLORATION_READY`가 된다.
* 수동으로 `/uav/exploration_start true`가 publish된다.

## uav_path.csv 형식

```csv
x,y,z,yaw_deg,gimbal_pitch_deg,hold_sec,tag
-97.408805847168,67.4708023071289,13.85313296318054,0.0,-90.0,3.0,project_wp_1_action_1
```

현재 `path/uav_path.csv`는 원본 CSV
`/home/subin/Autonomous-System-Platform-final-project/uav_controller/path/uav_path.csv`를
이 패키지의 `x,y,z,yaw_deg,gimbal_pitch_deg,hold_sec,tag` 형식으로 변환한 것이다.

변환 규칙:

* 원본 `x,y,z`는 map frame waypoint로 유지했다.
* 원본 yaw 값은 degree로 변환해 `yaw_deg`에 넣었다.
* 원본 gimbal pitch 값은 `gimbal_pitch_deg`로 유지했다.
* 원본 action 값은 tag 이름에 반영했다.
* 모든 waypoint의 `hold_sec`는 초기 검증값 3.0초로 설정했다.
* yaw 또는 gimbal pitch가 없는 waypoint는 안전한 기본 관찰값으로 채웠다.

Gazebo 모델 위치, 장애물, marker 후보 위치가 달라지면 waypoint 좌표, yaw, gimbal pitch,
hold time은 실제 실행 로그를 기준으로 다시 튜닝해야 한다.

## CSV waypoint 수집 방식

Gazebo에서 UAV를 직접 조종해서 waypoint를 찍는 방식은 좁은 장애물 주변에서 어렵고 위험하다.
대신 PX4/Gazebo world와 model 파일에 spawn된 marker pose를 파악하고, marker를 볼 수 있는
roof/wall 관측 waypoint 후보를 자동 생성한다.

현재 전체 미션은 UAV가 UGV 위에 실려 Mission2 시작 지점까지 이동하는 구조이다. 따라서
UAV exploration 시작 위치는 launch 시점이 아니라 Mission2 Trigger 시점이다.
`uav_path.csv`는 전체 비행 경로가 아니라 marker scan waypoint 후보 목록으로 취급한다.
`dynamic_safe_prefix: true`이면 `uav_exploration_node`가 start signal을 받은 순간
`map -> x500_gimbal_0/base_link` TF를 읽고, runtime에서 `takeoff_climb`, `safe_altitude`,
`transition_###` waypoint를 앞에 자동으로 붙인다.

Mission1 동안 UGV와 UAV가 함께 움직이는지 아래 TF를 동시에 확인한다.

```bash
ros2 run tf2_ros tf2_echo map X1_asp/base_link
ros2 run tf2_ros tf2_echo map x500_gimbal_0/base_link
```

```bash
cd /home/subin/ros2_ws
python3 tools/path_tools/extract_marker_poses.py
python3 tools/path_tools/generate_uav_path_from_markers.py \
  --input tools/path_tools/marker_poses.csv \
  --output src/asp_uav_control/path/uav_path_generated.csv \
  --mode both
```

생성된 `uav_path_generated.csv`는 path 설계용 초기값이다. 기존 `uav_path.csv`를 바로
덮어쓰지 말고, Gazebo/RViz에서 충돌 가능성, yaw, gimbal pitch, camera FOV를 확인한 뒤
필요한 row만 반영한다.

marker pose 기반 generated path는 관측 후보일 뿐이며, 실제 안전 비행 경로가 아니다.
안전한 비행을 위해 `uav_path.csv` 앞에는 `takeoff_climb`, `safe_altitude`, `transition_###`
waypoint가 필요하다. 현재 UAV 위치를 TF로 측정한 뒤 safe path generator로 보정한다.

```bash
ros2 run tf2_ros tf2_echo map x500_gimbal_0/base_link
python3 tools/path_tools/generate_safe_uav_path.py \
  --input src/asp_uav_control/path/uav_path_generated.csv \
  --output src/asp_uav_control/path/uav_path_safe.csv \
  --start-x <현재_x> --start-y <현재_y> --start-z <현재_z>
```

실제 marker detection은 waypoint 생성 도구가 아니라 UAV camera와 ArUco detector가 수행한다.
marker 위치가 지붕 위 또는 건물 옆면에서 조금 변할 수 있으므로, roof scan과 wall scan 후보를
함께 생성해 관측 가능성을 높인다.

marker detection report는 `marker_id` 또는 `id` key만 marker ID로 사용한다.
좌표나 시간에 포함된 숫자는 marker ID로 기록하지 않는다.

## ENU/NED 좌표 확인

CSV waypoint는 `map` ENU 좌표 형식이고, `/command/pose`는 map frame `PoseStamped`이다.
PX4 `TrajectorySetpoint`는 PX4 local NED 상대좌표를 사용한다. 따라서 `offboard_control`은
map ENU 절대좌표에서 start 시점의 UAV map 위치를 origin으로 빼고, local ENU를 PX4용
NED setpoint로 변환해 `/fmu/in/trajectory_setpoint`에 publish한다.

예:

```text
target map ENU = (-120, 35, 6)
origin map ENU = (-120, 35, 1)
local ENU      = (0, 0, 5)
PX4 NED        = (0, 0, -5)
```

이 구조가 정상이어야 `takeoff_climb`이 현재 위치에서 위로만 상승하는 명령으로 해석된다.
따라서 offboard 로그의 X/Y/Z는 CSV의 map ENU 좌표와 축이 달라 보일 수 있다.

좌표 변환은 다음 debug topic으로 확인한다.

```bash
ros2 topic echo /debug/offboard/input_pose_enu
ros2 topic echo /debug/offboard/local_pose_enu
ros2 topic echo /debug/offboard/setpoint_pose_ned
ros2 topic echo /debug/offboard/frame_report
```

## topic

Subscriptions:

```text
/mission/state
/uav/exploration_start
/perception/uav/marker_detections
```

Publishers:

```text
/command/pose
/gimbal_pitch_degree
/uav/exploration_state
/uav/exploration_event
/mission/uav_exploration_complete
```

## 실행

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch asp_uav_control uav_exploration.launch.py
```

기본 launch인 `uav_exploration.launch.py`는 기본 경로 `uav_path.csv`를 사용한다.
marker pose 기반 후보 경로를 테스트하려면 generated 전용 launch를 사용한다.

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch asp_uav_control uav_exploration_generated.launch.py
```

`uav_path_generated.csv`는 최종 경로가 아니라 후보 경로이다. launch 로그에서
`UAV exploration path_csv`와 `First waypoint`를 확인해 실제 어떤 CSV가 로드됐는지 먼저 확인한다.
generated launch도 launch 직후 자동 시작되지 않으며, `/uav/exploration_start true` 또는
FSM state `UAV_EXPLORATION_READY`가 있어야 waypoint publish를 시작한다.

launch 직후에는 `/command/pose`가 publish되지 않아야 한다. FSM 연동으로 시작하려면
mission manager가 `/mission/state`에 `UAV_EXPLORATION_READY`를 publish하도록 한다.
수동 검증에서는 아래 명령으로 exploration을 시작한다.

```bash
ros2 topic pub --once /uav/exploration_start std_msgs/msg/Bool "{data: true}"
```

## 검증

```bash
ros2 topic echo /mission/state
ros2 topic echo /command/pose
ros2 topic echo /uav/exploration_state
ros2 topic echo /uav/exploration_event
ros2 topic echo /mission/uav_exploration_complete
```

launch 직후 `/uav/exploration_state`가 `IDLE`이고 `/command/pose` 출력이 없다면 자동 시작 방지 조건이 맞다.
`UAV_EXPLORATION_READY` 또는 수동 start 이후에만 waypoint pose와 gimbal pitch가 publish되어야 한다.

## 완료 기준

* `uav_exploration_node`가 `/command/pose`에 waypoint pose를 publish한다.
* waypoint 도착 후 hold time 동안 같은 pose와 gimbal pitch를 유지한다.
* 모든 waypoint를 순회하면 `/mission/uav_exploration_complete true`를 publish한다.
* launch만 실행한 상태에서는 UAV pose command를 publish하지 않는다.

## 튜닝 파라미터

```text
waypoint_tolerance
default_hold_sec
exploration_timeout_sec
minimum_unique_markers
uav_path.csv waypoint 좌표, yaw, gimbal pitch
```
