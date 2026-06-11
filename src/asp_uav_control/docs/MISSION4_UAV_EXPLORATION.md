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
* `/mission/state`가 `MISSION2_UAV_EXPLORATION_AND_UGV_RENDEZVOUS`가 된다.
* 수동으로 `/uav/exploration_start true`가 publish된다.

전체 미션에서는 `mission_manager_node`가 `/ugv/mission_event: MISSION2_START_REACHED`를
받는 순간 `/uav/exploration_start true`를 publish한다. 같은 순간 Mission3 UGV rendezvous도
시작되므로, Mission2 UAV exploration과 Mission3 UGV rendezvous는 병렬로 진행된다.

## uav_path.csv 형식

```csv
x,y,z,yaw_deg,gimbal_pitch_deg,hold_sec,tag
-97.408805847168,67.4708023071289,13.85313296318054,0.0,-90.0,3.0,project_wp_1_action_1
```

현재 `path/uav_path.csv`는 이 패키지의 `x,y,z,yaw_deg,gimbal_pitch_deg,hold_sec,tag`
형식으로 정리한 waypoint CSV이다.

Mission2 실사용 runtime path는 `path/uav_path_mission2_senior.csv`이다. 이 파일은
Mission2용으로 고정한 runtime 경로이며, marker pose 기반 generated path처럼 `wall_E/W/N/S`
후보를 촘촘하게 돌지 않는다. `uav_path_generated.csv`와 `uav_path_mission2.csv`는 marker
배치 확인과 path 설계 보조용으로만 둔다.
기본 runtime guard는 `allow_generated_path_runtime: false`,
`runtime_path_must_contain: "senior"`로 설정되어 generated/reference path가 full mission에
들어오지 않게 한다.

변환 규칙:

* 입력 `x,y,z`는 map frame waypoint로 유지했다.
* 입력 yaw 값은 degree로 변환해 `yaw_deg`에 넣었다.
* 입력 gimbal pitch 값은 `gimbal_pitch_deg`로 유지했다.
* 입력 action 값은 tag 이름에 반영했다.
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
전체 미션 runtime 기본값은 `dynamic_safe_prefix: false`이다. Mission2 start signal 이후
`uav_exploration_node`는 latch된 Mission2 origin을 확인한 뒤, 별도 강제 이륙 waypoint를 만들지
않고 senior CSV scan waypoint를 바로 시작한다.
전체 미션에서는 `/ugv/mission_event`의 `MISSION2_START_REACHED` event를 받은 순간
UAV TF를 `mission2_takeoff_origin`으로 latch한다.
`/command/pose` publish는 guard를 통과해야 하며, Mission2 origin이 없으면 차단된다.
`offboard_control`도 `/uav/mission2_takeoff_origin`을 받기 전에는 `/command/pose`를 PX4
setpoint로 변환하지 않는다.

중요 좌표계 수정:

* PX4 `TrajectorySetpoint.position`은 PX4 local NED absolute setpoint이다.
* 따라서 Mission2 map origin을 빼서 `(0, 0, -8)`을 보내면 안 된다.
* Mission2 순간의 PX4 `/fmu/out/vehicle_local_position`을 함께 anchor로 저장해야 한다.
* 변환식은 아래와 같다.

```text
delta_map_enu = target_map_enu - mission2_map_anchor_enu
delta_ned = ENU_TO_NED(delta_map_enu)
final_px4_setpoint_ned = mission2_px4_anchor_ned + delta_ned
```

첫 path waypoint에서 `final_px4_setpoint_ned`는 Mission2 순간 PX4 local anchor에
Mission2 map anchor 대비 waypoint 차이를 더한 값이어야 한다.

현재 Mission1에서는 UGV와 UAV가 함께 이동하는 것이 확인되었다. Mission1 이후 Mission2를
준비하는 UAV 이륙 동작도 현재 위치 기준 상승 단계까지 안정적으로 동작하는 것을 확인했다.
다만 WP CSV는 현재 맵과 완전히 맞지 않는 구간이 있어, 실제 장애물/marker 배치 기준으로
좌표와 yaw, gimbal pitch를 추가 보정해야 한다. ArUco marker detection 노드의 실제 연동은
아직 별도로 확인하지 않았다.

## Marker timeout and waypoint progression

marker detection은 waypoint 진행을 막지 않는다. `marker_detections_cb`는 marker ID 기록과
`MARKER_DETECTED:<id>` event publish만 담당한다. UAV는 waypoint에 도착하면 hold를 시작하고,
marker가 보이면 기록한 뒤 hold 완료까지 유지하고 다음 waypoint로 간다. marker가 보이지 않는
scan waypoint에서는 `marker_wait_timeout_sec` 이후 `MARKER_TIMEOUT_CONTINUE:<tag>` event를
publish하고 다음 waypoint로 넘어간다.

연속 waypoint가 `min_waypoint_separation`보다 가까우면 runtime load 단계에서 뒤쪽 waypoint를
제거하고 `CLOSE_WAYPOINT_REMOVED:<tag>` event를 기록한다. `ignore_yaw_for_waypoint_reached`
기본값은 true이므로 yaw가 맞지 않는다는 이유만으로 같은 waypoint 주변을 계속 돌지 않는다.

진행 관련 event는 다음과 같다.

```text
MARKER_DETECTED:<id>
MARKER_TIMEOUT_CONTINUE:<tag>
WAYPOINT_HOLD_COMPLETE:<tag>
NEXT_WAYPOINT:<index>:<tag>
CLOSE_WAYPOINT_REMOVED:<tag>
WAYPOINT_STUCK_SKIP:<tag>
EXPLORATION_TIMEOUT
```

같은 waypoint에서 진행 개선이 `waypoint_stuck_timeout_sec` 이상 없거나 hold 시간이
`max_same_waypoint_hold_sec`를 넘으면 `WAYPOINT_STUCK_SKIP:<tag>`로 강제 skip한다. 이 정책
때문에 첫 marker를 못 찾거나 marker 인식이 늦어도 전체 senior path는 계속 진행한다.

## Forced takeoff before path

Mission1 동안 UAV가 UGV와 함께 이동하므로, UAV exploration 시작 시점의 위치는 매번 달라질 수
있다. 따라서 exploration start signal이 들어오면 CSV 첫 waypoint로 바로 이동하지 않고,
Mission2 시작 event 시점에 저장한 `mission2_takeoff_origin`은 좌표 변환 anchor로만 사용하고,
별도 forced takeoff command는 publish하지 않는다.

기본 동작 순서:

```text
IDLE
  -> /ugv/mission_event: MISSION2_START_REACHED
  -> latch mission2_takeoff_origin from map -> x500_gimbal_0/base_link
  -> start signal
  -> PATH_FOLLOWING_STARTED
  -> EXPLORING
```

start signal은 `/uav/exploration_start true`, `/mission/state=UAV_EXPLORATION_READY`,
또는 `required_start_states`에 포함된 `UAV_TAKEOFF_READY`로 받을 수 있다. start 전에는
`/command/pose`를 publish하지 않는다. `require_mission2_latched_origin: true`에서는
Mission2 origin이 latch되기 전 start signal이 들어와도 `WAITING_FOR_MISSION2_ORIGIN` 상태로
머물며 `/command/pose`와 CSV waypoint publish를 시작하지 않는다. 이것은 spawn 위치,
launch 시점 위치, CSV 첫 waypoint 쪽으로 이동하지 않기 위한 의도된 안전 동작이다.

관련 파라미터:

```yaml
mission_event_topic: "/ugv/mission_event"
mission2_start_event: "MISSION2_START_REACHED"
use_mission2_latched_origin: true
require_mission2_latched_origin: true
mission2_origin_map_frame: "map"
mission2_origin_uav_frame: "x500_gimbal_0/base_link"
dynamic_safe_prefix: false
transition_altitude: 18.0
max_transition_step: 15.0
```

`/uav/mission2_takeoff_origin`은 latch된 origin을 `geometry_msgs/msg/PoseStamped`로 publish한다.
CSV 안의 `takeoff_climb`, `safe_altitude`, `transition_*` row는 전체 미션에서는 정적 spawn
위치 기준일 수 있으므로 scan waypoint 후보에서 제거한다. 전체 미션 runtime은 Mission2 origin
기준 forced prefix를 다시 만들지 않는다.

## Mission2 takeoff origin hard rule

* UAV는 spawn 위치나 launch 시점 위치에서 이륙하지 않는다.
* UAV는 반드시 `/ugv/mission_event: MISSION2_START_REACHED` 순간의 UAV TF를 Mission2 takeoff origin으로 저장한다.
* `/uav/mission2_takeoff_origin`이 publish되지 않으면 UAV exploration은 시작하지 않는다.
* `uav_exploration_node`는 기본적으로 `/uav/mission2_takeoff_origin`을 한 번만 publish한다.
* 같은 Mission2 event가 반복되면 기존 Mission2 takeoff origin을 유지하고 `MISSION2_TAKEOFF_ORIGIN_DUPLICATE_EVENT_IGNORED`를 publish한다.
* 첫 `/command/pose`는 반드시 `mission2_takeoff_origin`의 x/y와 같고 z만 증가해야 한다.
* 첫 takeoff pose가 확인되면 `FIRST_TAKEOFF_POSE_CONFIRMED` event에 `mission2_origin`, `first_takeoff_pose`, `xy_error`를 함께 기록한다.
* CSV는 takeoff 완료 후 scan waypoint 후보로만 사용한다.
* `/command/pose` publish는 guard를 통과해야 하며, 잘못된 x/y는 `POSE_BLOCKED_*` event로 차단된다.
* `offboard_control`도 `/uav/mission2_takeoff_origin` 없이 `/command/pose`를 PX4로 보내지 않는다.
* `offboard_control`은 첫 `/uav/mission2_takeoff_origin`에서 map anchor와 PX4 local anchor를 함께 latch한다.
* `allow_external_origin_reanchor: false`에서는 반복 origin message가 들어와도 `mission2_px4_anchor_ned`를 바꾸지 않는다.
* `/mission/reset true`가 들어오면 latch된 external origin anchor를 clear하고 다음 mission에서 다시 한 번 latch한다.

Mission2 origin one-shot 관련 파라미터:

```yaml
republish_mission2_origin: false
allow_external_origin_reanchor: false
clear_origin_on_mission_reset: true
external_origin_duplicate_tolerance_m: 0.2
```

런타임 확인 포인트:

```text
/uav/exploration_event: MISSION2_TAKEOFF_ORIGIN_PUBLISHED_ONCE
/uav/exploration_event: FIRST_TAKEOFF_POSE_CONFIRMED mission2_origin=(...) first_takeoff_pose=(...) xy_error=(...)
/debug/offboard/frame_report: last_origin_msg_ignored=true
/debug/offboard/frame_report: duplicate_origin_ignored_count=<n>
```

## Mission2 latch 실행 순서

필수 실행 순서:

1. PX4/Gazebo 실행
2. `turn_interfaces` 실행
3. `mission_manager` 실행
4. `uav_exploration_safe.launch.py` 실행
5. `ugv_path_follower` 실행
6. UGV가 Mission2 시작 지점에 도착
7. `/ugv/mission_event`에 `MISSION2_START_REACHED` 발생
8. `uav_exploration_node`가 이 event를 받아 Mission2 takeoff origin latch
9. `mission_manager_node`가 `/uav/exploration_start true`와 `/ugv/rendezvous_start true`를 publish
10. UAV가 latched origin의 x/y에서 먼저 상승
11. 이후 CSV scan waypoint 수행

UAV exploration node는 Mission2 event를 받아야 하므로, Mission2 event 발생 전에 실행되어
있어야 한다. Mission2 event를 놓치면 origin이 latch되지 않고,
`require_mission2_latched_origin: true`이면 UAV는 출발하지 않는다. 이것은 의도된 안전 동작이다.

Mission1 동안 UGV와 UAV가 함께 움직이는지 아래 TF를 동시에 확인한다.

```bash
ros2 run tf2_ros tf2_echo map X1_asp/base_link
ros2 run tf2_ros tf2_echo map x500_gimbal_0/base_link
```

```bash
cd /home/desktop1/ros2_ws
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
/ugv/mission_event
/uav/exploration_start
/perception/uav/marker_detections
```

Publishers:

```text
/command/pose
/gimbal_pitch_degree
/uav/exploration_state
/uav/exploration_event
/uav/mission2_takeoff_origin
/mission/uav_exploration_complete
```

## 실행

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch asp_uav_control uav_exploration.launch.py
```

## Safe takeoff CSV

기본 `uav_path.csv`는 scan waypoint 후보를 포함한다. UAV가 첫 waypoint로 바로 수평 이동하면
나무나 건물에 부딪힐 수 있으므로, safe launch 테스트에서는 `uav_path_safe.csv`를 사용한다.
`uav_path_safe.csv`는 현재 시작 위치에서 먼저 z 방향으로 상승한 뒤, safe altitude에서 첫 scan
waypoint 방향으로 단계적으로 이동한다.

다만 전체 미션 검증에서는 UAV가 Mission1 동안 UGV와 함께 이동한 뒤 Mission2 Trigger 시점의
현재 위치에서 이륙해야 한다. 최초 spawn 위치 기준으로 만들어진 정적 `uav_path_safe.csv`를
그대로 런타임에서 사용하면 안 된다. 이 파일은 정적 후보/과거 산출물로만 둔다.

`uav_exploration_mission2.launch.py`와 `uav_exploration_safe.launch.py`는
`uav_path_mission2_senior.csv`를 scan waypoint 후보로 읽고, runtime에서 현재 TF 기준 forced
takeoff와 dynamic transition을 적용한다.
`dynamic_safe_prefix: true`일 때 CSV 내부의 `takeoff_climb`, `safe_altitude`, `transition_*`
row는 scan waypoint에서 제거된다.
Mission2 이후 시작 state는 `UAV_TAKEOFF_READY` 또는 `UAV_EXPLORATION_READY`를 허용하며,
수동 테스트는 `/uav/exploration_start true`로 수행할 수 있다.

```bash
python3 tools/path_tools/generate_takeoff_safe_csv.py \
  --input src/asp_uav_control/path/uav_path.csv \
  --output src/asp_uav_control/path/uav_path_safe.csv \
  --start-x <현재_x> --start-y <현재_y> --start-z <현재_z> \
  --safe-altitude 18.0
```

기본 launch:

```bash
ros2 launch asp_uav_control uav_exploration.launch.py
```

safe launch:

```bash
ros2 launch asp_uav_control uav_exploration_safe.launch.py
```

기본 launch인 `uav_exploration.launch.py`는 기본 경로 `uav_path.csv`를 사용한다.
marker pose 기반 후보 경로를 테스트하려면 generated 전용 launch를 사용한다.

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch asp_uav_control uav_exploration_generated.launch.py
```

`uav_path_generated.csv`는 최종 경로가 아니라 후보 경로이다. launch 로그에서
`UAV exploration path_csv`와 `First waypoint`를 확인해 실제 어떤 CSV가 로드됐는지 먼저 확인한다.
generated 전용 launch는 `allow_generated_path_runtime: true`와
`runtime_path_must_contain: ""`를 명시해 debug 실행임을 구분한다.
generated 전용 launch도 launch 직후 자동 시작되지 않으며, `/uav/exploration_start true` 또는
FSM state `UAV_EXPLORATION_READY`가 있어야 waypoint publish를 시작한다.

launch 직후에는 `/command/pose`가 publish되지 않아야 한다. FSM 연동으로 시작하려면
mission manager가 `/mission/state`에 `UAV_EXPLORATION_READY`를 publish하도록 한다.
수동 검증에서는 아래 명령으로 exploration을 시작한다.

```bash
ros2 topic pub --once /uav/exploration_start std_msgs/msg/Bool "{data: true}"
```

## 검증

```bash
ros2 topic echo /ugv/mission_event
ros2 topic echo /mission/state
ros2 topic echo /command/pose
ros2 topic echo /uav/exploration_state
ros2 topic echo /uav/exploration_event
ros2 topic echo /uav/mission2_takeoff_origin
ros2 topic echo /mission/uav_exploration_complete
ros2 run tf2_ros tf2_echo map x500_gimbal_0/base_link
```

launch 직후 `/uav/exploration_state`가 `IDLE`이고 `/command/pose` 출력이 없다면 자동 시작 방지 조건이 맞다.
`MISSION2_START_REACHED` 이후 `/uav/exploration_event`에
`MISSION2_TAKEOFF_ORIGIN_LATCHED`가 출력되고, `/uav/mission2_takeoff_origin`이 Mission2 지점의
UAV 위치를 publish해야 한다. start 후 첫 `/command/pose`는 runtime path의 첫 scan waypoint이며,
offboard 변환은 Mission2 map/PX4 local anchor 기준으로 계산되어야 한다.

Mission2 origin이 latch되기 전에 수동 start를 보내면 `WAITING_FOR_MISSION2_ORIGIN` 상태가
되어야 하고 `/command/pose`가 publish되면 안 된다.

전체 런타임 검증 예:

```bash
# Terminal 1
cd ~/PX4-Autopilot_ASP
make px4_sitl gz_x500_gimbal

# Terminal 2
px4humble
source ~/ros2_ws/install/setup.bash
ros2 launch gazebo_env_setup turn_interfaces.launch.py

# Terminal 3
px4humble
source ~/ros2_ws/install/setup.bash
ros2 launch asp_mission_manager mission_manager.launch.py

# Terminal 4
px4humble
source ~/ros2_ws/install/setup.bash
ros2 launch asp_uav_control uav_exploration_safe.launch.py

# Terminal 5
px4humble
source ~/ros2_ws/install/setup.bash
ros2 launch gazebo_env_setup ugv_bridge.launch.py

# Terminal 6
px4humble
source ~/ros2_ws/install/setup.bash
ros2 launch asp_ugv_control ugv_path_follower.launch.py
```

Monitoring:

```bash
ros2 topic echo /ugv/mission_event
ros2 topic echo /uav/exploration_event
ros2 topic echo /uav/mission2_takeoff_origin
ros2 topic echo /mission/state
ros2 topic echo /command/pose
ros2 topic info -v /command/pose
ros2 topic echo /debug/offboard/frame_report
ros2 topic echo /debug/offboard/local_pose_enu
ros2 topic echo /debug/offboard/setpoint_pose_ned
ros2 topic echo /fmu/out/vehicle_local_position
ros2 topic echo /fmu/in/trajectory_setpoint
```

정상 흐름:

1. `/ugv/mission_event`: `MISSION2_START_REACHED`
2. `/uav/exploration_event`: `MISSION2_TAKEOFF_ORIGIN_LATCHED`
3. `/uav/mission2_takeoff_origin`에 Mission2 지점 UAV 위치 publish
4. `/mission/state`가 `UAV_TAKEOFF_READY` 또는 `UAV_EXPLORATION_READY`
5. 또는 수동 `/uav/exploration_start true`
6. `/uav/exploration_event`: `PATH_FOLLOWING_STARTED`
7. 첫 `/command/pose`: senior CSV 첫 scan waypoint
8. 이후 scan waypoint 이동

offboard 변환 정상 기준:

```text
origin_source=mission2_map_and_px4_local_anchor
delta_map_enu ~= first senior waypoint - mission2 origin
final_setpoint_ned == mission2_px4_anchor_ned + ENU_TO_NED(delta_map_enu)
```

문제가 발생하면 즉시 아래 진단 스크립트를 실행한다.

```bash
bash tools/diagnostics/uav_pose_source_audit.sh
```

이 로그에서 `/command/pose` publisher 목록, 첫 `/command/pose`와 Mission2 origin의 x/y 차이,
정적 safe path 또는 forbidden spawn 좌표 참조 여부를 확인한다.

## 완료 기준

* `uav_exploration_node`가 `/command/pose`에 waypoint pose를 publish한다.
* waypoint 도착 후 hold time 동안 같은 pose와 gimbal pitch를 유지한다.
* 모든 waypoint를 순회하면 `/mission/uav_exploration_complete true`를 publish한다.
* launch만 실행한 상태에서는 UAV pose command를 publish하지 않는다.

## 튜닝 파라미터

```text
waypoint_tolerance
ignore_yaw_for_waypoint_reached
default_hold_sec
marker_wait_timeout_sec
waypoint_stuck_timeout_sec
min_waypoint_separation
max_same_waypoint_hold_sec
exploration_timeout_sec
minimum_unique_markers
uav_path_mission2_senior.csv waypoint 좌표, yaw, gimbal pitch
```
