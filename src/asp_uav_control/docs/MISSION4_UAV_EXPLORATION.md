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
