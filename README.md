# ASP ROS2 Workspace

## 최근 미션 시간 단축 관련 변경

Landing과 Mission1 -> Mission2 전환에서 체감 지연을 줄이기 위해 현재 워크스페이스 구조 기준으로 아래 변경을 반영했다.

- `asp_final_uav/uav_mission_node.py`
- `asp_final_px4_bridge/px4_offboard_bridge.py`
- `asp_final_mission/mission_supervisor.py`
- `asp_final_uav/config/uav_params.yaml`

### 1. Landing marker search 무한 대기 제거

- `landing_marker_search_timeout_s` 파라미터를 추가했다.
- UAV가 `pre_approach` 이후 `marker_search` 단계에서 일정 시간 안에 ArUco marker lock을 못 잡으면 무한 대기하지 않는다.
- timeout 이후에는 `UGV landing TF`가 있으면 그 좌표를, 없으면 rendezvous 좌표를 fallback landing target으로 사용하고 바로 `descend` 단계로 넘어간다.
- 기본 설정값은 `3.0초`다.

### 2. Mission1 동안 UAV offboard pre-roll 준비

- `/asp_final/mission/state == MISSION1_CARRIER` 동안 UAV 현재 pose를 `/asp_final/uav/cmd_pose`로 계속 발행하도록 추가했다.
- UAV가 아직 `idle` 상태여도 현재 TF를 기준으로 pose를 publish해서 PX4 bridge가 미리 setpoint를 받을 수 있게 했다.

### 3. PX4 bridge command gate 추가

- `/asp_final/uav/offboard_command_enable` 토픽을 추가했다.
- PX4 bridge는 Mission1 동안 들어오는 `cmd_pose`로 setpoint counter를 미리 채우고, 가능한 경우 map/PX4 anchor도 미리 latch한다.
- 다만 gate가 열리기 전에는 OFFBOARD/ARM 명령을 보내지 않도록 해서 Mission1 중 의도치 않은 이륙을 막는다.
- Mission2 시작 시에만 command gate를 열어 pre-roll 이후 바로 OFFBOARD/ARM 요청이 가능하도록 연결했다.

### 4. Landing 조기 disarm 경로 보수화

- 기존의 일반 `vehicle_land_detected.landed=true` 기반 자동 disarm 경로는 제거했다.
- 현재는 landing phase에서 `/asp_final/uav/land`가 요청된 뒤, bridge의 `fallback_touchdown_ready()` 조건을 만족할 때만 disarm 후 `/command/disarm`을 발행한다.
- `mission_timer_node` 자체는 수정하지 않고, disarm 타이밍만 bridge 쪽에서 보정했다.

### 이번에 일부러 보류한 항목

- 친구가 적용한 `landing_descent_step_m = 3.4` 같은 큰 착륙 속도 증가는 현재 구조에서 안정성 리스크가 커 보여 이번 변경에는 포함하지 않았다.
- UGV Mission1 속도 상향도 아직은 적용하지 않았다.
- 현재는 구조 변경 대비 효과가 확실한 `offboard pre-roll`, `command gate`, `landing search timeout`, `조기 disarm 방지`만 반영한 상태다.


## 실행 시간

이 환경에서 전체 기동 시간은 평균 약 `120초` 정도로 확인했다.

이 시간에는 아래 과정이 포함된다.

- 빌드 확인
- PX4 시작
- Gazebo world 초기화
- 모델 spawn
- Gazebo GUI 연결

환경 상태나 이전 캐시 상태에 따라 약간 달라질 수 있다.

## 실행 방법

다른 터미널에서 아래처럼 실행하면 된다.

```bash
cd ~/PX4-Autopilot_ASP
make px4_sitl gz_x500_gimbal
```

ROS 2 환경이 필요한 경우:

```bash
source /opt/ros/humble/setup.bash
```

## 여전히 창이 안 보일 때

실행 자체는 됐지만 창이 다른 workspace에 있거나 포커스를 잃은 경우가 있을 수 있다.

확인용 명령:

```bash
pgrep -af "gz sim|px4"
```

필요하면 GUI만 수동으로 다시 붙일 수 있다.

```bash
gz sim -g
```

## 확인된 비치명 경고

아래 경고들은 확인됐지만, 이번 문제의 직접 원인은 아니었고 실행 자체를 막지도 않았다.

- `Failed to load system plugin [libGstCameraSystem.so]`
- `libEGL warning: egl: failed to create dri2 screen`
- `Gazebo does not support Ogre material scripts`

즉, 이 경고들이 있어도 PX4와 Gazebo 기동 자체는 가능했다.

## 메모

- 이번 단계에서 ROS 패키지 구조 자체를 바꾼 것은 아니다.
- 핵심 기능 수정은 워크스페이스 내부가 아니라 PX4 실행 스크립트 쪽에서 이루어졌다.
- 목적은 `make px4_sitl gz_x500_gimbal` 명령 하나로 Gazebo GUI까지 안정적으로 올라오게 만드는 것이었다.


### 검증

아래 명령으로 문법 및 패키지 빌드를 확인했다.

```bash
python3 -m py_compile \
  src/asp_final_uav/asp_final_uav/uav_mission_node.py \
  src/asp_final_px4_bridge/asp_final_px4_bridge/px4_offboard_bridge.py \
  src/asp_final_mission/asp_final_mission/mission_supervisor.py

colcon build --packages-select asp_final_uav
colcon build --packages-select asp_final_uav asp_final_px4_bridge asp_final_mission
```
