# Mission Manager FSM

## 목표

Mission2 Trigger의 주 publisher를 `mission_manager_node`로 둔다. UGV path follower는 Mission1
종료 지점에 도착하면 `/ugv/mission_event`에 `MISSION2_START_REACHED`를 publish한다.
mission manager는 이를 받아 Mission2 UAV exploration과 Mission3 UGV rendezvous를 동시에
시작하고, 두 작업이 모두 완료된 뒤 Mission4 precision landing을 시작한다.

## 전체 구조

```text
ugv_path_follower_node
  -> /ugv/mission_event: MISSION2_START_REACHED
  -> mission_manager_node
  -> /mission/mission2_trigger: true
  -> /uav/exploration_start: true
  -> /ugv/rendezvous_start: true
  -> /mission/uav_exploration_complete: true
  -> /ugv/rendezvous_reached: true
  -> /mission/precision_landing_start: true
  -> /status/landing_complete: true
  -> /mission/mission_complete: true
```

## FSM 상태

```text
INIT
READY
MISSION1_RUNNING
MISSION2_TRIGGERED
UAV_TAKEOFF_READY
UAV_TAKEOFF_REQUESTED
UAV_EXPLORATION_READY
UAV_EXPLORATION_RUNNING
MISSION2_UAV_EXPLORATION_AND_UGV_RENDEZVOUS
UAV_EXPLORATION_COMPLETE
UGV_RENDEZVOUS_READY
UGV_RENDEZVOUS_RUNNING
UGV_RENDEZVOUS_COMPLETE
PRECISION_LANDING_READY
PRECISION_LANDING_RUNNING
MISSION_COMPLETE
MISSION_ABORTED
```

## 주요 전이

* node 시작 후 첫 timer에서 `INIT -> READY`
* `auto_start=true`이면 `READY -> MISSION1_RUNNING`
* `/mission/start true` 수신 시 `READY -> MISSION1_RUNNING`
* `/ugv/mission_event`가 `MISSION2_START_REACHED`이면 Mission2 trigger를 한 번 publish한다.
* `start_uav_exploration_on_mission2_start=true`이면 같은 callback에서 `/uav/exploration_start true`를 한 번 publish한다.
* `start_rendezvous_on_mission2_start=true`이면 같은 callback에서 `/ugv/rendezvous_start true`를 한 번 publish한다.
* 위 두 값이 모두 true이면 `MISSION2_UAV_EXPLORATION_AND_UGV_RENDEZVOUS`로 전환하고 `PARALLEL_MISSION2_3_STARTED`를 기록한다.
* `/mission/uav_exploration_complete true` 수신 시 `uav_exploration_complete=true`로 기록한다.
* `/ugv/rendezvous_reached true` 또는 `/ugv/mission_event: RENDEZVOUS_REACHED` 수신 시 `rendezvous_reached=true`로 기록한다.
* `wait_both_uav_and_ugv_before_landing=true`이면 `uav_exploration_complete`와 `rendezvous_reached`가 모두 true가 될 때까지 `/mission/precision_landing_start`를 publish하지 않는다.
* 두 완료 조건이 모두 충족되면 `/mission/precision_landing_start true`를 한 번 publish하고 `PRECISION_LANDING_RUNNING`으로 전환한다.
* `/status/landing_complete true` 수신 시 `MISSION_COMPLETE`로 전환하고 `/mission/mission_complete true`를 publish
* `/mission/abort true` 수신 시 `MISSION_ABORTED`로 전환
* `/mission/reset true` 수신 시 trigger flag를 초기화하고 `READY`로 전환

## 병렬 Mission2/Mission3 정책

기본값:

```yaml
start_uav_exploration_on_mission2_start: true
start_rendezvous_on_mission2_start: true
wait_both_uav_and_ugv_before_landing: true
```

정상 시작 sequence:

```text
/ugv/mission_event: MISSION2_START_REACHED
  -> /mission/mission2_trigger true
  -> /uav/exploration_start true
  -> /ugv/rendezvous_start true
  -> /mission/state: MISSION2_UAV_EXPLORATION_AND_UGV_RENDEZVOUS
  -> /mission/status.last_manager_event: PARALLEL_MISSION2_3_STARTED
```

착륙 시작 조건:

```text
/mission/uav_exploration_complete true
/ugv/rendezvous_reached true
  -> /mission/precision_landing_start true
```

## topic 목록

Subscriptions:

```text
/mission/start
/mission/reset
/mission/abort
/ugv/state
/ugv/mission_event
/perception/mission2_trigger
/mission/uav_exploration_complete
/ugv/rendezvous_reached
/status/landing_complete
```

Publishers:

```text
/mission/state
/mission/status
/mission/mission2_trigger
/command/takeoff
/uav/exploration_start
/ugv/rendezvous_start
/mission/precision_landing_start
/mission/mission_complete
```

## perception trigger 처리

`/perception/mission2_trigger`는 현재 FSM 전이를 직접 일으키지 않는다. 이 값은 status에 기록만 하며, Mission2 진입 기준은 UGV 위치 도착 event이다.

## UAV exploration 연결

`/uav/exploration_start`와 `/mission/uav_exploration_complete`를 통해 waypoint 기반 UAV
exploration node와 연결한다. Mission3 rendezvous는 UAV exploration 완료를 기다리지 않고
`MISSION2_START_REACHED`에서 함께 시작한다. Mission4 precision landing은 UAV exploration
완료와 UGV rendezvous 도착을 모두 확인한 뒤 시작한다. 각 trigger는 한 번만 publish된다.

## 실행

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch asp_mission_manager mission_manager.launch.py
ros2 launch asp_mission_manager full_mission.launch.py
```

## 검증

```bash
ros2 topic echo /mission/state
ros2 topic echo /mission/status
ros2 topic echo /mission/mission2_trigger
ros2 topic echo /uav/exploration_start
ros2 topic echo /ugv/rendezvous_start
ros2 topic echo /mission/precision_landing_start
ros2 topic pub --once /ugv/mission_event std_msgs/msg/String "{data: MISSION2_START_REACHED}"
```

정상 기준:

```text
/mission/mission2_trigger: true
/uav/exploration_start: true
/ugv/rendezvous_start: true
/mission/state: MISSION2_UAV_EXPLORATION_AND_UGV_RENDEZVOUS
/mission/status.last_manager_event: PARALLEL_MISSION2_3_STARTED
```
