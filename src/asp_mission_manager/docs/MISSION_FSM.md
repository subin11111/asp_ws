# Mission Manager FSM

## 목표

Mission2 Trigger의 주 publisher를 `mission_manager_node`로 둔다. UGV path follower는 Mission1 종료 지점에 도착하면 `/ugv/mission_event`에 `MISSION2_START_REACHED`를 publish하고, mission manager가 이를 받아 `/mission/mission2_trigger`와 `/mission/state`를 갱신한다.

## 전체 구조

```text
ugv_path_follower_node
  -> /ugv/mission_event: MISSION2_START_REACHED
  -> mission_manager_node
  -> /mission/mission2_trigger: true
  -> /mission/state: UAV_TAKEOFF_READY
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
UAV_EXPLORATION_COMPLETE
```

## 주요 전이

* node 시작 후 첫 timer에서 `INIT -> READY`
* `auto_start=true`이면 `READY -> MISSION1_RUNNING`
* `/mission/start true` 수신 시 `READY -> MISSION1_RUNNING`
* `/ugv/mission_event`가 `MISSION2_START_REACHED`이면 Mission2 trigger를 한 번 publish하고 `UAV_TAKEOFF_READY`로 전환
* `auto_publish_takeoff=false`이면 수동 takeoff/exploration gate로 보고 `UAV_EXPLORATION_READY`까지 전환
* `auto_start_exploration=true`이면 `UAV_EXPLORATION_READY`에서 `/uav/exploration_start true`를 publish하고 `UAV_EXPLORATION_RUNNING`으로 전환
* `/mission/uav_exploration_complete true` 수신 시 `UAV_EXPLORATION_COMPLETE`로 전환
* `/mission/reset true` 수신 시 trigger flag를 초기화하고 `READY`로 전환

## topic 목록

Subscriptions:

```text
/mission/start
/mission/reset
/ugv/state
/ugv/mission_event
/perception/mission2_trigger
/mission/uav_exploration_complete
```

Publishers:

```text
/mission/state
/mission/status
/mission/mission2_trigger
/command/takeoff
/uav/exploration_start
```

## perception trigger 처리

`/perception/mission2_trigger`는 현재 FSM 전이를 직접 일으키지 않는다. 이 값은 status에 기록만 하며, Mission2 진입 기준은 UGV 위치 도착 event이다.

## UAV exploration 연결

`/uav/exploration_start`와 `/mission/uav_exploration_complete`를 통해 waypoint 기반 UAV exploration node와 연결한다. Exploration 완료 후에는 향후 Mission3 UGV rendezvous 단계로 확장할 예정이다.

## 실행

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch asp_mission_manager mission_manager.launch.py
```

## 검증

```bash
ros2 topic echo /mission/state
ros2 topic echo /mission/status
ros2 topic echo /mission/mission2_trigger
ros2 topic pub --once /ugv/mission_event std_msgs/msg/String "{data: MISSION2_START_REACHED}"
```

정상 기준:

```text
/mission/mission2_trigger: true
/mission/state: UAV_TAKEOFF_READY
```
