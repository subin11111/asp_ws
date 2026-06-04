# Mission3 UGV Rendezvous

## 목표

Mission2 시작과 동시에 UGV가 rendezvous point로 이동한다. Mission1 path follower는
`MISSION2_START_REACHED` 이후 `/command/ugv_cmd_vel`에 zero command를 계속 publish하지 않고,
`ugv_rendezvous_node`가 rendezvous 동안 command를 주도한다.

## Command Policy

```text
Mission1 ugv_path_follower_node
  -> /ugv/mission_event: MISSION2_START_REACHED
  -> zero Twist burst
  -> /ugv/mission_event: MISSION1_CMD_RELEASED
  -> release /command/ugv_cmd_vel

mission_manager_node
  -> /ugv/rendezvous_start true

ugv_rendezvous_node
  -> /ugv/rendezvous_state: RENDEZVOUS_START_DELAY
  -> /ugv/mission_event: RENDEZVOUS_START_DELAY_COMPLETE
  -> /command/ugv_cmd_vel
  -> /ugv/rendezvous_reached true
  -> /ugv/mission_event: RENDEZVOUS_REACHED
```

Mission1 follower 관련 기본값:

```yaml
zero_publish_after_stop_count: 5
disable_cmd_after_stop: true
```

Rendezvous 시작 대기 기본값:

```yaml
start_delay_sec: 1.5
```

이 delay는 UGV rendezvous node에만 적용된다. `/ugv/rendezvous_start true`는 Mission2 start와
동시에 publish되지만, UGV는 `RENDEZVOUS_START_DELAY` 상태에서 zero Twist로 대기한 뒤
Mission3 경로 추종을 시작한다. UAV exploration start는 이 delay의 영향을 받지 않는다.

Rendezvous 관련 기본값:

```yaml
lookahead_distance: 3.5
max_linear_speed: 1.4
cruise_speed: 1.1
min_linear_speed: 0.35
max_angular_speed: 1.4
stuck_timeout_sec: 20.0
progress_epsilon_m: 0.4
```

## Path

Runtime path:

```text
src/asp_ugv_control/path/mission3_rendezvous_senior.csv
```

이 경로는 Mission3 runtime용으로 고정한 경로이며 final rendezvous point는
`(-57.949, 101.780)`이다. CSV target speed는 cruise 기준인 `1.1m/s`로 맞춘다.
node는 `lookahead_distance` 안에 들어온 중간 waypoint를 skip할 수 있으므로, 촘촘한 row가 있어도
불필요하게 감속하지 않는다.

## Diagnostics

```bash
bash tools/diagnostics/ugv_cmd_source_audit.sh
bash tools/diagnostics/mission_flow_audit.sh
```

확인 기준:

```text
/command/ugv_cmd_vel publisher 목록 확인
/ugv/state: STOPPED after Mission2 start
/ugv/mission_event: MISSION1_CMD_RELEASED
/ugv/rendezvous_start: true
/ugv/rendezvous_state: RENDEZVOUS_START_DELAY
/ugv/mission_event: RENDEZVOUS_START_DELAY_COMPLETE
/command/ugv_cmd_vel linear.x ~= 0.8..1.2 during rendezvous
/ugv/rendezvous_reached: true at final point
```
