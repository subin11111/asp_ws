# UAV Offboard Runtime Debug Guide

## 1. 문제 증상

PX4/Gazebo, QGroundControl, `turn_interfaces.launch.py`, `keyboard_control_node`를 실행하면 처음에는 UAV 키보드 제어가 정상 동작한다. `q` 키로 이륙도 가능하다.

하지만 `turn_interfaces.launch.py`가 켜진 뒤 일정 시간이 지나면 `q/w/s/a/d` 등 어떤 키 입력도 UAV에 반영되지 않는다. 이때 `keyboard_control_node`만 재시작해도 복구되지 않고, `turn_interfaces.launch.py`를 재시작하면 다시 정상 동작한다.

## 2. 의심 원인

* `keyboard_control_node` 문제가 아니라 `offboard_control` 또는 PX4 Offboard 상태 문제일 가능성
* `offboard_control` 내부 `armed/offboard` 상태와 PX4 실제 상태가 어긋날 가능성
* `/clock`과 `use_sim_time` 설정으로 인해 timestamp가 멈추거나 오래된 값으로 publish될 가능성
* `/fmu/in/offboard_control_mode` stream rate가 낮아져 PX4 Offboard proof-of-life 조건을 잃을 가능성

## 3. 진단 스크립트 실행 방법

```bash
cd ~/ros2_ws
bash tools/diagnostics/uav_offboard_runtime_check.sh
```

스크립트는 화면에 결과를 출력하면서 동시에 아래 형식의 로그 파일을 저장한다.

```text
tools/diagnostics/logs/uav_runtime_YYYYmmdd_HHMMSS.log
```

## 4. 언제 실행해야 하는지

* 정상 동작 직후 한 번 실행
* 키보드가 안 먹는 문제 발생 직후 한 번 실행
* 두 log 파일을 비교

최근 두 로그는 다음 명령으로 비교한다.

```bash
cd ~/ros2_ws
bash tools/diagnostics/compare_runtime_logs.sh
```

## 5. 결과 해석표

| Observation | Likely Cause |
| --- | --- |
| `/command/twist` changes, but `/fmu/in/trajectory_setpoint` does not | `offboard_control` callback/state issue |
| `/fmu/in/offboard_control_mode` below 2Hz | PX4 Offboard proof-of-life lost |
| `/clock` stopped and `use_sim_time=true` | sim time/timestamp issue |
| `vehicle_command_ack` rejected | PX4 rejected ARM/OFFBOARD command |
| relaunching `turn_interfaces.launch.py` fixes it | `offboard_control` internal state stale |

## 6. 다음 수정 후보

* `offboard_control`이 실제 `/fmu/out/vehicle_control_mode`, `/fmu/out/vehicle_status_v1`를 subscribe하도록 수정
* 내부 `armed_` flag만 믿지 않고 PX4 실제 상태를 기준으로 OFFBOARD/ARM 재시도
* `/clock` 문제가 확인되면 `use_sim_time=false` 테스트
* command watchdog 추가
