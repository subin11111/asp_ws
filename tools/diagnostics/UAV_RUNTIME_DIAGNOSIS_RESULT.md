# UAV Runtime Diagnosis Result

## 1. 사용한 GOOD log 파일명

`tools/diagnostics/logs/uav_runtime_20260602_202230.log`

## 2. 사용한 BAD log 파일명

`tools/diagnostics/logs/uav_runtime_20260602_202359.log`

## 3. 관찰된 차이

* `/command/twist`
  * GOOD: publish 확인, rate 약 1.5~3.6 Hz
  * BAD: publish 확인, rate 약 5 Hz
  * 결론: keyboard 입력 topic 자체는 고장 시점에도 살아 있었다.

* `/fmu/in/trajectory_setpoint`
  * GOOD: 약 20 Hz로 계속 publish
  * BAD: 약 20 Hz로 계속 publish
  * 결론: setpoint stream 자체가 멈춘 증거는 없다.

* `/fmu/in/offboard_control_mode`
  * GOOD: 약 20 Hz
  * BAD: 약 18 Hz 측정 후 ros2 command가 status 120으로 종료
  * 결론: 2 Hz 미만 proof-of-life 상실로 보기는 어렵다.

* `/fmu/out/vehicle_control_mode`
  * GOOD: `flag_armed: true`, `flag_control_offboard_enabled: true`
  * BAD: `flag_armed: false`, `flag_control_offboard_enabled: true`
  * 결론: 고장 시점에는 PX4 실제 상태가 disarmed였다.

* `/fmu/out/vehicle_status_v1`
  * GOOD: `arming_state: 2`, `nav_state: 14`, `failsafe: false`
  * BAD: one-shot echo timeout
  * 결론: GOOD에서는 armed/offboard 상태가 명확하고, BAD에서는 상태 echo가 불안정했다.

* `/fmu/out/vehicle_command_ack`
  * GOOD/BAD 모두 one-shot echo timeout
  * 결론: 명확한 reject/denied 증거는 없다.

* `/clock` 및 `/offboard_control use_sim_time`
  * GOOD: `/offboard_control use_sim_time=true`
  * BAD: `/clock` one-shot echo에서 type 확인 실패, parameter check 시 `/offboard_control` node not found
  * 결론: clock 문제 가능성은 있으나 GOOD에서도 `/clock` hz/echo가 timeout이라 launch의 `use_sim_time`을 바꿀 만큼 명확하지 않다.

## 4. 원인 분류

주 원인 분류: PX4 OFFBOARD/ARM 상태 문제 및 `offboard_control` runtime/state 문제

근거:

* BAD 로그에서도 `/command/twist`는 publish되고 있어서 keyboard_control_node 입력 문제로 보기 어렵다.
* BAD 로그에서도 `/fmu/in/trajectory_setpoint`는 약 20 Hz로 publish되고 있어서 setpoint publisher 자체가 멈춘 문제로 보기 어렵다.
* GOOD과 BAD의 가장 결정적인 차이는 PX4 실제 상태의 `flag_armed`가 `true`에서 `false`로 바뀐 점이다.
* 기존 `offboard_control.cpp`는 OFFBOARD/ARM command를 한 번 보낸 뒤 내부 `armed_` flag를 `true`로 설정하고, 이후 PX4 실제 disarm/offboard 이탈 상태를 구독하거나 복구하지 않았다.

## 5. 적용한 수정

`src/utilities_pkg/px4_ros_com/src/examples/offboard/offboard_control.cpp`만 수정했다.

* `/fmu/out/vehicle_control_mode` subscription 추가
* `/fmu/out/vehicle_status_v1` subscription 추가
* `/fmu/out/vehicle_command_ack` subscription 추가
* PX4 실제 `flag_control_offboard_enabled`, `flag_armed`, `arming_state`, `failsafe` 상태를 내부 상태에 반영
* command ack를 throttled log로 출력하고 reject/denied/failed 결과는 WARN으로 출력
* 기존처럼 `OffboardControlMode`와 `TrajectorySetpoint`는 20 Hz로 계속 publish
* 첫 command 이후 PX4 실제 상태가 OFFBOARD가 아니거나 ARM 상태가 아니면 1초 간격으로 OFFBOARD/ARM command 재시도
* `/command/twist`, `/command/pose`, `/gimbal_pitch_degree`, `/command/disarm` topic 이름과 동작은 유지

## 6. 수정하지 않은 이유

`turn_interfaces.launch.py`는 수정하지 않았다.

BAD 로그에서 `/clock` 문제가 의심되기는 하지만 GOOD 로그에서도 `/clock` hz/echo가 timeout이라서, `/clock + use_sim_time`만을 명확한 원인으로 판정하기 어렵다. 따라서 launch의 `use_sim_time` 기본값은 바꾸지 않았다.

UGV 관련 파일, `keyboard_control_node.cpp`, `default.sdf`, `PX4-Autopilot_ASP`는 수정하지 않았다.

## 7. 재검증 명령

빌드:

```bash
cd /home/subin/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
colcon build --packages-select px4_ros_com
source install/setup.bash
```

실행 및 테스트:

```bash
cd ~/PX4-Autopilot_ASP
make px4_sitl gz_x500_gimbal
```

```bash
px4humble
source ~/ros2_ws/install/setup.bash
ros2 launch gazebo_env_setup turn_interfaces.launch.py
```

```bash
px4humble
source ~/ros2_ws/install/setup.bash
ros2 run utilities_pkg keyboard_control_node
```

정상 상태에서 `q/s/w/s/a/s/d/s`를 테스트하고, 5분 이상 기다린 뒤 같은 키 입력을 다시 테스트한다.

문제가 다시 발생하면:

```bash
cd ~/ros2_ws
bash tools/diagnostics/uav_offboard_runtime_check.sh
bash tools/diagnostics/compare_runtime_logs.sh
```

## 8. 남은 리스크

* `/clock`이 실제로 장시간 실행 후 멈추는 문제가 별도로 존재할 수 있다.
* PX4가 ARM command를 거부하는 경우에는 새 ack WARN log로 원인을 추가 확인해야 한다.
* BAD 로그에서 `/offboard_control` node가 parameter check 시점에 사라진 점은 추가 관찰이 필요하다. 같은 현상이 반복되면 node crash/log 확인이 필요하다.
