# Mission1 UGV Path Follower

## 1. Mission1 목표

Mission1의 목표는 UGV `X1_asp`가 `mission.csv` waypoint를 저속으로 따라가고, `mission_type=2` waypoint에 도착하면 Mission2 시작 지점으로 판단하여 정지하는 것이다.

## 2. 전체 구조

```text
ugv_path_follower_node
  -> /command/ugv_cmd_vel
  -> ugv_cmd_vel_bridge
  -> /model/X1_asp/cmd_vel
  -> Gazebo X1_asp
```

UGV 자동 주행 노드는 ROS2 내부 명령 topic인 `/command/ugv_cmd_vel`만 publish한다.

## 3. 구현 개념

* TF2로 `map` 기준 UGV pose를 조회한다.
* 현재 위치와 target waypoint 사이의 거리와 heading error를 계산한다.
* waypoint에 가까워지면 다음 waypoint로 넘어간다.
* target marker를 publish해서 RViz에서 현재 목표점을 볼 수 있게 한다.
* `mission_type=2` waypoint를 Mission2 연결 지점으로 해석한다.

## 4. 현재 workspace 구조에 맞춘 부분

* node name을 `ugv_path_follower_node`로 정리했다.
* output topic 기본값을 `/command/ugv_cmd_vel`로 고정했다.
* TF frame 기본값을 `map -> X1_asp/base_link`로 맞췄다.
* CSV 형식을 `x,y,mission_type,target_speed`로 확장했다.
* Mission2 시작 지점에 도착하면 UAV takeoff 명령을 보내지 않고 `/ugv/mission_event`에 `MISSION2_START_REACHED`를 publish한다.
* `/ugv/state` topic에 `WAITING_FOR_TF`, `LOADED_MISSION`, `FOLLOWING_PATH`, `WAYPOINT_REACHED`, `STOPPED` 같은 상태를 publish한다.
* 저속 Mission1 테스트를 위해 최대 속도 기본값을 0.8 m/s로 제한했다.

## 5. 직접 Gazebo cmd_vel에 publish하지 않는 이유

UGV keyboard와 자동 주행이 같은 ROS2 command path를 공유해야 bridge 구조가 단순하고 안전하다. 따라서 자동 주행 노드는 Gazebo topic에 직접 publish하지 않고 `/command/ugv_cmd_vel`만 publish한다. 실제 Gazebo topic으로의 전달은 `ugv_bridge.launch.py`가 담당한다.

## 6. mission.csv 형식

```csv
x,y,mission_type,target_speed
-120.36,36.04,1,0.5
-122.00,39.00,1,0.5
-124.00,42.00,1,0.5
-126.00,45.00,1,0.4
-127.50,47.00,2,0.0
```

필드 의미:

```text
x            map frame 기준 target x
y            map frame 기준 target y
mission_type 일반 waypoint는 1, Mission2 시작 정지 지점은 2
target_speed 해당 waypoint로 이동할 때 사용할 목표 속도
```

초기 waypoint는 `X1_asp` 시작 위치 근처의 테스트용 좌표이다. 실제 Gazebo 시작 위치가 다르면 측정한 `map -> X1_asp/base_link` 좌표로 교체해야 한다.

## 7. 실행 명령

Terminal 1:

```bash
cd ~/PX4-Autopilot_ASP
make px4_sitl gz_x500_gimbal
```

Terminal 2:

```bash
px4humble
source ~/ros2_ws/install/setup.bash
ros2 launch gazebo_env_setup turn_interfaces.launch.py
```

Terminal 3:

```bash
px4humble
source ~/ros2_ws/install/setup.bash
ros2 launch gazebo_env_setup ugv_bridge.launch.py
```

Terminal 4:

```bash
px4humble
source ~/ros2_ws/install/setup.bash
ros2 launch asp_ugv_control ugv_path_follower.launch.py
```

## 8. 검증 명령

```bash
ros2 topic echo /command/ugv_cmd_vel
ros2 topic echo /ugv/state
ros2 topic echo /ugv/mission_event
ros2 run tf2_ros tf2_echo map X1_asp/base_link
rqt_graph
```

## 9. 완료 기준

* `/command/ugv_cmd_vel`에 `geometry_msgs/msg/Twist`가 publish된다.
* `linear.x`와 `angular.z`만 제어에 사용된다.
* UGV가 waypoint를 따라 저속 이동한다.
* `mission_type=2` waypoint에 도착하면 zero Twist를 publish하고 정지한다.
* `/ugv/state`가 `STOPPED`가 된다.
* `/ugv/mission_event`에 `MISSION2_START_REACHED`가 publish된다.
