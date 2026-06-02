# Marker Pose 기반 UAV Path 생성 도구

## 목적

Gazebo에서 UAV를 직접 조종해서 waypoint를 찍는 방식은 충돌 위험이 있고, 좁은 장애물 주변에서 반복 측정하기 어렵다. 특히 현재 waypoint CSV는 우리 map과 완전히 맞는 좌표로 검증된 상태가 아니어서 UAV가 나무나 장애물 쪽으로 이동할 수 있다.

이 도구는 PX4/Gazebo world와 model 파일에 spawn된 ArUco marker pose를 읽고, marker를 볼 가능성이 높은 roof/wall 관측 waypoint 후보를 자동 생성한다.

## 중요한 제한

이 방식은 path 설계 보조용이다. 실제 미션 중 marker 위치 파일을 읽어서 marker 검출을 대체하면 안 된다. 실제 marker 검출은 반드시 UAV camera image와 ArUco detector node가 수행해야 한다.

생성된 `uav_path_generated.csv`는 초기 후보 경로이다. 바로 기존 `uav_path.csv`를 덮어쓰지 말고, Gazebo/RViz에서 위치와 고도, yaw, gimbal pitch를 확인한 뒤 필요한 row만 `uav_path.csv`에 반영한다.

## 실행 순서

```bash
cd /home/subin/ros2_ws

python3 tools/path_tools/extract_marker_poses.py

python3 tools/path_tools/generate_uav_path_from_markers.py \
  --input tools/path_tools/marker_poses.csv \
  --output src/asp_uav_control/path/uav_path_generated.csv \
  --mode both
```

기본 추출은 world에 spawn된 marker pose를 우선 사용한다. model asset 내부의 local pose까지 확인해야 할 때만 다음 옵션을 사용한다.

```bash
python3 tools/path_tools/extract_marker_poses.py --include-model-local-poses
```

UAV 시작 위치를 알고 있을 때는 시작 상승 waypoint를 추가할 수 있다.

```bash
python3 tools/path_tools/generate_uav_path_from_markers.py \
  --input tools/path_tools/marker_poses.csv \
  --output src/asp_uav_control/path/uav_path_generated.csv \
  --mode both \
  --start-x -120.0 --start-y 35.0 --start-z 2.0
```

## marker_poses.csv 형식

```csv
name,x,y,z,roll,pitch,yaw,source_file
aruco_marker_7,-96.48,68.52,3.85,2.70,1.57,-3.14,/path/to/default.sdf
```

필드 의미:

```text
name        SDF include/model name
x,y,z       map/world 기준 marker spawn position
roll,pitch,yaw marker spawn orientation
source_file pose를 읽은 SDF/XML 파일
```

## uav_path_generated.csv 형식

```csv
x,y,z,yaw_deg,gimbal_pitch_deg,hold_sec,tag
-96.483772,68.522156,18.000000,-179.96,-90.00,4.00,aruco_marker_7_roof
```

이 형식은 `asp_uav_control`의 `uav_exploration_node`가 읽는 CSV 형식과 같다.

## Roof Marker 처리

Roof scan waypoint는 marker 위쪽에서 아래를 내려다보는 후보이다.

```text
x = marker_x
y = marker_y
z = max(marker_z + roof_view_altitude_offset_m, safe_altitude_m)
gimbal_pitch_deg = -90
```

marker가 지붕 위에 있거나 카메라가 아래를 보며 지나가야 하는 경우에 우선 확인한다.

## Wall Marker 처리

Wall scan waypoint는 marker 주변 동서남북 4방향에 후보를 만든다.

```text
distance = wall_view_distance_m
z = max(marker_z + wall_view_height_offset_m, 8.0)
yaw_deg = waypoint 위치에서 marker를 바라보는 각도
gimbal_pitch_deg = -35
```

marker가 건물 옆면이나 방 내부 벽면에 있을 수 있으므로, roof scan만으로 보이지 않는 경우를 대비한다.

## 실제 사용 전 확인

생성된 waypoint는 반드시 Gazebo와 RViz에서 확인한다.

확인할 항목:

```text
UAV가 나무, 벽, 지형, 구조물과 충돌하지 않는가
z 고도가 충분히 안전한가
yaw가 marker 방향을 향하는가
gimbal pitch가 marker 후보 영역을 카메라 FOV 안에 넣는가
waypoint 수가 너무 많아 timeout이 발생하지 않는가
```

확인 후에는 `src/asp_uav_control/path/uav_path_generated.csv`에서 필요한 row만 골라 `src/asp_uav_control/path/uav_path.csv`에 반영한다.

## Safe Path 생성

`uav_path_generated.csv`는 marker 관측 후보 경로이므로, 현재 UAV 시작 위치에서 첫 관측 waypoint까지
안전하게 이동하는 이륙/전이 구간이 부족할 수 있다. `generate_safe_uav_path.py`는 기존 후보 경로 앞에
`takeoff_climb`, `safe_altitude`, `transition_###` waypoint를 추가해 `uav_path_safe.csv`를 만든다.

먼저 현재 UAV 위치를 TF로 측정한다.

```bash
ros2 run tf2_ros tf2_echo map x500_gimbal_0/base_link
```

측정한 translation 값을 `--start-x`, `--start-y`, `--start-z`에 넣는다.

```bash
python3 tools/path_tools/generate_safe_uav_path.py \
  --input src/asp_uav_control/path/uav_path_generated.csv \
  --output src/asp_uav_control/path/uav_path_safe.csv \
  --start-x -120.0 --start-y 35.0 --start-z 2.0
```

생성된 `uav_path_safe.csv`를 Gazebo/RViz에서 확인한 뒤 필요한 경우에만 기존 경로에 반영한다.

```bash
head -n 20 src/asp_uav_control/path/uav_path_safe.csv
cp src/asp_uav_control/path/uav_path_safe.csv src/asp_uav_control/path/uav_path.csv
colcon build --packages-select asp_uav_control
source install/setup.bash
```

도구는 기본적으로 `uav_path.csv`를 덮어쓰지 않는다. `--apply` 옵션은 검토가 끝난 뒤에만 사용한다.
