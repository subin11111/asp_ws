아래 그대로 복붙해서 `README.md`에 넣으면 돼.

````markdown
# UAV ArUco Marker Detection

## 담당자

**이승윤 — UAV ArUco Marker Detection 담당**

본 패키지는 UAV 카메라 영상을 기반으로 ArUco Marker를 검출하고, 검출된 마커의 ID와 위치 좌표를 계산하여 저장하는 기능을 수행한다.

Mission 2 - UAV에서 담당하는 핵심 기능은 다음과 같다.

```text
Detect ArUco marker and save its position with ID
````

---

## 최종 동작 결과

현재 다음 기능이 정상 동작함을 확인하였다.

```text
1. UAV 카메라 영상 수신
2. ArUco Marker ID 검출
3. 카메라 기준 marker pose 계산
4. TF를 이용한 map 기준 marker pose 변환
5. marker ID와 좌표 CSV 저장
```

확인된 예시 결과:

```text
marker_id: 0

camera_frame: x500_gimbal_0/camera_link
camera_position:
  x: -4.026
  y:  1.054
  z: -1.778

map_frame: map
map_position:
  x: -96.782
  y: 101.302
  z: 14.407
```

---

## ROS2 Topics

### Subscribe

| Topic                                                                           | Type                         | Description |
| ------------------------------------------------------------------------------- | ---------------------------- | ----------- |
| `/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image`       | `sensor_msgs/msg/Image`      | UAV 카메라 영상  |
| `/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 카메라 내부 파라미터 |
| `/mission_state`                                                                | `std_msgs/msg/Int32`         | 미션 상태       |

### Publish

| Topic                          | Type                            | Description          |
| ------------------------------ | ------------------------------- | -------------------- |
| `/aruco/marker_id`             | `std_msgs/msg/Int32`            | 검출된 ArUco Marker ID  |
| `/aruco/marker_pose`           | `geometry_msgs/msg/PoseStamped` | 카메라 기준 marker pose   |
| `/aruco/marker_pose_map`       | `geometry_msgs/msg/PoseStamped` | map 기준 marker pose   |
| `/offboard_control/image_proc` | `sensor_msgs/msg/Image`         | ArUco 검출 결과가 표시된 이미지 |

---

## Coordinate Flow

ArUco Marker 좌표 생성 흐름은 다음과 같다.

```text
UAV Camera Image
        ↓
OpenCV ArUco Detection
        ↓
Marker ID / Corner 검출
        ↓
estimatePoseSingleMarkers()
        ↓
Camera 기준 Pose 계산
        ↓
/aruco/marker_pose 발행
        ↓
TF 변환
x500_gimbal_0/camera_link → map
        ↓
/aruco/marker_pose_map 발행
        ↓
marker_detections.csv 저장
```

---

## TF 구조

현재 TF는 다음 구조를 기준으로 사용한다.

```text
map
└── x500_gimbal_0/base_link
    └── x500_gimbal_0/camera_link
```

ArUco Detector의 `camera_frame_id`는 아래 frame으로 맞춘다.

```text
x500_gimbal_0/camera_link
```

TF 확인 명령:

```bash
ros2 run tf2_ros tf2_echo map x500_gimbal_0/base_link
ros2 run tf2_ros tf2_echo map x500_gimbal_0/camera_link
```

---

## 실행 순서

총 6개 터미널을 사용한다.

---

### Terminal 1 — PX4 / Gazebo 실행

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_gimbal
```

Gazebo에서 `x500_gimbal_0` 모델이 실행되어야 한다.

---

### Terminal 2 — TF / Pose Bridge 실행

```bash
cd ~/asp_project/asp_ws
source install/setup.bash
ros2 launch gazebo_env_setup turn_interfaces.launch.py
```

이 launch는 Gazebo pose 정보를 ROS2 TF로 변환한다.

확인해야 하는 TF:

```bash
ros2 run tf2_ros tf2_echo map x500_gimbal_0/base_link
ros2 run tf2_ros tf2_echo map x500_gimbal_0/camera_link
```

---

### Terminal 3 — Camera Bridge 실행

```bash
source /opt/ros/humble/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

카메라 토픽 publisher 확인:

```bash
ros2 topic info /world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image
```

정상 출력 예시:

```text
Publisher count: 1
```

---

### Terminal 4 — ArUco Detector 실행

```bash
cd ~/asp_project/asp_ws
source install/setup.bash
ros2 launch uav_aruco_detector aruco_detector.launch.py
```

---

### Terminal 5 — Mission State 설정

ArUco Detector가 mission state 조건을 사용할 경우 다음 명령을 실행한다.

```bash
cd ~/asp_project/asp_ws
source install/setup.bash
ros2 topic pub /mission_state std_msgs/msg/Int32 "{data: 1}"
```

`mission_state = 1`은 Precision Landing 또는 Marker Detection 활성 상태로 사용한다.

---

### Terminal 6 — 카메라 화면 확인

```bash
ros2 run rqt_image_view rqt_image_view
```

선택할 카메라 토픽:

```text
/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image
```

ArUco 검출 처리 이미지가 발행되는 경우 아래 토픽도 확인할 수 있다.

```text
/offboard_control/image_proc
```

---

## 결과 확인

마커가 카메라 화면에 보이는 상태에서 다음 명령으로 결과를 확인한다.

### Marker ID 확인

```bash
ros2 topic echo /aruco/marker_id --once
```

예시:

```yaml
data: 0
```

---

### Camera 기준 Pose 확인

```bash
ros2 topic echo /aruco/marker_pose --once
```

예시:

```yaml
header:
  frame_id: x500_gimbal_0/camera_link
pose:
  position:
    x: -4.026
    y: 1.054
    z: -1.778
```

---

### Map 기준 Pose 확인

```bash
ros2 topic echo /aruco/marker_pose_map --once
```

예시:

```yaml
header:
  frame_id: map
pose:
  position:
    x: -96.782
    y: 101.302
    z: 14.407
```

---

## CSV 저장 결과

검출 결과는 CSV로 저장된다.

CSV 형식:

```csv
time_sec,marker_id,camera_frame,camera_x,camera_y,camera_z,map_frame,map_x,map_y,map_z
```

예시:

```csv
1.78041e+09,0,x500_gimbal_0/camera_link,-4.02607,1.05398,-1.77837,map,-96.7816,101.302,14.4069
```

의미:

```text
marker_id = 0

camera 기준 좌표:
x = -4.02607
y =  1.05398
z = -1.77837

map 기준 좌표:
x = -96.7816
y = 101.302
z = 14.4069
```

CSV 확인 명령:

```bash
cd ~/asp_project/asp_ws
find . -name "marker_detections.csv"
cat <찾은_경로>
```

---

## Troubleshooting

### 1. `/aruco/marker_pose`는 나오는데 `/aruco/marker_pose_map`이 안 나오는 경우

TF frame이 맞지 않을 가능성이 높다.

확인:

```bash
ros2 run tf2_ros tf2_echo map x500_gimbal_0/camera_link
```

또한 `/aruco/marker_pose`의 frame이 아래처럼 나와야 한다.

```text
x500_gimbal_0/camera_link
```

만약 아래처럼 나오면 map 변환이 실패할 수 있다.

```text
x500_gimbal_0/camera_link/camera
```

---

### 2. Extrapolation into the past 오류

예시 오류:

```text
Lookup would require extrapolation into the past
```

원인:

```text
camera image stamp와 TF stamp의 시간 기준이 다름
```

해결:

```text
marker pose의 header.stamp를 image stamp 대신 현재 ROS 시간 this->now()로 설정
```

---

### 3. rqt_image_view에서 카메라 화면이 안 나오는 경우

카메라 bridge가 켜져 있는지 확인한다.

```bash
ros2 topic info /world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image
```

정상:

```text
Publisher count: 1
```

`Publisher count: 0`이면 Camera Bridge를 다시 실행한다.

---

## 최종 성공 기준

아래 네 가지가 모두 확인되면 ArUco Detection 파트 완료이다.

```text
1. /aruco/marker_id 출력 성공
2. /aruco/marker_pose 출력 성공
3. /aruco/marker_pose_map 출력 성공
4. marker_detections.csv 저장 성공
```

현재 테스트 결과:

```text
Marker ID 검출: 성공
Camera 기준 좌표 생성: 성공
Map 기준 좌표 변환: 성공
CSV 저장: 성공
```

---

## 팀원 공유용 요약

```text
ArUco Marker Detection 정상 동작 확인했습니다.

현재 UAV 카메라 영상에서 marker ID를 검출하고,
/aruco/marker_id,
/aruco/marker_pose,
/aruco/marker_pose_map
토픽으로 발행됩니다.

확인된 예시:
marker_id: 0
camera_frame: x500_gimbal_0/camera_link
camera_position: x=-4.026, y=1.054, z=-1.778
map_frame: map
map_position: x=-96.782, y=101.302, z=14.407

검출 결과는 marker_detections.csv에도 저장됩니다.
```

---

## 발표용 설명

본인은 Mission 2의 UAV Exploration 단계에서 ArUco Marker Detection을 담당하였다.
UAV 카메라 영상에서 ArUco Marker를 검출하고, marker ID와 pose를 계산하였다.
계산된 pose는 카메라 기준 좌표로 `/aruco/marker_pose`에 발행하고, TF를 이용해 map 기준 좌표로 변환하여 `/aruco/marker_pose_map`에 발행하였다.
또한 검출된 marker ID와 위치 좌표를 CSV 파일로 저장하여 탐색 결과로 활용할 수 있도록 구현하였다.

```
```
