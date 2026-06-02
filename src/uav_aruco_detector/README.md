

````markdown
# UAV ArUco Detector with Map Coordinate Output

이 패키지는 UAV Subsystem의 ArUco Detector 역할을 수행한다.

UAV 카메라 영상에서 ArUco Marker를 검출하고, 검출된 marker ID와 pose를 발행한다.  
또한 TF를 이용하여 카메라 기준 좌표를 map 기준 좌표로 변환하고, 검출 결과를 CSV로 저장한다.

---

## 담당 기능

```text
Detect ArUco marker and save its position with ID
````

현재 확인된 기능은 다음과 같다.

```text
1. UAV 카메라 영상 수신
2. ArUco Marker ID 검출
3. 카메라 기준 marker pose 발행
4. map 기준 marker pose 발행
5. marker ID와 좌표 CSV 저장
```

---

## Publish

| Topic                          | Type                            | Description          |
| ------------------------------ | ------------------------------- | -------------------- |
| `/aruco/marker_id`             | `std_msgs/msg/Int32`            | 검출된 marker ID        |
| `/aruco/marker_pose`           | `geometry_msgs/msg/PoseStamped` | 카메라 기준 마커 상대 pose    |
| `/aruco/marker_pose_map`       | `geometry_msgs/msg/PoseStamped` | map 기준 마커 절대 pose    |
| `/offboard_control/image_proc` | `sensor_msgs/msg/Image`         | ArUco 검출 결과가 표시된 이미지 |

---

## Subscribe

| Topic                                                                           | Type                         | Description |
| ------------------------------------------------------------------------------- | ---------------------------- | ----------- |
| `/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image`       | `sensor_msgs/msg/Image`      | UAV 카메라 영상  |
| `/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 카메라 내부 파라미터 |
| `/mission_state`                                                                | `std_msgs/msg/Int32`         | 검출 활성화 상태   |

---

## Build

```bash
cd ~/asp_project/asp_ws
colcon build --packages-select uav_aruco_detector
source install/setup.bash
```

---

## Run

```bash
ros2 launch uav_aruco_detector aruco_detector.launch.py
```

---

## Activate Detection

ArUco Detector가 mission state 조건을 사용할 경우 다음 명령을 실행한다.

```bash
ros2 topic pub /mission_state std_msgs/msg/Int32 "{data: 1}"
```

---

## Camera Bridge

카메라 image와 camera_info가 ROS2로 들어와야 ArUco 검출이 가능하다.

```bash
source /opt/ros/humble/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

카메라 토픽 확인:

```bash
ros2 topic info /world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image
```

정상이라면 다음처럼 표시된다.

```text
Publisher count: 1
```

---

## TF 확인

`/aruco/marker_pose_map`이 나오려면 camera frame에서 map으로 TF 변환이 가능해야 한다.

현재 최종적으로 사용한 camera frame은 다음과 같다.

```text
x500_gimbal_0/camera_link
```

따라서 TF 확인 명령은 아래와 같다.

```bash
ros2 run tf2_ros tf2_echo map x500_gimbal_0/base_link
ros2 run tf2_ros tf2_echo map x500_gimbal_0/camera_link
```

주의: 아래 frame은 사용하지 않는다.

```text
x500_gimbal_0/camera_link/camera
```

이 frame은 TF tree에 없어서 map 변환이 실패했었다.

---

## Check Output

마커가 카메라 화면에 보이는 상태에서 확인한다.

```bash
ros2 topic echo /aruco/marker_id --once
ros2 topic echo /aruco/marker_pose --once
ros2 topic echo /aruco/marker_pose_map --once
```

정상 출력 예시:

```yaml
data: 0
```

```yaml
header:
  frame_id: x500_gimbal_0/camera_link
pose:
  position:
    x: -4.026
    y: 1.054
    z: -1.778
```

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

## CSV Log

검출 결과는 CSV로 저장된다.

기본 저장 위치:

```text
aruco_log/marker_detections.csv
```

형식:

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

CSV 확인:

```bash
cd ~/asp_project/asp_ws
find . -name "marker_detections.csv"
cat <찾은_경로>
```

---

## 실행 순서 요약

### Terminal 1 — PX4 / Gazebo

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_gimbal
```

### Terminal 2 — TF / Pose Bridge

```bash
cd ~/asp_project/asp_ws
source install/setup.bash
ros2 launch gazebo_env_setup turn_interfaces.launch.py
```

### Terminal 3 — Camera Bridge

```bash
source /opt/ros/humble/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

### Terminal 4 — ArUco Detector

```bash
cd ~/asp_project/asp_ws
source install/setup.bash
ros2 launch uav_aruco_detector aruco_detector.launch.py
```

### Terminal 5 — Mission State

```bash
cd ~/asp_project/asp_ws
source install/setup.bash
ros2 topic pub /mission_state std_msgs/msg/Int32 "{data: 1}"
```

### Terminal 6 — Result Check

```bash
ros2 topic echo /aruco/marker_id --once
ros2 topic echo /aruco/marker_pose --once
ros2 topic echo /aruco/marker_pose_map --once
```

---

## Troubleshooting

### `/aruco/marker_pose`는 나오는데 `/aruco/marker_pose_map`이 안 나오는 경우

먼저 `/aruco/marker_pose`의 frame을 확인한다.

```bash
ros2 topic echo /aruco/marker_pose --once
```

정상 frame:

```text
x500_gimbal_0/camera_link
```

잘못된 frame:

```text
x500_gimbal_0/camera_link/camera
```

잘못된 frame이 나오면 `aruco_detector_node.cpp`에서 camera_info의 frame_id가 `camera_frame_id_`를 덮어쓰지 않도록 수정해야 한다.

---

### Extrapolation into the past 오류

예시:

```text
Lookup would require extrapolation into the past
```

원인:

```text
camera image stamp와 TF stamp의 시간 기준이 다름
```

해결:

```text
marker pose의 header.stamp를 image stamp 대신 this->now()로 설정한다.
```

---

## 최종 성공 결과

현재 테스트에서 다음 결과를 확인하였다.

```text
Marker ID 검출: 성공
Camera 기준 좌표 생성: 성공
Map 기준 좌표 변환: 성공
CSV 저장: 성공
```

확인된 예시:

```text
marker_id: 0
camera_frame: x500_gimbal_0/camera_link
camera_position: x=-4.026, y=1.054, z=-1.778
map_frame: map
map_position: x=-96.782, y=101.302, z=14.407
```

```

