# UAV ArUco Detector with Map Coordinate Output

이 패키지는 UAV Subsystem의 ArUco Detector 역할을 수행한다.

## Publish

| Topic | Type | Description |
|---|---|---|
| `/aruco/marker_pose` | `geometry_msgs/msg/PoseStamped` | 카메라 기준 마커 상대 pose |
| `/aruco/marker_pose_map` | `geometry_msgs/msg/PoseStamped` | map 기준 마커 절대 pose |
| `/aruco/marker_id` | `std_msgs/msg/Int32` | 검출된 marker ID |

## Build

```bash
cd ~/asp_project/asp_ws
colcon build --packages-select uav_aruco_detector
source install/setup.bash
```

## Run

```bash
ros2 launch uav_aruco_detector aruco_detector.launch.py
```

## Activate Detection

```bash
ros2 topic pub /mission_state std_msgs/msg/Int32 "{data: 1}"
```

## Check Output

```bash
ros2 topic echo /aruco/marker_pose
ros2 topic echo /aruco/marker_pose_map
ros2 topic echo /aruco/marker_id
```

## TF 확인

`/aruco/marker_pose_map`이 나오려면 camera frame에서 map으로 TF 변환이 가능해야 한다.

```bash
ros2 run tf2_ros tf2_echo map x500_gimbal_0/camera_link/camera
```

## CSV Log

기본 저장 위치:

```text
aruco_log/marker_detections.csv
```

형식:

```csv
time_sec,marker_id,camera_frame,camera_x,camera_y,camera_z,map_frame,map_x,map_y,map_z
```
