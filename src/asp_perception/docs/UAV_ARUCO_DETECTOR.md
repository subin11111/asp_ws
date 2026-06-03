# UAV ArUco Detector

## 목표

UAV camera image에서 ArUco marker를 검출하고 marker ID, annotated image, 가능하면 map frame 기준 marker 위치를 publish 및 CSV로 저장한다.

## 입력 topic

```text
/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image
/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info
```

## 출력 topic

```text
/perception/uav/aruco/annotated
/perception/uav/marker_id
/perception/uav/marker_detections
/perception/uav/marker_map_points
```

## CSV log

```text
/home/subin/ros2_ws/mission_logs/uav_marker_detections.csv
```

형식:

```csv
stamp,marker_id,source,camera_x,camera_y,camera_z,map_x,map_y,map_z
```

## map 좌표 변환

camera_info가 들어오면 OpenCV ArUco pose estimation으로 marker의 camera frame 좌표를 계산한다. 이후 `camera_frame -> map_frame` TF가 있으면 map 좌표로 변환해 topic과 CSV에 기록한다.

pose estimation 또는 TF 변환이 실패해도 marker ID, pixel center, annotated image,
`marker_detections` publish는 계속 수행한다. 이 경우 JSON에는 `has_map=false`가 들어가고
map 좌표는 `null`로 publish된다. CSV 기록에서만 duplicate suppression을 적용하며, topic
publish 자체는 매 검출 frame 기준으로 유지한다.

## marker_detections 형식

`/perception/uav/marker_detections`는 `marker_id` key를 포함한 구조화된 문자열로 publish한다.

```json
{
  "marker_id": 14,
  "source": "uav_camera",
  "stamp": 123.45,
  "center_u": 320.0,
  "center_v": 240.0,
  "image_width": 640,
  "image_height": 480,
  "camera_x": 0.12,
  "camera_y": -0.03,
  "camera_z": 4.5,
  "map_x": -97.2,
  "map_y": 67.4,
  "map_z": 15.0,
  "has_map": true
}
```

`asp_uav_control`의 exploration node는 `marker_id` 또는 `id` key만 marker ID로 파싱한다.
따라서 좌표, 시간, stamp에 포함된 숫자는 marker ID로 취급하지 않는다.

## RViz 확인

```bash
rviz2 -d ~/ros2_ws/src/utilities_pkg/gazebo_env_setup/config/asp_final_proj.rviz
ros2 topic hz /perception/uav/aruco/annotated
ros2 topic echo /perception/uav/marker_detections
```

## Mission2 Trigger와의 관계

이 detector는 `/mission/mission2_trigger`를 publish하지 않는다. Mission2 진입 판단은 mission manager와 UGV 위치 도착 event가 담당한다.
