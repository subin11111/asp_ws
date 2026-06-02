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

pose estimation 또는 TF 변환이 실패해도 marker ID와 annotated image publish는 계속 수행한다. 이 경우 map 좌표는 `nan`으로 기록한다.

## RViz 확인

```bash
rviz2 -d ~/ros2_ws/src/utilities_pkg/gazebo_env_setup/config/asp_final_proj.rviz
ros2 topic hz /perception/uav/aruco/annotated
ros2 topic echo /perception/uav/marker_detections
```

## Mission2 Trigger와의 관계

이 detector는 `/mission/mission2_trigger`를 publish하지 않는다. Mission2 진입 판단은 mission manager와 UGV 위치 도착 event가 담당한다.
