# Mission2 Trigger ArUco Detector

## 1. Mission2 Trigger 목표

UGV `X1_asp`의 `camera_front` 영상에서 ArUco marker ID `0`을 검출하고, Mission2 시작 trigger를 ROS2 topic으로 publish한다.

## 2. 전체 구조

```text
X1_asp camera_front image
  -> ugv_aruco_detector_node
  -> /perception/aruco/annotated
  -> /perception/marker_id
  -> /perception/marker_detections
  -> /perception/mission2_trigger
```

## 3. 구현 개념

* UGV camera image를 `rclcpp::SensorDataQoS()`로 구독한다.
* `camera_info`를 구독해 이후 marker pose estimation 확장에 사용할 수 있게 저장한다.
* OpenCV ArUco detector로 marker ID를 검출한다.
* target marker ID가 연속 threshold 이상 검출되면 perception trigger를 publish한다.
* detector 단독 테스트를 위해 기본값은 UGV mission event 없이 trigger할 수 있게 설정한다.
* `require_ugv_event=true`로 바꾸면 `/ugv/mission_event`에서 `MISSION2_START_REACHED`를 받은 뒤에만 trigger한다.
* `/mission/mission2_trigger`는 mission manager가 담당하므로 기본값에서는 publish하지 않는다.

## 4. topic 목록

Subscriptions:

```text
/world/default/model/X1_asp/link/base_link/sensor/camera_front/image
/world/default/model/X1_asp/link/base_link/sensor/camera_front/camera_info
/ugv/mission_event
```

Publishers:

```text
/perception/aruco/annotated
/perception/marker_id
/perception/marker_detections
/perception/mission2_trigger
```

## 5. parameter 목록

```text
image_topic
camera_info_topic
annotated_image_topic
marker_id_topic
marker_detections_topic
mission2_trigger_topic
mission2_manager_trigger_topic
publish_manager_trigger
ugv_mission_event_topic
required_ugv_event
require_ugv_event
target_marker_id
marker_size_m
dictionary
consecutive_detection_threshold
trigger_once
publish_debug_image
use_camera_info
```

## 6. 실행 방법

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
rviz2 -d ~/ros2_ws/src/utilities_pkg/gazebo_env_setup/config/asp_final_proj.rviz
```

Terminal 4:

```bash
px4humble
source ~/ros2_ws/install/setup.bash
ros2 launch asp_perception ugv_aruco_detector.launch.py
```

## 7. 검증 방법

```bash
ros2 topic list | grep -Ei "aruco|marker|perception|trigger"
ros2 topic hz /perception/aruco/annotated
ros2 topic echo /perception/marker_id
ros2 topic echo /perception/mission2_trigger
```

정상 topic:

```text
/perception/aruco/annotated
/perception/marker_id
/perception/marker_detections
/perception/mission2_trigger
```

ID `0` 검출 시:

```text
/perception/marker_id: 0
/perception/mission2_trigger: true
```

## 8. 완료 기준

* RViz Marker Detected display가 `/perception/aruco/annotated`를 표시한다.
* marker ID `0`이 검출되면 `/perception/marker_id`에 `0`이 publish된다.
* 연속 검출 threshold를 넘으면 `/perception/mission2_trigger`에 `true`가 publish된다.
* 제어 topic은 publish하지 않는다.

## 9. 추후 mission_manager 연결 방식

현재 Mission2 진입은 mission manager가 `/ugv/mission_event`를 받아 판단한다. perception trigger는 marker exploration 또는 precision landing 단계에서 mission manager 입력으로 확장할 수 있다.
