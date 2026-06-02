# gazebo_env_setup TF Fix

작성 기준: 2026-06-02 변경 기록

## 목적

RViz에서 camera image는 표시되지만 TF tree에 실제 모델 frame인 아래 frame이 보이지 않는 문제를 해결한다.

```text
X1_asp/base_link
x500_gimbal_0/base_link
```

원인:

- Gazebo pose topic이 ROS2로 bridge되지 않음
- `pose_tf_broadcaster`가 `bridge_and_tf.launch.py`에서 실행되지 않음
- 기존 static TF publisher가 `x500_depth_0` 기준으로 남아 있어 현재 모델명과 맞지 않음

현재 모델명:

```text
X1_asp
x500_gimbal_0
```

---

## 추가/수정 파일

```text
gazebo_env_setup/
├── src/
│   └── pose_tf_broadcaster.cpp
├── launch/
│   ├── bridge_and_tf.launch.py
│   └── turn_interfaces.launch.py
├── CMakeLists.txt
└── package.xml
```

---

## Bridge되는 Gazebo Pose Topics

```text
/model/X1_asp/pose
/model/X1_asp/pose_static
/model/x500_gimbal_0/pose
/model/x500_gimbal_0/pose_static
```

---

## TF 변환 규칙

`pose_tf_broadcaster`는 수신한 transform에서:

```text
parent frame == default
```

이면:

```text
parent frame = map
```

으로 바꾼다.

child frame은 Gazebo가 제공하는 이름을 그대로 유지한다.

기대 결과:

```text
map
├── X1_asp/base_link
└── x500_gimbal_0/base_link
```

---

## 적용 방법

기존 패키지 백업:

```bash
cd ~/asp_project/asp_ws/src
cp -r gazebo_env_setup gazebo_env_setup_backup
```

압축 해제 후 덮어쓰기:

```bash
unzip ~/Downloads/gazebo_env_setup_tf_fix.zip
cp -r gazebo_env_setup/* ~/asp_project/asp_ws/src/gazebo_env_setup/
```

빌드:

```bash
cd ~/asp_project/asp_ws
colcon build --packages-select gazebo_env_setup
source install/setup.bash
```

기존 launch 재시작:

```bash
ros2 launch gazebo_env_setup turn_interfaces.launch.py
```

---

## 확인 명령

UAV TF:

```bash
ros2 run tf2_ros tf2_echo map x500_gimbal_0/base_link
```

UGV TF:

```bash
ros2 run tf2_ros tf2_echo map X1_asp/base_link
```

TF tree 확인:

```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

---

## ArUco 좌표 연결

이 TF가 정상적으로 나오면 ArUco detector에서 아래 변환을 시도할 수 있다.

```text
x500_gimbal_0/camera_link/camera
→ map
```

단, `map -> x500_gimbal_0/base_link`만 있고 camera frame이 없다면,
추가로 camera link static TF가 필요하다.

확인:

```bash
ros2 run tf2_ros tf2_echo map x500_gimbal_0/camera_link/camera
```

---

## 수정하지 않은 것

- 제어 코드
- keyboard node
- UGV bridge
- image bridge topic
- PX4/default.sdf
