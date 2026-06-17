# ASP ROS2 Workspace

## 실행 시간

이 환경에서 전체 기동 시간은 평균 약 `130초` 정도로 확인했다.

이 시간에는 아래 과정이 포함된다.

- 빌드 확인
- PX4 시작
- Gazebo world 초기화
- 모델 spawn
- Gazebo GUI 연결

환경 상태나 이전 캐시 상태에 따라 약간 달라질 수 있다.

## 실행 방법

다른 터미널에서 아래처럼 실행하면 된다.

```bash
cd ~/PX4-Autopilot_ASP
make px4_sitl gz_x500_gimbal
```

ROS 2 환경이 필요한 경우:

```bash
source /opt/ros/humble/setup.bash
```

## 여전히 창이 안 보일 때

실행 자체는 됐지만 창이 다른 workspace에 있거나 포커스를 잃은 경우가 있을 수 있다.

확인용 명령:

```bash
pgrep -af "gz sim|px4"
```

필요하면 GUI만 수동으로 다시 붙일 수 있다.

```bash
gz sim -g
```

## 확인된 비치명 경고

아래 경고들은 확인됐지만, 이번 문제의 직접 원인은 아니었고 실행 자체를 막지도 않았다.

- `Failed to load system plugin [libGstCameraSystem.so]`
- `libEGL warning: egl: failed to create dri2 screen`
- `Gazebo does not support Ogre material scripts`

즉, 이 경고들이 있어도 PX4와 Gazebo 기동 자체는 가능했다.

## 메모

- 이번 단계에서 ROS 패키지 구조 자체를 바꾼 것은 아니다.
- 핵심 기능 수정은 워크스페이스 내부가 아니라 PX4 실행 스크립트 쪽에서 이루어졌다.
- 목적은 `make px4_sitl gz_x500_gimbal` 명령 하나로 Gazebo GUI까지 안정적으로 올라오게 만드는 것이었다.
