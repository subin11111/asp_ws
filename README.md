# ASP ROS2 Workspace

## 개요

이 워크스페이스는 `PX4-Autopilot_ASP`, ROS 2 Humble, Gazebo Sim(`gz`) 환경과 함께 사용한다.

이번에 정리한 핵심 내용은 아래 명령이 보다 안정적으로 동작하도록 만든 것이다.

```bash
make px4_sitl gz_x500_gimbal
```

목표는 위 명령을 실행했을 때:

- PX4가 정상 시작되고
- Gazebo 서버가 정상 시작되고
- 모델이 정상 spawn되며
- Gazebo GUI까지 자동으로 보이게 만드는 것

## 기존 문제

기존에는 `make px4_sitl gz_x500_gimbal` 실행 시 Gazebo 서버만 뜨고 GUI가 보이지 않는 경우가 있었다.

실제로는 아래와 같은 서버 프로세스만 올라간 상태가 자주 발생했다.

```text
gz sim --verbose=1 -r -s <world.sdf>
```

반면 GUI 프로세스인 아래 명령은 실행되지 않거나, 실행되더라도 유지되지 않는 경우가 있었다.

```text
gz sim -g
```

그래서 겉으로는:

- Gazebo가 이미 실행 중이라고 나오고
- PX4는 모델을 spawn하려고 진행하지만
- 실제 Gazebo 창은 안 보이는

혼란스러운 상태가 생겼다.

## 원인

원인은 PX4의 Gazebo 실행 스크립트 동작 방식에 있었다.

파일:

`/home/sunny/PX4-Autopilot_ASP/ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim`

기존 흐름은 대략 다음과 같았다.

1. Gazebo 서버를 `-s` 옵션으로 실행
2. GUI를 한 번 실행 시도
3. world 준비가 되면 PX4 계속 진행

문제는 world 준비가 정상이어도 GUI 프로세스가 없거나 중간에 죽은 경우를 다시 확인하지 않는다는 점이었다.

즉:

- 서버는 살아 있음
- world도 준비됨
- PX4도 계속 진행함
- 하지만 GUI는 실제로 없음

이 상황이 발생할 수 있었다.

## 적용한 수정

PX4 실행 스크립트에 Gazebo world 준비 완료 후 GUI가 실제로 실행 중인지 다시 확인하는 로직을 추가했다.

수정 파일:

`/home/sunny/PX4-Autopilot_ASP/ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim`

수정 내용:

- `start_gz_gui_if_needed()` 함수 추가
- `HEADLESS` 환경에서는 GUI를 띄우지 않도록 유지
- `gz sim -g` 프로세스가 이미 있으면 중복 실행하지 않음
- Gazebo world가 준비된 뒤 GUI가 없으면 자동으로 다시 실행

즉, 이제는 world 준비 후에도 GUI가 빠져 있으면 자동 보정이 들어간다.

## 수정 결과

수정 후 아래 동작을 확인했다.

- PX4 정상 시작
- Gazebo 서버 정상 시작
- Gazebo world 준비 완료
- `x500_gimbal_0` 모델 spawn 성공
- `gz sim -g` 프로세스 실행 확인
- Gazebo GUI 창 표시 확인

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
