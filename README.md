# glim_assignment


## 프로젝트 구조

```
ros2_ws/
├── src/
│   ├── my_ros2_assignment/          # 메인 GUI 컨트롤러 패키지
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── resource/
│   │   │   └── my_ros2_assignment
│   │   └── my_ros2_assignment/
│   │       ├── __init__.py
│   │       ├── my_node.py           # 진입점
│   │       ├── robot_controller.py  # ROS2 노드 (서비스 클라이언트)
│   │       ├── gui/
│   │       │   ├── __init__.py
│   │       │   ├── main_window.py   # 메인 윈도우
│   │       │   ├── control_panel.py # 제어 패널
│   │       │   ├── status_panel.py  # 상태 패널
│   │       │   └── worker_threads.py # 백그라운드 스레드
│   │       └── utils/
│   │           ├── __init__.py
│   │           └── constants.py     # 상수 정의
│   │
│   ├── doosan-robot2/               # Git Submodule (DoosanRobotics/doosan-robot2)
│   │
│   └── doosan-robot2_patches/       # 수정된 파일 (덮어쓰기용)
│       ├── dsr_bringup2/launch/
│       │   └── dsr_bringup2_gazebo.launch.py
│       └── dsr_description2/
│           ├── config/
│           │   └── macro.gazebo.xacro
│           └── xacro/
│               └── e0509.urdf.xacro
│
├── Presentation.pptx                # 과제 발표 자료
├── README.md                        # 실행 매뉴얼
└── requirements.txt                 # 의존성 패키지 목록
```

---

## 실행 매뉴얼

### 1. 사전 요구사항

- Ubuntu 22.04 (Jammy)
- ROS2 Humble
- Python 3.10+

### 2. 저장소 클론

```bash
# submodule(doosan-robot2) 포함하여 클론
git clone --recursive https://github.com/woo10000k/glim_my_ros2_assignment.git ~/ros2_ws

# 이미 클론한 경우 submodule 받기
cd ~/ros2_ws
git submodule update --init
```

### 3. 의존성 설치

```bash
cd ~/ros2_ws

# APT 패키지 (requirements.txt에서 추출하여 설치)
sudo apt update
sudo apt install -y $(grep "^# ros-\|^# python3-" requirements.txt | sed 's/^# //')

# Python 패키지
pip install -r requirements.txt
```

### 4. doosan-robot2 패치 적용

```bash
cd ~/ros2_ws/src

cp doosan-robot2_patches/dsr_description2/config/macro.gazebo.xacro \
   doosan-robot2/dsr_description2/config/

cp doosan-robot2_patches/dsr_description2/xacro/e0509.urdf.xacro \
   doosan-robot2/dsr_description2/xacro/

cp doosan-robot2_patches/dsr_bringup2/launch/dsr_bringup2_gazebo.launch.py \
   doosan-robot2/dsr_bringup2/launch/
```

| 파일 | 수정 내용 |
|------|----------|
| macro.gazebo.xacro | Gazebo 링크 이름 및 visual material 수정 |
| e0509.urdf.xacro | ros2_control update_rate 파라미터 추가 |
| dsr_bringup2_gazebo.launch.py | model 파라미터 전달 추가 |

### 5. 빌드

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 6. 실행

**터미널 1: Gazebo 시뮬레이션**
```bash
ros2 launch dsr_bringup2 dsr_bringup2_gazebo.launch.py model:=e0509
```

**터미널 2: GUI 컨트롤러**
```bash
ros2 run my_ros2_assignment my_node
```

---

## 작동 로직

### 동작 프로세스

1. 사용자가 목표 TCP 좌표 (X, Y, Z, RX, RY, RZ) 입력
2. 좌표 모드 선택 (Absolute: 절대좌표 / Relative: 상대좌표)
3. 최대 속도/가속도 설정
4. Add Target으로 타겟 리스트에 추가
5. Run All(순차 실행) 또는 Run Single(단일 실행)
6. Stop/Pause/Resume으로 동작 제어

### Gazebo 시뮬레이션 연동

- **MoveJointx 서비스 사용**: MoveLine은 Gazebo에서 Singularity 에러 발생 → MoveJointx로 내부 IKin 수행하여 회피
- **Fkin으로 TCP 계산**: Gazebo는 RobotState 토픽 미발행 → JointState에서 관절 각도 수신 후 Fkin 서비스로 TCP 변환
- **별도 스레드 실행**: ROS2 서비스 호출이 GUI 블로킹 → QThread 기반 워커 스레드로 분리

---

## 각 기능 설명

### 사용 서비스

| 서비스 | 용도 |
|--------|------|
| /dsr01/motion/move_jointx | TCP 좌표로 이동 (내부 IKin) |
| /dsr01/motion/move_joint | 관절 각도로 이동 |
| /dsr01/motion/move_stop | 동작 정지 |
| /dsr01/motion/move_pause | 동작 일시정지 |
| /dsr01/motion/move_resume | 동작 재개 |
| /dsr01/motion/fkin | 순기구학 (관절→TCP) |
| /dsr01/system/get_last_alarm | 알람 조회 |

### 구독 토픽

| 토픽 | 용도 |
|------|------|
| /dsr01/joint_states | 현재 관절 각도 (Gazebo) |
| /dsr01/state | 로봇 상태 (실제 로봇) |

### 버튼 기능

| 버튼 | 기능 |
|------|------|
| Add Target | 현재 입력값을 타겟 리스트에 추가 |
| Run All | 타겟 리스트 순차 실행 |
| Run Single | 선택된 타겟 또는 입력값 단일 실행 |
| Stop | 동작 정지 (Soft Stop) |
| Pause | 동작 일시정지 |
| Resume | 동작 재개 |
| Home | 홈 위치 이동 (모든 관절 0도) |

---

## UI 구성

### 레이아웃

| 영역 | 구성 요소 |
|------|----------|
| **제어 패널 (좌측)** | 좌표 입력, 모드 선택, 속도/가속도 설정, 실행 버튼 |
| **상태 패널 (우측)** | 연결 상태, 관절 각도, TCP 위치, 타겟 리스트, 로그 |

### 제어 패널 상세

- **좌표 입력**: X, Y, Z (mm) / RX, RY, RZ (deg) + 증감 버튼 (±0.1, ±1, ±10, ±100)
- **좌표 모드**: Absolute (Base 원점 기준) / Relative (현재 TCP 기준 오프셋)
- **속도/가속도**: 1~1000 mm/s, mm/s² 범위 설정
- **실행 버튼**: Run All, Run Single, Stop, Pause, Resume, Home

### 상태 패널 상세

- **연결 상태**: 로봇 연결 여부 표시
- **관절 각도**: J1~J6 현재 각도 (deg)
- **TCP 위치**: X, Y, Z, RX, RY, RZ 현재 위치
- **타겟 리스트**: 번호, 좌표, 속도, 모드, 상태 (Pending/Moving/Completed/Error)
- **에러 표시**: 동작 실패시 에러 메시지
- **실시간 로그**: 타임스탬프 포함 동작 로그

---

## 프레임워크

### 소프트웨어 스택

| 계층 | 기술 |
|------|------|
| GUI | PyQt5 (코드 직접 구현, Qt Designer 미사용) |
| ROS2 통신 | rclpy (Service Client, Topic Subscriber) |
| 시뮬레이션 | Gazebo + ros2_control |
| 로봇 패키지 | doosan-robot2 (dsr_msgs2, dsr_bringup2) |

### 아키텍처

```
┌─────────────────────────────────────────────────────────────────┐
│                    my_node.py (Entry Point)                     │
│  - ROS2 초기화 (rclpy.init)                                     │
│  - PyQt5 Application 생성                                        │
└─────────────────────────────────────────────────────────────────┘
                                │
                ┌───────────────┴───────────────┐
                ▼                               ▼
┌─────────────────────────┐     ┌─────────────────────────────────┐
│   RobotController       │     │   MainWindow (GUI)              │
│   (ROS2 Node)           │     │   - ControlPanel (사용자 입력)    │
│                         │     │   - StatusPanel (상태 표시)       │
│   Services:             │◄────┤   - WorkerThreads (별도 스레드)   │
│   - MoveJointx          │     │                                 │
│   - Fkin                │     │                                 │
│   - GetLastAlarm        │     │                                 │
└─────────────────────────┘     └─────────────────────────────────┘
```

### 스레드 구조

| 스레드 | 역할 |
|--------|------|
| Main Thread | PyQt5 GUI 이벤트 루프 |
| StateMonitorThread | 로봇 상태 모니터링 (100ms 주기), ROS2 spin |
| MoveWorkerThread | 다중 타겟 순차 실행 |
| SingleMoveWorkerThread | 단일 타겟 실행 |
| HomeMoveWorkerThread | 홈 위치 이동 |

