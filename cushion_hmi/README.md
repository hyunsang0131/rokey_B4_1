# cushion_hmi

쿠션 진단·모듈 교체 작업을 보고 조작하는 **PyQt5 HMI**와, 로봇 없이 HMI를 시험하는 **모의 미션 노드**입니다.

| 실행 파일 | 역할 |
|---|---|
| `hmi` | 작업 화면. 모듈 6개 상태, 플로우 단계, 힘-변위 그래프, 작업자 확인, 알람, 로그 |
| `mock_mission` | 가짜 미션 노드. 플로우차트 순서대로 상태를 바꾸고 모의 측정값을 발행 |

HMI는 로봇을 직접 움직이지 않고 `/cushion/*` 토픽으로 미션 노드와만 통신합니다. 그래서 로봇이 움직이는 동안에도 화면이 멈추지 않습니다.

## 실행

```bash
cd ~/ws_cobot_pjt/ws_dsr
colcon build --packages-select cushion_hmi
source install/setup.bash
```

로봇 없이 시험 (모의 미션 + HMI 같이 실행):

```bash
ros2 launch cushion_hmi demo.launch.py
```

따로 띄우기, 결함 위치 바꾸기:

```bash
ros2 run cushion_hmi mock_mission --ros-args -p defects:="[2,5]" -p spares:=2 -p bad_spares:=0
ros2 run cushion_hmi hmi
```

- `defects`: 처음부터 무른 모듈이 들어 있는 슬롯 (결함 모의 실험)
- `spares`: 선반에 있는 교체용 모듈 수
- `bad_spares`: 교체용 모듈 중 불량 개수. 1 이상이면 재측정에서 다시 불량이 나와 재시도·알람 흐름을 시험할 수 있습니다.

## 화면

- **상단**: 미션 노드 연결 상태, 선반 재고·재시도 횟수, 모드 선택(기준값 측정 / 진단만 / 진단 + 교체), 시작·정지 요청·초기화
- **단계 표시줄**: 플로우차트 단계 중 현재 단계 강조
- **작업자 확인 배너**: 천 조이기·커버 열기처럼 방식이 아직 정해지지 않은 단계는 작업자가 직접 하고 `작업 완료 · 계속`을 누릅니다
- **모듈 격자**: 배치 v3 기준(위 = 선반, 왼쪽 = 로봇). 상태별 색과 k, z0 값
- **힘-변위 그래프**: 압입 중 실시간 곡선. 현재 모듈은 진하게, 이전 모듈은 회색
- **로그**: 미션 노드 로그와 HMI 조작 기록

> `정지 요청`은 미션 노드에 보내는 소프트웨어 명령입니다. 위급 상황에서는 티칭 펜던트나 비상정지 버튼을 쓰세요.

## 통신 규약

모두 `std_msgs/String`에 JSON을 담습니다. 자세한 필드는 [`protocol.py`](cushion_hmi/protocol.py) 맨 위 주석에 있습니다.

| 토픽 | 방향 | 내용 |
|---|---|---|
| `/cushion/cmd` | HMI → 미션 | `start`(+`mode`), `stop`, `continue`, `reset` |
| `/cushion/status` | 미션 → HMI | 상태, 모듈별 결과, 작업자 확인, 알람. 0.5초마다 발행하며, 2초 동안 안 오면 HMI가 "응답 없음"으로 표시 |
| `/cushion/force` | 미션 → HMI | 압입 샘플 `{module, z, fz, reset}` |
| `/cushion/log` | 미션 → HMI | 로그 한 줄 |

실제 미션 노드를 만들 때는 `protocol.py`를 import 해서 같은 형식으로 발행하면 HMI를 고치지 않고 그대로 쓸 수 있습니다. `mock_mission.py`의 구조(제너레이터로 만든 상태 머신)를 출발점으로 삼으면 됩니다.

## 파일

```
cushion_hmi/
├─ cushion_hmi/
│  ├─ protocol.py      # 토픽 이름, 상태·모드 목록, 모듈 배치, JSON 형식
│  ├─ ros_bridge.py    # rclpy 를 QThread 에서 돌리고 Qt 시그널로 전달
│  ├─ widgets.py       # 모듈 격자, 단계 표시줄, 힘-변위 그래프
│  ├─ hmi.py           # 메인 창 (실행 파일 hmi)
│  └─ mock_mission.py  # 모의 미션 노드 (실행 파일 mock_mission)
└─ launch/demo.launch.py
```

추가 설치는 필요 없습니다. Ubuntu 24.04 기본 `python3-pyqt5`만 씁니다.
