# cushion_hmi

쿠션 진단·모듈 교체 작업을 보고 조작하는 **PyQt5 HMI**와, 로봇 없이 HMI를 시험하는 **모의 미션 노드**입니다.

| 실행 파일 | 역할 |
|---|---|
| `hmi` | 재고·트레이·RViz·주문 탭이 있는 HMI 시연 화면. 기본으로 트레이 현황을 표시 |
| `mock_mission` | 가짜 미션 노드. 플로우차트 순서대로 상태를 바꾸고 모의 측정값을 발행 |
| `order_server` | 로컬 HTML 주문을 받아 ROS 2로 HMI에 전달 |

HMI는 로봇을 직접 움직이지 않는 UI 시연 화면입니다. HTML 주문은 `order_server`가 ROS 2로 전달하고, HMI는 주문 토픽을 구독해 고객 주문 탭을 갱신합니다.

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

브라우저 주문 HTTP 서버는 HMI가 시작할 때 함께 실행됩니다. ROS 2 환경에서 HMI를 실행합니다.
HMI 시작 시 바탕화면에 "쿠션 HMI 및 주문 페이지" 실행기를 자동으로 만들며,
같은 실행기가 이미 있으면 중복 생성하지 않습니다. 이 실행기는 주문 페이지를 브라우저로
열고 HMI를 시작합니다.

```bash
ros2 run cushion_hmi hmi
```

그다음 `bed_ordering.html`을 브라우저에서 열고 여섯 슬롯을 구성한 뒤 `나만의 침대 주문하기`를 누릅니다.
주문서에는 전송 성공/실패가 표시됩니다. 접수된 주문은 HMI의 고객 주문 탭에 접수 순서대로
대기열에 쌓입니다. 주문을 선택하면 모델·규격, 슬롯별 강성, 임시 배송 주소를 볼 수 있으며
삭제, 우선순위 위/아래 이동,
제작 실행 버튼을 사용할 수 있습니다. 제작 실행은 현재 UI 안내만 표시하며 ROS 2 명령을
보내지 않습니다. HMI는 `127.0.0.1:5000`에서 요청을 받으므로 HTML과 HMI는 같은 PC에서 실행해야 합니다.
주문 서버만 따로 사용하려면 `ros2 run cushion_hmi order_server`를 실행할 수 있지만,
HMI와 동시에 실행하면 포트가 겹치므로 함께 실행하지 마세요.
HTML에서 배송 주소를 보내지는 않으며, 주소는 주문을 받을 때마다 새로 생성됩니다.
주문 슬롯은 강성(`부드러움`/`단단함`)과 높이(`NORMAL`/`HIGH`)를 함께 전달합니다.
이전 형식의 강성 문자열만 있는 주문도 기본 높이로 표시합니다. HMI 트레이는 HTML 색상과
맞춰 단단함은 파란색, 부드러움은 베이지색으로 표시하고, 부드러운 높은 모듈은 더 진한
베이지색과 `높은 지지` 라벨로 구분합니다.

- `defects`: 처음부터 무른 모듈이 들어 있는 슬롯 (결함 모의 실험)
- `spares`: 선반에 있는 교체용 모듈 수
- `bad_spares`: 교체용 모듈 중 불량 개수. 1 이상이면 재측정에서 다시 불량이 나와 재시도·알람 흐름을 시험할 수 있습니다.

## 화면

- **상단**: 모드 선택, 시작·정지 요청·초기화, 비상 정지 시연 버튼
- **재고 관리**: 시연용 교체 블록 수량과 기준 강성
- **트레이 현황**: 매트리스/예비 트레이 모식도, 상세 정보, 그래프와 작업 로그
- **작업 상태 (RViz)**: RViz 연결 영역 자리표시자
- **고객 주문 사항**: HTML 주문 대기열, 선택 주문의 트레이 그림·상세, 삭제·우선순위 변경·제작 실행(UI 전용)
- **비상 정지**: UI 시연 상태만 잠그며 실제 안전회로와 연결되지 않습니다. 실제 장비에는 사용하지 마세요.

## 통신 규약

모두 `std_msgs/String`에 JSON을 담습니다. 자세한 필드는 [`protocol.py`](cushion_hmi/protocol.py) 맨 위 주석에 있습니다.

| 토픽 | 방향 | 내용 |
|---|---|---|
| `/cushion/cmd` | HMI → 미션 | `start`(+`mode`), `stop`, `continue`, `reset` |
| `/cushion/status` | 미션 → HMI | 상태, 모듈별 결과, 작업자 확인, 알람. 0.5초마다 발행하며, 2초 동안 안 오면 HMI가 "응답 없음"으로 표시 |
| `/cushion/force` | 미션 → HMI | 압입 샘플 `{module, z, fz, reset}` |
| `/cushion/log` | 미션 → HMI | 로그 한 줄 |
| `/cushion/order_config` | 로컬 주문 서버 → HMI | 브라우저 주문 번호와 6개 슬롯 설정 |

실제 미션 노드를 만들 때는 `protocol.py`를 import 해서 같은 형식으로 발행하면 HMI를 고치지 않고 그대로 쓸 수 있습니다. `mock_mission.py`의 구조(제너레이터로 만든 상태 머신)를 출발점으로 삼으면 됩니다.

## 파일

```
cushion_hmi/
├─ cushion_hmi/
│  ├─ protocol.py      # 토픽 이름, 상태·모드 목록, 모듈 배치, JSON 형식
│  ├─ ros_bridge.py    # rclpy 를 QThread 에서 돌리고 Qt 시그널로 전달
│  ├─ order_server.py  # HTML 주문 HTTP 수신 및 ROS 토픽 전달
│  ├─ widgets.py       # 모듈 격자, 단계 표시줄, 힘-변위 그래프
│  ├─ hmi.py           # 메인 창 (실행 파일 hmi)
│  └─ mock_mission.py  # 모의 미션 노드 (실행 파일 mock_mission)
└─ launch/demo.launch.py
```

추가 설치는 필요 없습니다. Ubuntu 24.04 기본 `python3-pyqt5`만 씁니다.
