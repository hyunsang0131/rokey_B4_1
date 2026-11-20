# cushion_hmi

쿠션 진단·모듈 교체 작업을 보고 조작하는 **PyQt5 HMI**와, 로봇 없이 HMI를 시험하는 **모의 미션 노드**입니다.

| 실행 파일 | 역할 |
|---|---|
| `hmi` | 작업 화면. 모듈 6개 상태, 플로우 단계, 힘-변위 그래프, 작업자 확인, 알람, 로그 |
| `mock_mission` | 가짜 미션 노드. 플로우차트 순서대로 상태를 바꾸고 모의 측정값을 발행 |
| `order_server` | 로컬 HTML 주문을 받아 ROS 2로 HMI에 전달 |

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

브라우저 주문값을 HMI에 표시하려면 같은 ROS 환경에서 터미널 두 개를 열어 각각 실행합니다.

터미널 1:

```bash
ros2 run cushion_hmi order_server
```

터미널 2:

```bash
ros2 run cushion_hmi hmi
```

그다음 `bed_ordering.html`을 브라우저에서 열고 여섯 슬롯을 선택해 `ADD TO CART`를 누릅니다.
`order_server`는 `127.0.0.1:5000`에서만 요청을 받으며, HMI의 "브라우저 주문 설정"에
주문 번호와 슬롯별 강성값(6.2 또는 12 N/mm)을 표시합니다. HTML과 주문 서버는 슬롯 선택값을
`부드러움`/`단단함` 문자열로 주고받고, HMI가 화면 표시 시에만 `protocol.py`의 강성값으로
변환합니다. `/cushion/order_config`의 `slots` 값도 원래 문자열을 유지합니다.

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

## 실제 로봇 공정(bed_inspect)과 연결해서 주문대로 구성하기

`bed_inspect` 의 `cushion_bridge` 가 이 화면(`/cushion/*`)과 9단계 공정(`bed_process`, `/bed/*`)을 이어 준다.
작업은 명령 두 개로 한다 (launch 를 켠 뒤 다른 터미널에서):

```bash
ros2 run bed_inspect bed_cmd inspect          # ① 첫 공정 전체: 측정 → 판정 → 커버 열기 → 교체 → 커버 닫기 → 재측정 (자동 보수)
ros2 run bed_inspect bed_cmd order            # ② 주문대로 구성: 웹 주문(마지막으로 받은 것, 없으면 기다림)
ros2 run bed_inspect bed_cmd order 주문.json  #    웹 대신 bed_ordering.html 이 내려받은 JSON 파일
```

주문대로 구성은 **측정 → 주문과 다른 칸·빈칸 찾기 → 보관대에서 맞는 큐브로 교체 → 재측정** 을 한다
(`bed_process` 의 repair 사용자 지정 모드, 슬롯 값 부드러움 → soft, 단단함 → hard).

```bash
# 로봇 bringup 을 먼저 켠 뒤, 한 번에:
ros2 launch bed_inspect cushion_system.launch.py teach_file:=$HOME/ws_cobot_pjt/teach_points/bed.json
#   auto_start:=true   → 웹 주문이 오면 bed_cmd 없이 바로 시작 (기본 false)
#   port:=5000 host:=0.0.0.0
```

- **IP**: `order_server` 는 기본으로 `0.0.0.0:5000` 에서 받는다(다른 PC 에서도 접속 가능). 실행하면 로그에 `주문 페이지: http://<이 PC IP>:5000/` 이 나온다.
  다른 PC 브라우저에서 그 주소를 열면 주문 페이지가 뜨고, 주문은 같은 주소의 `/order` 로 간다(HTML 의 IP 를 고칠 필요 없음).
  HTML 을 파일로 열거나 다른 서버에 올렸으면 주소 뒤에 `?hmi=http://<로봇 PC IP>:5000/order` 를 붙인다. 방화벽에서 5000 포트를 열어야 할 수 있다.
- **칸 번호**: 웹 Slot n = 화면 Mn = 공정 n번 칸. 다르면 `cushion_bridge` 의 `slot_map` 파라미터로 바꾼다 (예: `-p slot_map:="[2,1,4,3,6,5]"`).
- **화면 모드**: 기준값 측정 / 진단만(측정) / 진단 + 교체(불량 교체) / 자동 보수 / 주문대로 구성(마지막 주문). 단계 표시는 9단계 공정과 같다(천 조이기·풀기 없음).
- 공정이 도는 중에 주문이 오면 끝난 뒤 시작한다. 공정이 오류 상태면 자동으로 시작하지 않는다.

### 가상으로 전 과정 돌리기 (에뮬레이터 + RViz, 로봇 없이)

실물 bringup 과 겹치지 않게 ROS 도메인을 따로 쓴다 (실물이 켜져 있어도 된다). 터미널 두 개:

```bash
# 터미널 1: 에뮬레이터 + RViz
export ROS_DOMAIN_ID=77
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=virtual

# 터미널 2: 공정 + 연결 + 주문 서버 + 화면 (가상 모드)
export ROS_DOMAIN_ID=77
ros2 launch bed_inspect cushion_system.launch.py virtual:=true port:=5078

# 터미널 3: 명령 (같은 도메인)
export ROS_DOMAIN_ID=77
ros2 run bed_inspect bed_cmd inspect
ros2 run bed_inspect bed_cmd order        # 웹(http://localhost:5078/)에서 주문하거나 JSON 파일을 준다
```

- `bed_cmd` 로 시작하면 RViz 에서 홈 → 측정 → 보관대 재기 → 커버 열기 → 교체 → 커버 닫기 → 재측정까지 9단계가 돈다.
- 교시 파일은 `teach_points/bed_emu.json` (없으면 처음 한 번 `emu_teach` 가 에뮬레이터에서 자동으로 만든다). 실물에서는 쓰지 않는다.
- 힘은 가짜(`bed_emu`): `emu_force:="bed=H,H,H,H,H,H;rack=S,S,S,H,H,-"` (H 단단, S 무름, T 키 큰 무름, - 빈칸). 시작할 때마다 `~/ws_cobot_pjt/bed_data_emu` 재고가 이 값으로 초기화된다.
- `quick:=false` 면 실물과 같은 속도로 누른다 (오래 걸림).
- 에뮬레이터에는 GripperDA_v1 TCP 가 없어서 커버 좌표가 플랜지 기준으로 간다 (RViz 에서 그리퍼가 조금 낮아 보임).


## 명령 하나로 (권장)

```bash
ros2 launch bed_inspect bed_all.launch.py job:=inspect robot:=virtual      # 공정 전체 (가상)
ros2 launch bed_inspect bed_all.launch.py job:=order   robot:=virtual      # 주문대로 구성 (가상, 웹 주문을 기다림)
ros2 launch bed_inspect bed_all.launch.py job:=inspect robot:=real teach_file:=$HOME/ws_cobot_pjt/teach_points/bed.json
ros2 launch bed_inspect bed_all.launch.py job:=order   robot:=real teach_file:=$HOME/ws_cobot_pjt/teach_points/bed.json
```

로봇 bringup → 공정·연결·주문 서버·화면 → 고른 작업(9단계)까지 한 번에 한다.
`job:=order` 는 켠 뒤 웹(`http://<이 PC IP>:5000/`)에서 주문하면 시작한다 (`order_file:=주문.json` 이면 바로).
실물 bringup 이 켜져 있는데 가상을 같이 띄우려면 먼저 `export ROS_DOMAIN_ID=77`.
