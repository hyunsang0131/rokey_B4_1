# rokey_B4_1: 협동1 프로젝트

두산 **M0609** + **OnRobot RG2**로 실리콘 모듈 매트리스의 쿠션감(강성)을 측정하고, 무른 모듈을 찾아 교체하는 자동화 프로젝트의 ROS 2 코드입니다.

이 저장소에는 지금 **로봇을 노트북에서 다루기 위한 도구**, **측정 준비용 테스트 코드**, **작업 화면(HMI)**이 들어 있습니다.

| 패키지 | 역할 | 실행 파일 |
|---|---|---|
| [`robot_setting`](robot_setting/README.md) | 티칭 펜던트 기능을 노트북에서 실행 (상태, 서보, 홈, 직접 교시, 툴/TCP) | `status`, `servo_on`, `servo_off`, `home`, `direct_teach`, `tool_tcp` |
| [`pjt_test1`](pjt_test1/README.md) | 측정 기능 테스트 (센서 읽기, 교시 위치 이동, 위치별 힘 측정) | `read_sensor`, `move_to_teach`, `measure_teach_force` |
| [`cushion_hmi`](cushion_hmi/README.md) | PyQt5 작업 화면(HMI)과 로봇 없이 시험하는 모의 미션 노드 | `hmi`, `mock_mission` |

## search_test 브랜치: 큐브 탐지·트레이 넣기 (`cube_search`)

영역에 놓인 실리콘 큐브를 위에서 눌러 찾고(중심·각도·높이), 잡아서 교체 트레이의 빈칸에 넣는 노드입니다. 2026-11-20 실물에서 탐지 → 잡기 → 트레이 넣기까지 확인했습니다. 사용법은 [`pjt_test1/README.md`의 cube_search](pjt_test1/README.md#cube_search)를 보세요.

### 받아야 하는 파일 (코드 동작에 필요한 것은 이 7개가 전부, 이 안내가 있는 최상위 README.md 는 선택)

| 파일 | 상태 | 필요 |
|---|---|---|
| `pjt_test1/pjt_test1/cube_search.py` | 새 파일 | 필수: 메뉴, 로봇 제어, 시뮬 |
| `pjt_test1/pjt_test1/cube_search_core.py` | 새 파일 | 필수: 탐지 알고리즘 (`cube_search.py`가 import) |
| `pjt_test1/config/cube_search_config.json` | 새 파일 | 필수: 실측 설정 (영역, 바닥 높이 64.22, 트레이 바닥 78.57, 큐브 64×41 mm) |
| `pjt_test1/setup.py` | 수정 | 필수: `cube_search` 실행 파일과 설정 파일 등록 |
| `pjt_test1/README.md` | 수정 | 사용법 문서 |
| `pjt_test1/pjt_test1/gripper.py` | 수정 (주석만) | WebLogic 값 표기 |
| `pjt_test1/pjt_test1/bring_test.py` | 수정 (주석만) | WebLogic 값·큐브 크기 표기 |

`bed_inspect`, `cushion_hmi`, `robot_setting`, `pjt_test1`의 다른 파일은 이 브랜치에서 바뀌지 않았습니다.

### 받는 방법

**A. 내 브랜치(예: `ho3`)는 그대로 두고 위 7개 파일만 가져오기 (권장)**

```bash
cd ~/ws_cobot_pjt/ws_dsr/src/rokey_B4_1      # 저장소를 클론한 위치
git fetch origin
git checkout origin/search_test -- \
  pjt_test1/pjt_test1/cube_search.py \
  pjt_test1/pjt_test1/cube_search_core.py \
  pjt_test1/config/cube_search_config.json \
  pjt_test1/setup.py \
  pjt_test1/README.md \
  pjt_test1/pjt_test1/gripper.py \
  pjt_test1/pjt_test1/bring_test.py
```

- `ho3`의 `setup.py`·`README.md`·`gripper.py`·`bring_test.py`는 이 브랜치의 바탕(test2)과 같아서, 덮어써도 잃는 내용이 없습니다.
- `ho`·`ho2`처럼 더 오래된 브랜치에서는 `setup.py`에 `bring_test`·`stiffness_test`·`repair_test` 등록도 같이 들어갑니다. 해당 파일이 없으면 그 명령만 실행할 수 없고 빌드와 `cube_search`는 문제없습니다.

**B. 브랜치 통째로 쓰기**

```bash
git fetch origin
git switch search_test        # 처음이면 원격 search_test 를 따라가는 로컬 브랜치가 생김
```

이 브랜치는 `test2`(bed_inspect, repair_test 포함) 위에 위 7개 파일을 얹은 것입니다. `ho3`의 bed_inspect/cushion_hmi 변경은 들어 있지 않습니다.

### 받은 뒤 할 일

1. `~/ws_cobot_pjt/teach_points/exchange_fin.json`(교체 트레이 6칸 좌표)이 있어야 합니다. 저장소에는 없습니다. md5 `19ae1695a4bc4e5c967cd33b9e2b1fe3`이면 실측에 쓴 파일과 같습니다.
2. 빌드 (`setup.py`가 바뀌었으므로 꼭 필요):
   ```bash
   cd ~/ws_cobot_pjt/ws_dsr && colcon build --symlink-install --packages-select pjt_test1 && source install/setup.bash
   ```
3. 로봇 없이 확인 (시뮬, 1초 안에 끝남):
   ```bash
   ros2 run pjt_test1 cube_search --sim 275,0,30 --place
   ```
   `완료: 큐브를 트레이 1번 칸에 넣었습니다.`와 `[시뮬] 오차: 중심 1 mm 안팎`이 나오면 정상입니다.
4. 실물: `ros2 run pjt_test1 cube_search` → `s`(탐지만) / `g`(탐지 → 잡기 → 트레이 빈칸).
   - 첫 실행 때 `config/cube_search_config.json`이 `~/ws_cobot_pjt/cube_search/config.json`으로 복사됩니다. 이미 그 파일이 있으면 그 파일을 씁니다.
   - 로봇 조건: TCP `GripperDA_v1`, WebLogic 닫기 0 mm 5 N / 열기 66 mm 10 N / 잡기 60 mm 20 N, 영역 x 204.81~344.81 · y −73.94~76.06(베이스 기준)에 큐브 1개.
   - 배치가 바뀌었으면 `config.json`의 `area`를 고치고 메뉴 `f`(영역 바닥), `tf`(트레이 바닥)로 다시 잽니다.

## 환경

- Ubuntu 24.04, ROS 2 Jazzy
- Doosan ROS 2 드라이버 [`doosan-robot2`](https://github.com/DoosanRobotics/doosan-robot2) (`dsr_msgs2`, `dsr_common2`)
- RG2 브링업 패키지 `m0609_rg2_bringup` (강의 제공)

## 설치

워크스페이스 `src` 안에 클론하면 colcon이 두 패키지를 찾습니다.

```bash
cd ~/ws_cobot_pjt/ws_dsr/src
git clone https://github.com/hyunsang0131/rokey_B4_1.git
cd ..
colcon build --packages-select robot_setting pjt_test1 cushion_hmi
source install/setup.bash
```

## 실행 준비

1. **브링업**: 실물 로봇

   ```bash
   ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=<컨트롤러 IP> port:=12345 model:=m0609
   ```

   에뮬레이터로 먼저 시험하려면 `mode:=virtual`을 사용합니다.

2. **ROS_DOMAIN_ID 맞추기**: 브링업과 같은 값이어야 서비스에 연결됩니다. 연결되지 않으면 스크립트가 `서비스 연결 실패`를 출력합니다.

   ```bash
   export ROS_DOMAIN_ID=80   # 브링업을 실행한 터미널과 같은 값
   ```

3. **기본 사용 순서**

   ```bash
   ros2 run robot_setting status            # 상태 확인
   ros2 run robot_setting servo_on          # 서보 켜기
   ros2 run robot_setting home              # 작업 홈 [0,0,90,0,90,0]
   ros2 run robot_setting tool_tcp show     # 툴/TCP 확인 (툴 무게가 설정돼 있어야 힘 값이 맞음)
   ros2 run robot_setting direct_teach      # 콕핏으로 위치 교시 → JSON 저장
   ros2 run pjt_test1 move_to_teach         # 저장한 위치로 이동
   ros2 run pjt_test1 measure_teach_force   # 위치마다 Z축 힘 측정
   ```

## 데이터 저장 위치

| 내용 | 위치 | 만드는 실행 파일 |
|---|---|---|
| 교시 위치 (JSON) | `~/ws_cobot_pjt/teach_points/` | `robot_setting direct_teach` |
| 위치별 힘 측정 (CSV) | `~/ws_cobot_pjt/force_logs/` | `pjt_test1 measure_teach_force` |
| 센서 읽기 (CSV) | 실행한 폴더 | `pjt_test1 read_sensor` |

## 공통 주의사항

- **처음 실행할 때는 펜던트 비상정지를 손에 들고** 실행하세요. 동작 속도는 낮게 잡아 두었습니다(`movej` 30 deg/s).
- 로봇 모드: 프로그램 동작(`movej`, `movel`)은 **AUTONOMOUS**, 툴/TCP 설정과 직접 교시는 **MANUAL**에서만 됩니다. 각 스크립트가 필요한 모드로 전환하고, 설정 스크립트는 끝나면 원래 모드로 되돌립니다.
- 동기 `movel`/`movej`가 실행되는 동안에는 컨트롤러가 다른 서비스 요청을 처리하지 못합니다. 그래서 다른 터미널에서 읽는 센서 값이 이동 중에는 멈췄다가 이어집니다.
