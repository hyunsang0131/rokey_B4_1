# robot_setting

티칭 펜던트에서 하던 기본 조작을 노트북 터미널에서 실행하는 패키지입니다.

```bash
ros2 run robot_setting <실행 파일> [옵션]
```

| 실행 파일 | 펜던트 기능 | 로봇이 움직이나 |
|---|---|---|
| `status` | 상태 화면 | ✗ |
| `servo_on` / `servo_off` | 서보 켜기 / 끄기 | ✗ |
| `home` | 홈 이동 | ✅ |
| `direct_teach` | 콕핏 직접 교시 + 위치 저장 | 손으로 움직임 |
| `tool_tcp` | 툴(무게) / TCP(오프셋) 설정 | ✗ |

모든 실행 파일이 공통 연결 코드 [`common.py`](robot_setting/common.py)를 사용합니다.

---

## status

로봇을 움직이지 않고 현재 상태를 출력합니다.

```bash
ros2 run robot_setting status           # 한 번 출력
ros2 run robot_setting status --watch   # 1초마다 계속 출력 (Ctrl+C 종료)
```

출력 항목은 상태(STANDBY, SAFE_OFF 등), 모드(MANUAL/AUTONOMOUS), 툴/TCP 이름, 관절 각도, TCP 위치, 툴 힘, 최근 알람입니다.

## servo_on / servo_off

```bash
ros2 run robot_setting servo_on
ros2 run robot_setting servo_off
```

- `servo_on`: 현재 상태가 SAFE_OFF(서보 꺼짐)인지 SAFE_STOP(보호 정지)인지에 따라 맞는 해제 명령을 보내고, 실제로 STANDBY가 됐는지 확인합니다.
- `servo_off`: 서보를 끄고 SAFE_OFF가 됐는지 확인합니다.

## home

```bash
ros2 run robot_setting home              # 작업 홈 [0, 0, 90, 0, 90, 0] (movej)
ros2 run robot_setting home --user       # 펜던트에 등록한 사용자 홈 (move_home)
ros2 run robot_setting home --mechanic   # 기계 원점 [0, 0, 0, 0, 0, 0] (move_home)
```

- 실행 전 STANDBY 상태여야 합니다(먼저 `servo_on`). 필요하면 AUTONOMOUS 모드로 전환합니다.
- 이동 중 서보 오프나 보호 정지가 되면 바로 멈추고, 알람 이름을 출력합니다.

> ⚠️ **`--user` / `--mechanic`은 실물에서 서보가 꺼집니다.**
> 호출하고 약 0.5초 뒤 안전 컨트롤러 알람 `group=5 index=7056 (OPERATION_SAFETY_FUNCTION_SOS_VIOLATION)`이 발생합니다.
> 두산 라이브러리에서 `move_home`은 jog와 같은 hold-to-run 명령으로 분류되어 있어서, AUTONOMOUS 모드에서 한 번만 호출하는 것이 원인으로 보입니다(확인 중).
> 실물에서는 기본 `home`(movej)을 사용하세요.

## direct_teach

콕핏 버튼으로 로봇을 손으로 움직이고, 원하는 자세를 저장합니다.

```bash
ros2 run robot_setting direct_teach
```

1. 로봇을 MANUAL 모드로 바꿉니다.
2. 콕핏 버튼을 누른 채 팔을 움직이면 상태가 TEACHING이 되고, 터미널에 `>> 직접 교시 중`이 표시됩니다.
3. 버튼을 놓고 터미널에 명령을 입력합니다.

| 입력 | 동작 |
|---|---|
| Enter | 현재 위치를 `p1`, `p2`, … 이름으로 저장 |
| `이름` + Enter | 그 이름으로 저장 (예: `bed_c1`) |
| `l` | 저장한 목록 |
| `u` | 마지막 저장 취소 |
| `q` | 저장하고 종료 |

- 저장 위치: `~/ws_cobot_pjt/teach_points/teach_날짜_시간.json` (`name`, `posj`, `posx`, `tcp`, `time`)
- 종료할 때 코드에 붙여 쓸 수 있게 `posj(...)` / `posx(...)` 형식으로도 출력하고, 원래 모드로 되돌립니다.
- 한계: 콕핏 버튼의 기능 설정과 버튼 눌림 여부는 ROS 2 드라이버로 읽을 수 없어서 **펜던트 설정을 그대로** 쓰고, 버튼 대신 TEACHING 상태로 판단합니다.

## tool_tcp

툴(무게·무게중심)과 TCP(툴 끝점 오프셋)를 설정합니다. 둘 다 MANUAL 모드에서만 바꿀 수 있어서, MANUAL로 전환했다가 끝나면 원래 모드로 되돌립니다.

```bash
ros2 run robot_setting tool_tcp show
ros2 run robot_setting tool_tcp set --tool "Tool Weight_1" --tcp GripperDA_v1
ros2 run robot_setting tool_tcp add-tool <이름> <무게kg> <cx> <cy> <cz> [--inertia I×6] [--use]
ros2 run robot_setting tool_tcp add-tcp  <이름> <x> <y> <z> <a> <b> <c> [--use]
ros2 run robot_setting tool_tcp del-tool <이름>
ros2 run robot_setting tool_tcp del-tcp  <이름>
```

| 그리퍼의 무엇 | 두산 이름 | 명령 |
|---|---|---|
| 오프셋 (툴 끝점 위치·방향, 플랜지 기준 mm/deg) | TCP | `add-tcp`, `set --tcp` |
| 무게 (kg) + 무게중심 (플랜지 기준 mm) | Tool | `add-tool`, `set --tool` |

- `--use`를 붙이면 등록한 뒤 바로 선택합니다.
- **툴(무게)이 설정되어 있지 않으면 그리퍼 무게가 힘 값에 그대로 들어갑니다.** 정지 상태에서 Fz가 0이 아니면 툴 설정부터 확인하세요.
- 등록된 툴/TCP 목록은 드라이버로 읽을 수 없어서 펜던트에서 확인해야 합니다.
- 에뮬레이터는 등록되지 않은 이름도 성공으로 처리합니다. 실행 후 출력되는 "현재 툴/TCP"를 확인하세요.

## 문제 해결

| 증상 | 확인할 것 |
|---|---|
| `서비스 연결 실패` | 브링업 실행 여부, `ROS_DOMAIN_ID`가 브링업과 같은지 |
| `STANDBY 가 아닙니다` | `servo_on` 실행, 알람 확인(`status`) |
| 서보가 갑자기 꺼짐 | `status`의 최근 알람 (예: 7056 = SOS 위반) |
