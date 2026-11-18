# pjt_test1

쿠션감 측정에 필요한 동작을 하나씩 시험하는 패키지입니다.

```bash
ros2 run pjt_test1 <실행 파일> [옵션]
```

| 실행 파일 | 하는 일 | 로봇이 움직이나 |
|---|---|---|
| `read_sensor` | 위치와 툴 힘을 계속 출력 (노이즈 확인) | ✗ |
| `move_to_teach` | 교시 파일을 골라 저장된 위치로 이동 | ✅ |
| `measure_teach_force` | 교시 위치마다 이동해서 TCP Z축 힘 측정 | ✅ |
| `gripper` | RG2 그리퍼 열기 / 닫기 (컨트롤러 DO) | 그리퍼만 |
| `stiffness_test` | 침대 칸마다 눌러서 강성 k 측정 (`bed_inspect` 측정 코드 사용) | ✅ |

교시 파일은 [`robot_setting direct_teach`](../robot_setting/README.md#direct_teach)로 만듭니다.

---

## read_sensor

로봇을 움직이지 않고 TCP 위치와 툴 힘(BASE 기준)을 0.1초마다 출력합니다.

```bash
ros2 run pjt_test1 read_sensor   # Ctrl+C 종료
```

- 종료하면 Fx/Fy/Fz의 평균과 표준편차를 출력하고, 실행한 폴더에 `read_sensor_날짜_시간.csv`로 저장합니다.
- 확인할 것: 가만히 있을 때 Fz 표준편차(노이즈), 툴을 손으로 눌렀을 때 Fz 부호, 정지 상태 Fz 평균이 0 근처인지(툴 무게 설정).

## move_to_teach

```bash
ros2 run pjt_test1 move_to_teach                 # 파일 목록에서 선택 (최신 순)
ros2 run pjt_test1 move_to_teach 파일.json        # 파일 바로 지정
ros2 run pjt_test1 move_to_teach --movel         # 툴 끝 직선 이동
```

| 입력 | 동작 |
|---|---|
| 번호 | 그 위치로 이동 |
| `a` | 저장된 순서대로 전부 이동 |
| `l` | 목록 |
| `q` | 종료 |

- 기본은 `movej(posj)`라서 TCP 설정과 관계없이 교시한 자세 그대로 갑니다.
- `--movel`은 `movel(posx)`입니다. posx는 교시할 때의 TCP 기준이라 **현재 TCP가 다르면 실행을 거부**합니다.
- 이동 후 목표와 실제 위치의 차이를 출력합니다(movej는 관절 deg, movel은 위치 mm).
- `posj`가 없는 점(좌표로 계산한 점)은 `--movel`이 없어도 `movel(posx)`로 갑니다. 이때도 TCP가 같아야 합니다.

## stiffness_test

침대 칸 위로 가서 그리퍼를 닫고 손끝으로 눌러 강성을 잽니다. 누르는 방법은 공정(`bed_inspect`의 `press_at`)과 같아서, 여기서 나온 값을 공정 설정에 그대로 쓸 수 있습니다.

```bash
ros2 run pjt_test1 stiffness_test                  # 교시 파일 목록에서 선택 (Enter = teach_20261117_calc_bed6.json)
ros2 run pjt_test1 stiffness_test 파일.json         # 파일 바로 지정
ros2 run pjt_test1 stiffness_test --target 15      # 목표 힘 [N] (기본 20, 12~35)
ros2 run pjt_test1 stiffness_test --approach -30   # 교시 높이에서 빠르게 내려오는 양 [mm] (기본 -40)
ros2 run pjt_test1 stiffness_test --modbus         # 그리퍼를 /onrobot/sendCommand 로 (기본은 DO = WebLogic)
```

| 입력 | 동작 |
|---|---|
| 번호 | 그 칸 1회 측정 |
| `번호*횟수` | 같은 칸 반복 측정 (예: `4*5`, 흩어짐 ±5% 이내가 목표) |
| `a` | 전부 1회씩 |
| `n` | 지금 자리에서 힘 노이즈 측정 (접촉 기준 3N 이 충분한지) |
| `r` | 결과 표 |
| `q` | 저장 후 종료 (`~/ws_cobot_pjt/stiffness_test/`, CSV + 힘-변위 곡선 JSON) |

- 측정 순서: 그리퍼 닫기 → 칸 위 → 0.3초 쉬고 영점 → 40mm 내려가 0.3초 쉬고 영점 → 1mm 씩 내려가다 3N 넘으면 닿음 → 0.5mm 씩 목표 힘까지 → 5N~목표 구간 직선의 기울기 = k, 힘 0 지점 = 윗면 높이 → 교시 높이로 복귀.
- 두 영점 차이가 6N(`F_PRETOUCH`) 이상이면 "이미 닿음"으로 보고 그 칸은 측정하지 않습니다. 이때는 `--approach` 를 줄입니다.
- 결과 표: 칸별 k, 중앙값 대비 비율, 윗면 높이, 눌린 깊이, 최대 힘, R². 여러 칸을 재면 가장 단단한 칸 / 가장 무른 칸 비율도 나옵니다.
- 끝나도 그리퍼를 열지 않습니다 (`ros2 run pjt_test1 gripper open`).
- `bed_inspect` 패키지가 같은 워크스페이스에 빌드돼 있어야 합니다.

## measure_teach_force

```bash
ros2 run pjt_test1 measure_teach_force            # 파일 목록에서 선택
ros2 run pjt_test1 measure_teach_force 파일.json
```

| 입력 | 동작 |
|---|---|
| 번호 | 그 위치로 이동해서 측정 |
| `a` | 전부 순서대로 측정 |
| `r` | 지금까지 결과 (차수별) |
| `q` | 결과 출력, CSV 저장 후 종료 |

측정 순서는 `movej` 이동 → 0.5초 대기 → `get_tool_force` 20회 평균·표준편차입니다.

```
  --------------------------------------------------------
  [1차]  16:05:41
  이름                Fz(TCP) 평균      표준편차    Fz(BASE) 평균
  point1 z=172        11.50 N    0.16 N      -11.50 N
  point2 z=230        11.03 N    0.07 N      -11.03 N
  --------------------------------------------------------
```

- **Fz(TCP)**: 툴 좌표계 Z축 힘. 툴이 아래를 보면 툴 Z = 베이스 −Z라서 Fz(BASE)와 부호가 반대입니다.
- `a`나 번호를 한 번 실행할 때마다 1차, 2차, …로 구분됩니다.
- 저장 위치: `~/ws_cobot_pjt/force_logs/teach_force_날짜_시간.csv` (`round`, `name`, `fz_tool`, `fz_tool_std`, `fz_base`, 관절 각도, 교시 파일)
- 툴 무게가 설정되어 있지 않으면 시작할 때 경고합니다. 정지 상태 힘만 재며, 누르면서 재는 압입 측정은 아직 없습니다.

### HMI로 보기

측정하는 동안 [`cushion_hmi`](../cushion_hmi/README.md) 화면에 결과가 표시됩니다. 조작은 지금처럼 이 터미널에서 하고, HMI는 보기만 합니다(버튼 비활성).

```bash
# 터미널 1: 측정 (브링업과 같은 ROS_DOMAIN_ID)
ros2 run pjt_test1 measure_teach_force
# 터미널 2: HMI (같은 ROS_DOMAIN_ID)
ros2 run cushion_hmi hmi
```

- 교시점 1~6번이 HMI의 M1~M6에 대응합니다 (1 2 / 3 4 / 5 6, 위 = 선반 쪽, 왼쪽 = 로봇 쪽). 교시할 때 이 순서로 찍어 두면 됩니다. 7번째부터는 HMI에 나오지 않지만 측정·CSV에는 그대로 들어갑니다.
- 모듈 칸에는 Fz(TCP) 평균 ± 표준편차가, 그래프에는 샘플마다 Fz(TCP)가 시간축으로 그려집니다.
- 이동 실패나 STANDBY가 아닌 경우에는 HMI에 알람으로 뜹니다.
- `cushion_hmi`가 빌드돼 있지 않으면 경고만 하고 측정은 원래대로 진행합니다. 연결 코드는 [`hmi_link.py`](pjt_test1/hmi_link.py)입니다.

## gripper

```bash
ros2 run pjt_test1 gripper open                # 열기
ros2 run pjt_test1 gripper close               # 닫기
ros2 run pjt_test1 gripper                     # 대화형: o 열기 | c 닫기 | q 종료
ros2 run pjt_test1 gripper width 40 --modbus   # OnRobot 서비스로 너비 40 mm (아래 참고)
```

| 방식 | 옵션 | 경로 | 실물 결과 |
|---|---|---|---|
| **DO (기본)** | 없음 | 로봇 컨트롤러 DO1/DO2 → 그리퍼 (강의 `grip_test`와 같음) | ✅ 동작 (닫기 후 너비 0.7 mm) |
| Modbus | `--modbus` | `/onrobot/sendCommand` → OnRobot 드라이버 → Compute Box(192.168.1.1) | ❌ 명령은 전송되지만 그리퍼가 움직이지 않음 |

- DO 방식은 열기/닫기만 됩니다. 닫기는 DO1=ON, DO2=OFF, 열기는 DO1=OFF, DO2=ON입니다.
- 명령을 보낸 뒤 1초 기다립니다. 그리퍼가 실제로 다 움직였는지는 확인하지 않습니다.
- `--do`는 예전 옵션 이름이라 붙여도 기본(DO)과 같습니다.
- **Modbus 방식(`--modbus`)**: 너비 지정이 가능하지만, 이 셀에서는 그리퍼가 DO로 제어되도록 설정된 것으로 보여 동작하지 않았습니다. 시험 중 OnRobot 드라이버 노드가 종료되는 일도 있었고, 그러면 브링업을 다시 실행해야 서비스가 돌아옵니다. 에뮬레이터의 가상 그리퍼는 숫자를 너비가 아니라 관절 각도(rad)로 해석하므로, 에뮬레이터에서 `width`는 RViz 모양이 실제와 다릅니다.

## 좌표 메모

- `posj = [J1..J6]` (deg): 관절 각도
- `posx = [x, y, z, A, B, C]`: BASE 기준 TCP 위치(mm)와 방향(ZYZ 오일러, deg)
  - B ≈ 180 → 툴이 아래를 봄. 이때 A와 C는 차이(A − C)만 의미가 있습니다.
  - posx는 posj와 TCP 설정으로 계산되는 값(정기구학)이라 TCP가 바뀌면 같은 자세라도 값이 달라집니다.
