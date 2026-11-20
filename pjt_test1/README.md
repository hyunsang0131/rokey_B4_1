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
| `bring_test` | 침대 ↔ 트레이 같은 번호 칸으로 큐브 옮기기 | ✅ |
| `repair_test` | 자동 보수: 침대 측정 → 무른 모듈·빈칸 찾기 → 트레이에서 맞는 큐브 찾기 → 교체 → 재측정 | ✅ |

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
ros2 run pjt_test1 stiffness_test --pad             # 겉감(네오프렌)을 깔았을 때 = --force 30 --from 15 --step 0.25
```

| 옵션 | 뜻 | 기본 | `--pad` |
|---|---|---|---|
| `--force 30` | 최대로 누를 힘 [N] (12~35) | 20 | 30 |
| `--from 15` | 이 힘부터 강성 계산 [N]. 겉감이 눌리는 초반 구간을 뺀다 | 5 | 15 |
| `--step 0.25` | 닿은 뒤 한 번에 내려가는 깊이 [mm]. 단단한 큐브의 점 수를 늘린다 | 0.5 | 0.25 |
| `--down 30` | 칸 위에서 빠르게 내려가는 거리 [mm]. "이미 닿음"이 뜨면 줄인다 | 40 | - |
| `--modbus` | 그리퍼를 `/onrobot/sendCommand` 로 (기본은 DO = WebLogic) | | |

`--pad` 와 다른 옵션을 같이 주면 따로 준 값이 우선합니다 (예: `--pad --force 35`).

| 입력 | 동작 |
|---|---|
| 번호 | 그 칸 1회 측정 |
| `번호*횟수` | 같은 칸 반복 측정 (예: `4*5`, 흩어짐 ±5% 이내가 목표) |
| `a` | 전부 1회씩 |
| `n` | 지금 자리에서 힘 노이즈 측정 (접촉 기준 3N 이 충분한지) |
| `r` | 결과 표 |
| `q` | 저장 후 종료 (`~/ws_cobot_pjt/stiffness_test/`, CSV + 힘-변위 곡선 JSON) |

- 측정 순서: 그리퍼 닫기 → 칸 위 → 0.3초 쉬고 영점 → `--down` 만큼 내려가 0.3초 쉬고 영점 → 1mm 씩 내려가다 3N 넘으면 닿음 → `--step` 씩 `--force` 까지 → `--from`~`--force` 구간 직선의 기울기 = k, 힘 0 지점 = 윗면 높이 → 교시 높이로 복귀.
- 두 영점 차이가 6N(`F_PRETOUCH`) 이상이면 "이미 닿음"으로 보고 그 칸은 측정하지 않습니다. 이때는 `--down` 을 줄입니다.
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
ros2 run pjt_test1 gripper grip                # 너비로 잡기 (DO1·DO2 둘 다 ON, WebLogic 66 mm)
ros2 run pjt_test1 gripper                     # 대화형: o 열기 | c 닫기 | g 너비 잡기 | q 종료
ros2 run pjt_test1 gripper width 40 --modbus   # OnRobot 서비스로 너비 40 mm (아래 참고)
```

| 방식 | 옵션 | 경로 | 실물 결과 |
|---|---|---|---|
| **DO (기본)** | 없음 | 로봇 컨트롤러 DO1/DO2 → 그리퍼 (강의 `grip_test`와 같음) | ✅ 동작 (닫기 후 너비 0.7 mm) |
| Modbus | `--modbus` | `/onrobot/sendCommand` → OnRobot 드라이버 → Compute Box(192.168.1.1) | ❌ 명령은 전송되지만 그리퍼가 움직이지 않음 |

- DO 방식은 DO1·DO2 조합으로 WebLogic 프로그램을 고릅니다. 너비와 힘은 WebLogic에서 정합니다.

  | DO1 | DO2 | 동작 | WebLogic |
  |---|---|---|---|
  | ON | OFF | 닫기 (`close`, 강성 측정용) | 10 mm, 5 N |
  | OFF | ON | 열기 (`open`) | 80 mm, 10 N |
  | ON | ON | 너비로 잡기 (`grip`, 블록 옮기기용) | 66 mm, 20 N |

  너비로 잡기는 힘이 아니라 너비에서 멈추므로, 무른 블록도 68 → 66 mm 만큼만 눌려 덜 휩니다. 공정(`bed_inspect`)도 블록을 잡을 때 이 조합을 씁니다(`GRIP_DO_BOTH`).
- 명령을 보낸 뒤 1초 기다립니다. 그리퍼가 실제로 다 움직였는지는 확인하지 않습니다.
- `--do`는 예전 옵션 이름이라 붙여도 기본(DO)과 같습니다.
- **Modbus 방식(`--modbus`)**: 너비 지정이 가능하지만, 이 셀에서는 그리퍼가 DO로 제어되도록 설정된 것으로 보여 동작하지 않았습니다. 시험 중 OnRobot 드라이버 노드가 종료되는 일도 있었고, 그러면 브링업을 다시 실행해야 서비스가 돌아옵니다. 에뮬레이터의 가상 그리퍼는 숫자를 너비가 아니라 관절 각도(rad)로 해석하므로, 에뮬레이터에서 `width`는 RViz 모양이 실제와 다릅니다.

## bring_test

침대 칸의 큐브를 교체 트레이의 같은 번호 칸으로 옮깁니다(그리퍼 실제 동작). 좌표는 `teach_points/all_coordinate.json`(침대 `pin_b*` + 트레이 `tray_b*`, 36개)입니다.

```bash
ros2 run pjt_test1 bring_test            # 대화형
ros2 run pjt_test1 bring_test --close    # 잡을 때 닫기(DO1)만. 기본은 너비 잡기(DO1+DO2)
ros2 run pjt_test1 bring_test --plan     # 로봇 없이 동작 순서만 출력
```

| 입력 | 동작 |
|---|---|
| `1`~`6` | 그 칸: 침대 → 트레이 |
| `r1`~`r6` | 그 칸: 트레이 → 침대 |
| `a` | 전체 실행: 침대 → 트레이, 1 → 6 |
| `r` | 역순 전체 실행: 트레이 → 침대, 6 → 1 |

- 한 칸: 출발 칸 위 → 열기 → 내려가기 → 잡기 → 올라오기 → 도착 칸 위 → 내려가기 → 0.3 s 대기 → 놓기 → 올라오기. 전부 movel(posx)이라 TCP가 `GripperDA_v1`이어야 합니다.
- 이 좌표 자세는 큐브 긴 쪽(68 mm)을 잡습니다. WebLogic이 긴 쪽 설정(열기 80 / 잡기 66)이어야 합니다.

## repair_test

침대를 검사하고, 무른 모듈(10A)과 빈칸을 트레이의 맞는 큐브로 바꾼 뒤 다시 검사합니다. 좌표는 `teach_points/bed_fin.json` + `exchange_fin.json`입니다.

```bash
ros2 run pjt_test1 repair_test                  # 교체 계획을 보여 주고 y 를 받으면 실행 (트레이는 실행마다 처음에 한 번 잼)
ros2 run pjt_test1 repair_test --use-map        # 트레이를 재지 않고 저장된 강성 지도를 이어서 사용
ros2 run pjt_test1 repair_test --pad            # 침대에 겉감(네오프렌)이 있을 때 (침대 측정만 --force 30 --from 12 --step 0.25)
ros2 run pjt_test1 repair_test --show           # 저장된 강성 지도만 보기 (로봇 필요 없음)
ros2 run pjt_test1 repair_test --plan --sim-bed 12.3,12.2,-,7.0,12.4,12.1 --sim-tray 7.0,12.3,6.9,12.2,-,12.4
                                                # 로봇 없이 가짜 강성으로 판정·계획만 ('-' = 빈칸, --sim-tray 를 빼면 지도 사용)
```

**강성 지도** (`~/ws_cobot_pjt/repair_test/state.json`): 침대·트레이 각 칸에 지금 어떤 강성의 큐브가 있는지.

| 언제 | 지도에 일어나는 일 |
|---|---|
| 침대 측정 / 재측정 | 침대 6칸을 잰 값으로 덮어씀 |
| 트레이 측정 (실행마다 한 번, `--use-map`이면 생략) | 트레이 6칸을 잰 값으로 덮어씀 |
| 큐브 하나 옮길 때마다 | 출발 칸은 빈칸, 도착 칸에 강성이 따라감 (트레이↔침대는 `--rack-to-bed`로 환산한 **추정값**, "옮겨 옴"으로 표시) |
| 옮기는 도중에 멈춤 | 그 두 칸을 "확인 필요"로 표시 → 큐브 위치를 확인하고 다시 실행 (트레이를 새로 잼) |

- 매번 바로 파일에 저장해서, 중간에 멈춰도 그때까지 끝난 이동까지만 반영됩니다.
- **한 번 실행 = 한 시나리오**: 트레이는 처음에 한 번만 재고, 그 뒤로는 큐브를 옮길 때마다 지도를 고쳐 가며 씁니다. 다음 실행은 트레이를 다시 재므로 사이에 손으로 큐브를 옮겨도 됩니다.
- `--use-map`이면 트레이를 재지 않고 **지도를 그대로 믿습니다.** 손으로 큐브를 옮겼다면 쓰지 마세요.

| 단계 | 하는 일 |
|---|---|
| 1. 침대 측정 | `pin_b1~6`을 `stiffness_test`와 같은 방법(`press_at`, 스텝 0.25 mm)으로 누름 |
| 2. 판정 | **빈칸**: 닿지 않음(접근 높이에서 침대 16 mm, 트레이 20 mm 내려가도 3 N 미만 → 바닥까지 가지 않고 바로 올라옴. 큐브 윗면보다 약 8 mm 아래), 또는 접촉 높이가 다른 칸보다 8 mm 이상 낮음. **맞지 않음**: 강성을 가장 크게 벌어진 곳에서 두 무리로 나눠 무른 무리 (가장 단단한 / 가장 무른 이 1/`--ratio` 미만이면 모두 정상). 아주 깊이 들어감·키 큰 큐브도 맞지 않음 |
| 3. 끝? | 빈칸·맞지 않음이 없으면 "교체할 것 없음"으로 끝 |
| 4. 트레이 | `tray_b1~6`을 눌러서 빈 자리와 후보 강성을 잼 (`--use-map`이면 저장된 강성 지도 사용) |
| 5. 고르기 | 채울 칸마다 **상하좌우 이웃 칸 평균 강성**과 비율이 가장 가까운 트레이 큐브 (`bed_math.plan_fill`). 그래도 `--ratio` 밖이면 시작하지 않음 |
| 6. 이동 순서 | 빈칸 먼저 채움 → 맞지 않는 칸은 [그 모듈을 트레이 빈 자리로] → [고른 큐브를 침대로] (`bed_math.plan_repair_moves`) |
| 7. 교체 | 계획을 보여 주고 `y`면 실행. 잡기/놓기는 `bring_test`와 같은 순서 |
| 8. 재측정 | 침대 6칸을 다시 눌러 문제가 남은 칸을 알려 줌. 옮긴 큐브로 `--rack-to-bed` 제안 값 계산. 마지막에 강성 지도 출력 |

칸 배치(이웃 계산)는 좌표 그대로 `1 2 / 3 4 / 5 6`(3행 2열)입니다.

| 옵션 | 뜻 | 기본 |
|---|---|---|
| `--ratio 0.75` | 두 무리로 나눌 최소 강성 비율(1/0.75 = 1.33배). 실측 40A ≈ 12.3, 10A ≈ 7.0 N/mm (1.76배) | 0.75 |
| `--rack-to-bed 1.0` | 트레이 k → 침대 k 환산. 겉감이 있으면 1이 아님. 재측정 후 제안 값이 나옴 | 1.0 |
| `--pad` (`--mat`) | 침대 측정만 겉감 설정(30 N, 12 N부터, 0.25 mm) = `stiffness_test --force 30 --from 12 --pad` | |
| `--yes` | 계획을 묻지 않고 실행 | |
| `--use-map` | 트레이를 재지 않고 저장된 강성 지도 사용 | 끔 (매번 잼) |
| `--show` | 강성 지도만 보기 | |
| `--bed`, `--tray` | 좌표 파일 바꾸기 | fin 파일 |

- **트레이에 빈 자리가 1개 이상** 있어야 맞지 않는 모듈을 뺄 수 있습니다. 침대에 빈칸이 있으면 그 칸을 먼저 채우면서 트레이 자리가 생깁니다.
- 기록: `~/ws_cobot_pjt/repair_test/repair_날짜_시간.json` (측정값, 계획, 실제로 끝난 이동, 결과). 지금 상태는 `state.json`
- 에뮬레이터(sodvir)에서 흐름 확인: `REPAIR_EMU="bed=H,H,-,S,H,H;tray=S,H,S,H,-,H" ros2 run pjt_test1 repair_test` (H 40A, S 10A, - 빈칸). 가짜 힘이라 자세·충돌은 확인할 수 없습니다.

## 좌표 메모

- `posj = [J1..J6]` (deg): 관절 각도
- `posx = [x, y, z, A, B, C]`: BASE 기준 TCP 위치(mm)와 방향(ZYZ 오일러, deg)
  - B ≈ 180 → 툴이 아래를 봄. 이때 A와 C는 차이(A − C)만 의미가 있습니다.
  - posx는 posj와 TCP 설정으로 계산되는 값(정기구학)이라 TCP가 바뀌면 같은 자세라도 값이 달라집니다.
