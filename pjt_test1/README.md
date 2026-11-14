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

## 좌표 메모

- `posj = [J1..J6]` (deg): 관절 각도
- `posx = [x, y, z, A, B, C]`: BASE 기준 TCP 위치(mm)와 방향(ZYZ 오일러, deg)
  - B ≈ 180 → 툴이 아래를 봄. 이때 A와 C는 차이(A − C)만 의미가 있습니다.
  - posx는 posj와 TCP 설정으로 계산되는 값(정기구학)이라 TCP가 바뀌면 같은 자세라도 값이 달라집니다.
