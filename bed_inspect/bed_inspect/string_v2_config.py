# 나사로 실 고정 / 고정해제 설정 (2026-11-21 형님 손그림 "나사 실 고정 및 고정해제").
#
# 판 위 나사 3개 (BASE, TCP GripperDA_v1 = 그리퍼 손끝 기준):
#   실 고정 나사  (274.8, -223.93)  실 한쪽 끝이 묶여 있음 (로봇은 안 건드림)
#   가운데 구멍   (274.8,  -73.93)  실 끝 나사를 꽂아 조이는 곳
#   감는 나사     (274.8,   76.07)  실을 2바퀴 감는 기둥
# 실 다른 끝에 묶인 T 나사를 그리퍼로 잡고 → 감는 나사 둘레를 move_spiral 로 2바퀴 → 가운데 구멍에 꽂고
# "잡고 한 방향 180도 → 놓기 → 살짝 위 → 그리퍼만 되돌리기 → 다시 잡기" 반복해서 조인다.
# 블록은 사람이 -73.93 ~ 76.07 사이에 놓는다 (블록 있으면 실이 장력으로 블록을 잡음, 없으면 실만 고정).
# 고정해제: 끝 나사는 가운데 구멍에 끝까지 안 조이고 대략 꽂아 둔 상태 → 잡아서 뽑고 → 반대로 풀고 → 놓기.
#
# ※ 아직 실측/확인이 필요한 값에는 "확인 필요" 표시. 값은 환경변수로도 바꿀 수 있다 (STR_...).

import os


def _f(name, default):
    v = os.environ.get(name)
    return float(v) if v not in (None, '') else default


def _xyz(name, default):
    v = os.environ.get(name)
    if v in (None, ''):
        return default
    vals = [float(s) for s in v.replace(',', ' ').split()]
    if len(vals) != 3:
        raise ValueError(f'{name} 는 x,y,z 3개여야 합니다: {v!r}')
    return vals


STR_TCP = 'GripperDA_v1'                 # 좌표를 잡은 TCP (그리퍼 손끝)

# ── 판 위 위치 [mm] ───────────────────────────────────────────────────
TABLE_Z = 65.0                           # 탁자 표면 높이 (형님: 로봇 베이스 기준 65)
ANCHOR_XY = (274.8, -223.93)             # 실 고정 나사 (참고용)
HOLE_XY = (274.8, -73.93)                # 가운데 나사 구멍
POST_XY = (274.8, 76.07)                 # 실을 감는 나사 (형님 11-21: 그림의 96.07 아님, 76.07)

# 툴 자세 (툴 Z 가 아래를 보는 수직 자세). 팀원 침대 교시 자세와 같은 값 (확인 필요: 손가락 방향)
STR_ORI = [168.25, 179.97, -11.77]

# ── 나사 ──────────────────────────────────────────────────────────────
# 나사 머리 잡는 점에서 나사 끝까지 [mm]. 11-21 08:20 형님: 구멍에 꽂힌 나사 머리를 잡는 높이가 z 71.08 (탁자 65 보다 6mm 위)
# → 나사는 구멍 안에 들어가 있는 짧은 나사. '230' 은 23.0mm 로 해석 (확인 필요, 환경변수 STR_SCREW_LEN)
SCREW_LEN = _f('STR_SCREW_LEN', 23.0)
# 잡은 나사 끝이 탁자에 안 끌리는 여유 → 감을 때 TCP 높이 = 탁자 + 나사 길이 + 여유
WIND_CLEAR = _f('STR_WIND_CLEAR', 8.0)
HOLE_GRIP_Z = _f('STR_HOLE_GRIP_Z', 71.08)   # 구멍에 꽂힌 나사 머리를 잡는 TCP 높이 (형님 11-21 08:20)
# 실 끝 나사를 처음 잡는 위치: 가운데 구멍 중앙, 머리 잡는 높이 (형님 11-21 08:20: 처음엔 구멍에 대략 꽂혀 있음)
PICK_XYZ = _xyz('STR_PICK_XYZ', None) or [HOLE_XY[0], HOLE_XY[1], HOLE_GRIP_Z]
# 나사 피치 [mm/회전]. 머리 지름 12.7mm → M8 렌치볼트(머리 13mm)로 보고 1.25 (확인 필요, STR_SCREW_PITCH)
SCREW_PITCH = _f('STR_SCREW_PITCH', 1.25)
APPROACH_DZ = 60.0                       # 잡기/꽂기 전 위에서 접근하는 높이 [mm]
CARRY_DZ = 20.0                          # 잡은 뒤 들어 올리는 높이 (실이 걸리지 않게 조금만) [mm]
INSERT_GAP = _f('STR_INSERT_GAP', 3.0)   # 구멍 위 이 높이까지 위치 이동으로 내려가고, 그 뒤는 순응 제어 + 누르는 힘으로 닿게 함 [mm]

# ── 감기 (move_spiral) ────────────────────────────────────────────────
# 'inward' : 바깥 30 → 안쪽 20 (아이스크림). 두산 move_spiral 안쪽형은 목표점이 중심 → 목표 = 감는 나사.
#            반지름은 0 까지 줄어드는 나선이라, 2바퀴(반지름 20) 돌았을 때 멈춘다 (비동기 + 각도 감시).
# 'outward': 좁게 → 넓게. 두산 바깥형은 "현재 위치"가 중심 → 감는 나사 바로 위(머리보다 위)에서 시작해야 함.
#            잡은 나사 끝이 감는 나사 머리보다 높아야 안 부딪힘 → OUTWARD_Z 를 꼭 줘야 함.
SPIRAL_MODE = os.environ.get('STR_SPIRAL_MODE', 'inward')
# ── v2 감기 (11-21 08:46 형님: v2 는 move_spiral 없음) ─────────────────
# 뽑은 나사를 z 높이 올린 채로 → 실이 팽팽해질 때까지(장력 느낄 때까지) 이동 → 그 반지름으로 감는 나사 둘레를 크게 돈다.
WIND2_Z = _f('STR_WIND2_Z', 200.0)        # 돌 때 TCP 높이 [mm] ("z 를 많이 올린 채로")
TENSION_N = _f('STR_TENSION_N', 3.0)      # 이만큼(수평 외력 변화) 느끼면 팽팽하다고 보고 멈춤 [N]
TENSION_MAX = _f('STR_TENSION_MAX', 400.0)    # 장력 찾으며 최대 이동 거리 [mm]
TENSION_VEL = (20.0, 20.0)                # 장력 찾는 이동 속도
# 장력 찾는 방향 (베이스 xy 단위벡터). 기본 = 실 고정 나사 → 구멍 방향 (+y, 감는 나사 쪽). 감는 나사 위를 높이 넘어감
TENSION_DIR = None
VIRTUAL_TENSION_R = _f('STR_VIRTUAL_R', 120.0)   # RViz 가상 모드에서 장력 자리 대신 쓰는 반지름 [mm]
CIRCLE_TURNS = _f('STR_CIRCLE_TURNS', 2.0)    # 감는 바퀴 수
CIRCLE_VEL = (40.0, 40.0)
CIRCLE_ACC = (80.0, 80.0)
# 2026-11-21 08:00 형님(영상 확인 후): 실이 팽팽한 상태 기준 큰 원 30cm, 작은 원 26cm (처음엔 30mm/20mm 였음)
SPIRAL_R_OUT = _f('STR_R_OUT', 300.0)    # 넓은 반지름 [mm]
SPIRAL_R_IN = _f('STR_R_IN', 260.0)      # 좁은 반지름 [mm]
SPIRAL_TURNS = 2.0                       # 감는 바퀴 수 (형님: 총 2바퀴)
SPIRAL_RISE = _f('STR_SPIRAL_RISE', 8.0)     # 감는 동안 z 상승 [mm] (형님: 눈에 보이게 5~10)
SPIRAL_CCW = True                        # 위에서 볼 때 반시계 (형님)
SPIRAL_START_DIR_DEG = _f('STR_START_DIR', -90.0)   # 감기 시작점이 감는 나사에서 어느 쪽인지 (베이스 xy 각도, -90 = -y 쪽 = 구멍 쪽)
# 감는 길이 로봇 베이스(원점) 근처를 지나면 안 됨 (로봇 몸통·J1 축 특이점). 감는 나사는 베이스에서 약 285mm
BASE_CLEAR = _f('STR_BASE_CLEAR', 150.0)  # 감는 길과 베이스 중심 최소 거리 [mm] (높이 올린 채라 150)
REACH_MAX = 800.0                        # 감는 길과 베이스 중심 최대 거리 [mm] (M0609 도달 900 에서 여유)
ANCHOR_CLEAR = 30.0                      # 감는 길과 실 고정 나사(서 있음) 최소 거리 [mm]
# 11-21 실물: 40mm/s 로 보냈더니 컨트롤러 거부 (알람 1209 "Too High Circular Acc 8048 > 3000 mm/s2").
# 안쪽 나선은 끝이 반지름 0 이라 컨트롤러가 그 부근 원심 가속도(v^2/r)로 검사 → 속도 제곱에 비례 → 18mm/s 면 약 1630 (한계 3000)
SPIRAL_VEL = (_f('STR_SPIRAL_VEL', 18.0), 18.0)   # [mm/s, deg/s]
SPIRAL_ACC = (36.0, 36.0)
SPIRAL_STOP_LEAD_DEG = 8.0               # 감속 거리만큼 일찍 정지 명령 [deg]
SPIRAL_MIN_GAP = 12.0                    # 감는 나사 중심과 이보다 가까워지면 즉시 정지 (두 나사 굵기 + 여유) [mm]
SPIRAL_WRONG_DIR_DEG = 30.0              # 반대 방향으로 이만큼 돌면 즉시 정지 (rot_dir 부호 확인용)
SPIRAL_ROT_DIR_CCW = 0                   # 두산 rot_dir: 0 FORWARD(+) / 1 REVERSE(-). +Z 오른손 = 위에서 반시계로 가정 (확인 필요)
OUTWARD_Z = _f('STR_OUTWARD_Z', None)    # 'outward' 때 감는 TCP 높이 (감는 나사 머리보다 위). None 이면 outward 거부

# ── 조이기 (180도 반복) ───────────────────────────────────────────────
# 한 번에 돌리는 각도. 그리퍼가 한 방향 무한 회전이 안 돼서 잡았다 놓았다 반복. 180도로는 살짝도 안 물린다(형님 11-21)
# → J6 범위 안에서 크게. 돌리기 전에(공중에서) J6 를 반대쪽으로 미리 돌려 둔다
TIGHTEN_TURN_DEG = _f('STR_TURN_DEG', 330.0)
J6_LIMIT = 355.0                         # J6 가동 범위 ±360 에서 여유 [deg]
ROT_CHUNK_DEG = 110.0                    # 툴 회전 movel 한 번의 최대 각도 (큰 각도는 짧은 쪽으로 돌 수 있어 나눔)
TIGHTEN_REPS = int(_f('STR_TIGHTEN_REPS', 4))   # 반복 횟수 (확인 필요)
TIGHTEN_SIGN = 1                         # 툴 Z(아래 방향) 기준 +회전 = 위에서 봐서 시계 = 오른나사 조임 (확인 필요)
TIGHTEN_LIFT = 5.0                       # 놓은 뒤 살짝 올리는 높이 [mm] (형님: z 로 잠시 떼어냄)
TIGHTEN_VEL = (30.0, 60.0)               # 툴 회전 movel 속도 [mm/s, deg/s]
TIGHTEN_ACC = (60.0, 120.0)
# 꽂기·돌리기 동안 순응 제어 (형님 11-21: 처음 구멍에 놓을 때 순응 제어로 정확히 돌려서 고정).
# x·y 부드럽게 → 나사 끝이 구멍 가운데로 스스로 맞춰짐, z 부드럽게 + 아래로 일정한 힘 → 나사산이 물리면서 내려가는 만큼 따라감,
# 툴 Z 회전(rz)은 단단하게 → 돌리는 각도는 정확히. 강성 [N/m x3, Nm/rad x3]
TIGHTEN_COMPLIANCE = [500.0, 500.0, 500.0, 300.0, 300.0, 300.0]
TIGHTEN_PUSH_N = _f('STR_PUSH_N', 5.0)   # 돌리는 동안 아래로 누르는 힘 [N] (set_desired_force, 베이스 -Z)
TIGHTEN_SETTLE = 0.5                     # 순응 제어 켠 뒤 기다림 [s] (켠 직후 명령 무시되던 문제, 커버 때와 같음)

# ── 고정해제 ──────────────────────────────────────────────────────────
# 뽑기 전에 잡은 채로 반대로 돌려 풀기 횟수 (고정할 때 처음 잡을 때도, 고정해제 때도). 11-21 형님: 안 돌리면 안 빠짐
LOOSEN_REPS = int(_f('STR_LOOSEN_REPS', TIGHTEN_REPS))
# 풀 때 순응: z 만 부드럽게 (나사가 올라오는 만큼 따라감). 누르지 않음
LOOSEN_COMPLIANCE = [3000.0, 3000.0, 200.0, 300.0, 300.0, 300.0]
PULL_DZ = _f('STR_PULL_DZ', 40.0)        # 잡은 뒤 위로 뽑는 거리 [mm] (나사 끝이 구멍에서 빠지는 높이, 확인 필요)
UNWIND_RADIUS = _f('STR_UNWIND_R', 120.0)  # 풀 때 감는 나사 둘레를 도는 반지름 [mm] (높이 WIND2_Z 에서)
UNWIND_VEL = (40.0, 40.0)
UNWIND_ACC = (80.0, 80.0)
# 풀고 난 뒤 나사를 내려놓을 곳. None 이면 PICK_XYZ (처음 잡은 자리)
# 풀고 난 나사를 놓을 위치. 기본 = 처음 자리(가운데 구멍에 대략 꽂아 둠)
PLACE_XYZ = _xyz('STR_PLACE_XYZ', None) or list(PICK_XYZ)

# ── 일반 이동 ─────────────────────────────────────────────────────────
MOVE_VEL, MOVE_ACC = 60.0, 120.0         # 직선 이동 [mm/s, mm/s^2]
SLOW_VEL, SLOW_ACC = 20.0, 60.0          # 잡으러 내려갈 때 / 꽂을 때
# 그리퍼: 나사를 돌려야 해서 서비스 모드(OnRobot 드라이버, RG2 최대 40N) 기본. io(WebLogic DO1 5N)는 미끄러짐 (11-21 실물)
GRIPPER_MODE = os.environ.get('STR_GRIPPER_MODE', 'service')
GRIP_WIDTHS = {'screw_open': 25, 'screw_grip': 10}   # 머리 지름 12.7mm: 10mm 까지 조임(머리에서 멈춤), 25mm 벌림
GRIP_KEY = 'screw_grip'                  # 그리퍼 잡기 (io 모드: WebLogic 프리셋, bed_config.GRIP_WIDTH 에 추가)
OPEN_KEY = 'screw_open'                  # 그리퍼 놓기
