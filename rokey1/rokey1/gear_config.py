"""gear_config.py - 썬기어(유성기어) 조립 설정값 모음.

현장에서 바꿀 숫자는 전부 이 파일에만 있습니다.
좌표는 2026-11-11 DART 플랫폼에서 성공한 m0609_gearassembly_final_2 티칭값에서 계산했습니다.
단위: mm, deg, N, s. 좌표계: 로봇 베이스(DR_BASE).
"""

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# ── 툴/TCP ────────────────────────────────────────────────────────────
# None 이면 컨트롤러에 지금 설정된 값을 건드리지 않습니다(로그로만 출력).
# 좌표는 "DART에서 티칭할 때 켜져 있던 TCP" 기준이라, 그때와 같아야 합니다.
# 문자열을 넣으면 그 이름으로 설정하고, 설정이 안 되면 움직이지 않고 종료합니다.
TOOL_NAME = "Tool Weight_1"     # grip_test.py / move.py 와 동일
TCP_NAME = "GripperDA_v1"       # 2026-11-11 DART 티칭 때 사용한 TCP

# ── 속도 ──────────────────────────────────────────────────────────────
VELX = (250.0, 80.625)      # 이동 속도 (직선 mm/s, 회전 deg/s)  - 원본과 동일
ACCX = (1000.0, 322.5)      # 이동 가속도                        - 원본과 동일
VEL_FINE = 80.0             # 집기/놓기 직전 수직 하강 속도
ACC_FINE = 300.0

# ── 테스트 모드 ───────────────────────────────────────────────────────
# 0보다 크면 모든 Z를 그만큼 띄워서 허공에서 경로만 확인합니다.
# 이때 힘 제어 삽입은 건너뛰고, 그립 감지 실패도 에러로 보지 않습니다.
Z_TEST_OFFSET = 0.0

# ── 기준점 (절대 좌표는 이 두 개뿐) ───────────────────────────────────
HOME = (367.23, 7.44, 248.81, 137.16, 179.94, 137.01)
TRAY_ANCHOR_XY = (574.27, -147.87)      # 트레이의 썬기어 자리(p4). 지그를 옮기면 이 점만 다시 티칭
TOOL_ORI = (160.93, 180.00, 160.08)     # 모든 동작 공통 자세(툴이 아래를 봄)

# 트레이 → 조립 지그 평행이동 (베이스 기준 상대 이동량)
TRAY_TO_ASM = (-300.0, 150.0)

# ── 높이 ──────────────────────────────────────────────────────────────
# 유성기어 집는 높이: 어제 성공한 실측값을 자리별로 그대로 사용
# (하나로 통일하면 planet2는 6mm 이상 높게 집게 되어 놓칠 수 있음)
PLANET1_PICK_Z = 108.28
PLANET2_PICK_Z = 101.98
PLANET3_PICK_Z = 106.23
PLANET_PLACE_Z = 110.5      # 유성기어 놓는 높이. 실측 110.23~110.98
PLANET_LIFT = 90.0          # 집은 뒤 들어올리는 양
SUN_PICK_Z = 100.35         # 썬기어 집는 높이(실측 그대로. 부품이 달라서 통일하지 않음)
SUN_LIFT = 200.0
SUN_INSERT_START_Z = 123.5  # 힘 제어를 시작하는 높이
RETREAT_AFTER_INSERT = 100.0

# ── 기어 자리 (기준점에서의 상대 오프셋 dx, dy) ───────────────────────
# 썬기어 중심에서 반지름 약 60mm, 120도 간격.
SLOTS = [
    {"name": "planet1", "offset": (60.17, 2.84), "pick_z": PLANET1_PICK_Z, "lift": PLANET_LIFT, "task": "place"},
    {"name": "planet2", "offset": (-26.94, -53.03), "pick_z": PLANET2_PICK_Z, "lift": PLANET_LIFT, "task": "place"},
    {"name": "planet3", "offset": (-33.10, 52.12), "pick_z": PLANET3_PICK_Z, "lift": PLANET_LIFT, "task": "place"},
    {"name": "sun", "offset": (0.0, 0.0), "pick_z": SUN_PICK_Z, "lift": SUN_LIFT, "task": "insert"},
]

# ── 그리퍼 (OnRobot RG2, WebLogic I/O) ────────────────────────────────
DO_CLOSE = 1                # DO1=ON, DO2=OFF 이면 닫힘
DO_OPEN = 2                 # DO1=OFF, DO2=ON 이면 열림
DI_GRIP = 1                 # DI1=ON 이면 물체 잡힘 감지
GRIP_TIMEOUT = 3.0          # 잡힘 감지를 기다리는 최대 시간
GRIP_SETTLE = 0.2           # 감지 후 안정화 대기
RELEASE_WAIT = 0.8          # 열림 명령 후 대기(열림 완료 신호가 없어서 시간으로 대기)
STRICT_GRIP_CHECK = True    # True: 집기 실패 시 즉시 정지 (virtual 에서는 DI가 안 들어오므로 False)
# I/O와 함께 /onrobot/sendCommand 서비스도 호출 (grip_test.py 와 동일).
# virtual 모드에서 RViz의 RG2를 움직이려면 필요. 서비스가 없으면 한 번 경고하고 I/O만 사용.
USE_GRIPPER_SERVICE = True
GRIPPER_SERVICE = "/onrobot/sendCommand"

# ── 썬기어 힘 제어 삽입 (원본 값 그대로) ──────────────────────────────
INSERT_STIFFNESS = [3000.0, 3000.0, 200.0, 200.0, 200.0, 200.0]
INSERT_FORCE = [0.0, 0.0, -10.0, 0.0, 0.0, 0.0]     # 베이스 Z 아래로 10N
INSERT_FORCE_DIR = [0, 0, 1, 0, 0, 0]
WIGGLE_AMP = [0.0, 0.0, 0.0, 0.0, 0.0, 10.0]        # 툴 Z축 회전 ±10도
WIGGLE_PERIOD = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
WIGGLE_ATIME = 0.2
WIGGLE_REPEAT = 2
INSERT_JAM_FORCE = 20.0     # 이 이상 반력이면 흔들기 종료
INSERT_Z_MIN = 95.64        # Z가 이 구간에 들어오면 맞물린 것으로 판단
INSERT_Z_MAX = 115.64
INSERT_SEAT_FORCE = 9.0     # 바닥에 안착했다고 보는 반력
INSERT_WIGGLE_TIMEOUT = 30.0    # 흔들기 최대 시간 (원본은 무한 대기)
INSERT_SEAT_TIMEOUT = 10.0      # 안착 대기 최대 시간 (원본은 무한 대기)
