# 실리콘 침대 검사 공정 설정값. 현장에서 바꿀 숫자는 전부 이 파일에만 있다.
# 단위: mm, deg, N, s

import os

ROBOT_ID = 'dsr01'
ROBOT_MODEL = 'm0609'

# ── 파일 위치 ─────────────────────────────────────────────────────────
# 교시 파일: robot_setting direct_teach 가 만든 JSON 을 이 이름으로 복사해 둔다.
# (환경변수 BED_TEACH_FILE 이나 실행 인자로 다른 파일을 지정할 수도 있다)
TEACH_FILE = os.path.expanduser('~/ws_cobot_pjt/teach_points/bed.json')
DATA_DIR = os.path.expanduser('~/ws_cobot_pjt/bed_data')   # 재고, 기준값, 측정 기록이 저장되는 곳
SIM_TEACH_FILE = os.path.expanduser('~/ws_cobot_pjt/teach_points/bed_sim.json')   # bed_sim 이 만드는 가짜 교시 파일
SIM_DATA_DIR = DATA_DIR + '_sim'                           # 가짜 로봇으로 돌릴 때는 기록을 따로 둔다

# ── 교시 위치 이름 (direct_teach 에서 이 이름으로 저장) ───────────────
# 침대 칸 / 보관대 / 버리는 곳은 "블록을 든 채로 지나가도 안 부딪히는 높이"에서 교시한다.
# 거기서 아래로 내려가는 양은 아래 *_DZ 값으로 정한다 (전부 베이스 Z 상대 이동).
P_HOME = 'home'
P_BED = ['bed_1', 'bed_2', 'bed_3', 'bed_4', 'bed_5', 'bed_6']       # 각 블록 중심 위 (툴이 아래를 봄)
P_RACK = ['rack_1', 'rack_2', 'rack_3', 'rack_4', 'rack_5', 'rack_6', 'rack_7', 'rack_8']  # 보관대 자리 위
P_DISCARD = 'discard'                                                # 불량 블록 버리는 통 위
# 아래 3개는 "잡는 자세 그대로" 교시하고, 각각 5cm 쯤 뒤로 뺀 접근점(_app)도 같이 교시한다.
P_ROLLER = 'roller'               # 롤러 손잡이를 잡은 자세      (+ roller_app)
P_COVER_GRIP = 'cover_grip'       # 닫힌 커버 손잡이를 잡은 자세  (+ cover_grip_app)
P_COVER_OPEN = 'cover_open'       # 다 열린 커버 손잡이를 잡은 자세 (+ cover_open_app)
P_COVER_VIA = 'cover_via'         # 커버가 반쯤 열렸을 때 손잡이를 잡은 자세 (movec 경유점)
APP_SUFFIX = '_app'

# ── 침대 배치 ─────────────────────────────────────────────────────────
# 번호는 1행 왼쪽부터:  1 2 3
#                      4 5 6
BED_ROWS, BED_COLS = 2, 3

# ── 블록 종류와 처음 재고 ─────────────────────────────────────────────
BLOCK_HEIGHTS = [25.0, 30.0, 35.0]          # 준비한 블록 높이 3가지
H_REF = 30.0                                # *_PICK_DZ 를 맞춘 기준 높이
# 처음 실행할 때만 쓰는 값. 이후에는 DATA_DIR/inventory.json 에 기록된 것을 쓴다.
BED_INIT = [30.0, 30.0, 30.0, 30.0, 30.0, 30.0]
RACK_INIT = [25.0, 25.0, 25.0, 35.0, 35.0, 35.0, 30.0, None]   # None = 빈 자리 (최소 1개 필요)

# ── 이동 속도 ─────────────────────────────────────────────────────────
VELJ, ACCJ = 30, 30               # movej 일반 [deg/s, deg/s^2]
VELJ_SLOW, ACCJ_SLOW = 10, 20     # 접근점 ↔ 잡는 자세
VELX, ACCX = 60, 120              # 수직 이동 movel [mm/s, mm/s^2]
VELX_SLOW, ACCX_SLOW = 20, 60     # 블록을 잡으러 내려갈 때 / 놓을 때

# ── 그리퍼 (OnRobot RG2) ──────────────────────────────────────────────
# 'service': /onrobot/sendCommand 서비스 (m0609_rg2_bringup 이 띄우는 드라이버). 폭 지정 가능
# 'io'     : 컨트롤러 디지털 출력 (DO_CLOSE / DO_OPEN). 열기/닫기만 가능
GRIPPER_MODE = os.environ.get('BED_GRIPPER_MODE', 'service')   # 환경변수 BED_GRIPPER_MODE=io 로도 바꿀 수 있다
GRIP_WIDTH = {                    # 손가락 사이 폭 [mm] ('service' 모드에서만 의미 있음)
    'press': 0,                   # 누르기 측정: 완전히 닫고 손가락 끝으로 누른다
    'roller_open': 60, 'roller_grip': 20,
    'cover_open': 60, 'cover_grip': 10,
    'block_open': 70, 'block_grip': 35,     # 블록 폭보다 넓게 벌리고, 조금 좁게 잡는다
}
GRIP_WAIT = 1.2                   # 그리퍼 명령 후 대기 [s]
DO_CLOSE, DO_OPEN = 1, 2          # 'io' 모드: DO1=ON 닫힘, DO2=ON 열림

# ── 누르기 측정 ───────────────────────────────────────────────────────
PRESS_APPROACH_DZ = -50.0         # 교시 높이에서 빠르게 내려오는 양. 끝난 높이가 가장 높은 블록보다 5mm 이상 위여야 함
SEARCH_STEP = 1.0                 # 접촉 전 한 스텝 [mm]
PRESS_STEP = 0.5                  # 접촉 후 한 스텝 [mm]
STEP_VEL, STEP_ACC = 10, 40       # 스텝 이동 속도
PRESS_SETTLE = 0.3                # 접근 후 영점 잡기 전 대기 [s]
STEP_SETTLE = 0.1                 # 스텝 후 대기 [s]
TARE_SAMPLES = 15                 # 영점 읽기 횟수
READ_SAMPLES = 3                  # 스텝마다 읽기 횟수
F_CONTACT = 3.0                   # 접촉 판단 [N]
F_FIT_MIN = 5.0                   # 강성 계산에 쓰는 최소 힘 [N] (천이 늘어나는 초반 구간 제외)
F_TARGET = 20.0                   # 여기까지 누른다 [N]
F_LIMIT = 40.0                    # 넘으면 즉시 중단 [N]
SEARCH_MAX = 30.0                 # 접촉 없이 내려갈 수 있는 최대 거리 [mm]
D_MAX = 25.0                      # 접촉 후 최대 압입 깊이 [mm]
MIN_FIT_POINTS = 4                # 강성 계산에 필요한 최소 점 수
MIN_R2 = 0.90                     # 직선 맞춤 품질이 이보다 낮으면 "측정 불량"으로 표시

# ── 불량 판정 ─────────────────────────────────────────────────────────
WARN_RATIO = 0.15                 # 기준 강성과 15% 이상 차이 → 주의
FAIL_RATIO = 0.30                 # 30% 이상 차이 → 불량 (교체 대상)

# ── 롤러 (천 팽팽하게 / 풀기) ─────────────────────────────────────────
ROLLER_TURN_DEG = 90.0            # 한 번 잡고 돌리는 각도 (J6). + 방향이 팽팽하게
ROLLER_TURNS = 2                  # 다시 잡고 돌리는 횟수
ROLLER_VEL, ROLLER_ACC = 30, 30
J6_LIMIT = 350.0                  # J6 가 이 범위를 넘게 되면 돌리기 전에 멈춘다

# ── 커버 열기 / 닫기 (movec) ──────────────────────────────────────────
COVER_VEL, COVER_ACC = 30, 60     # 천천히

# ── 블록 집기 / 놓기 ──────────────────────────────────────────────────
BED_PICK_DZ = -75.0               # bed_N 교시 높이 → 기준 높이(H_REF) 블록을 잡는 높이
RACK_PICK_DZ = -75.0              # rack_N 교시 높이 → 기준 높이 블록을 잡는 높이
PLACE_CLEAR = 2.0                 # 놓을 때는 잡은 높이보다 이만큼 위에서 놓는다

# ── 기울기 맞춤 ───────────────────────────────────────────────────────
TILT_PLANE_TOL = 0.5              # 블록 윗면이 한 평면에서 이만큼 [mm] 이내여야 "평평한 기울기"로 인정
TILT_ALLOW_STEP = False           # True 면 계단 모양(완전한 평면이 아닌) 조합도 후보에 넣는다
TILT_OK_DEG = 1.0                 # 재측정 기울기가 목표와 이만큼 이내면 성공으로 표시

# ── 12단계 이름 (화면 표시용) ─────────────────────────────────────────
STEP_NAMES = [
    '', '홈 위치', '천 팽팽하게', '실리콘 측정', '불량품 확인', '천 느슨하게',
    '커버 열기', '블록 빼기', '블록 넣기', '커버 닫기', '천 팽팽하게', '재측정', '정상 확인',
]
