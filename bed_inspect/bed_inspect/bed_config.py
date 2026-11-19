# 실리콘 침대 검사 공정 설정값. 현장에서 바꿀 숫자는 전부 이 파일에만 있다.
# 단위: mm, deg, N, s

import os

ROBOT_ID = 'dsr01'
ROBOT_MODEL = 'm0609'

# ── 파일 위치 ─────────────────────────────────────────────────────────
# 교시 파일: robot_setting direct_teach 가 만든 JSON 을 이 이름으로 복사해 둔다.
# (환경변수 BED_TEACH_FILE 이나 실행 인자로 다른 파일을 지정할 수도 있다)
TEACH_FILE = os.path.expanduser('~/ws_cobot_pjt/teach_points/bed.json')
# 재고, 기준값, 측정 기록이 저장되는 곳. 에뮬레이터로 돌릴 때는 BED_DATA_DIR 로 다른 폴더를 지정한다
DATA_DIR = os.path.expanduser(os.environ.get('BED_DATA_DIR', '~/ws_cobot_pjt/bed_data'))
SIM_TEACH_FILE = os.path.expanduser('~/ws_cobot_pjt/teach_points/bed_sim.json')   # bed_sim 이 만드는 가짜 교시 파일
SIM_DATA_DIR = DATA_DIR + '_sim'                           # 가짜 로봇으로 돌릴 때는 기록을 따로 둔다

# ── 교시 위치 이름 (direct_teach 에서 이 이름으로 저장) ───────────────
# 침대 칸 / 보관대 / 버리는 곳은 "블록을 든 채로 지나가도 안 부딪히는 높이"에서 교시한다.
# 거기서 아래로 내려가는 양은 아래 *_DZ 값으로 정한다 (전부 베이스 Z 상대 이동).
P_HOME = 'home'
P_BED = ['bed_1', 'bed_2', 'bed_3', 'bed_4', 'bed_5', 'bed_6']       # 각 블록 중심 위 (툴이 아래를 봄)
P_RACK = ['rack_1', 'rack_2', 'rack_3', 'rack_4', 'rack_5', 'rack_6']  # 보관대 자리 위 (트레이 2개 x 3자리)
P_DISCARD = 'discard'                                                # 불량 블록 버리는 통 위
# 아래 3개는 "잡는 자세 그대로" 교시하고, 각각 5cm 쯤 뒤로 뺀 접근점(_app)도 같이 교시한다.
P_ROLLER = 'roller'               # 롤러 손잡이를 잡은 자세      (+ roller_app)
P_COVER_GRIP = 'cover_grip'       # 닫힌 커버 손잡이를 잡은 자세  (+ cover_grip_app)
P_COVER_OPEN = 'cover_open'       # 다 열린 커버 손잡이를 잡은 자세 (+ cover_open_app)
P_COVER_VIA = 'cover_via'         # 커버가 반쯤 열렸을 때 손잡이를 잡은 자세 (movec 경유점)
APP_SUFFIX = '_app'

# ── 침대 배치 ─────────────────────────────────────────────────────────
# 블록 번호 (위에서 본 모습, 로봇 베이스 좌표 기준)
#   2 4 6      y = -129.35
#   1 3 5      y = -179.35
#   x 가 커지는 쪽으로 1 → 3 → 5
# 코드에서는 3행 x 2열로 다룬다: 행 = x 방향(1→3→5), 열 = y 방향(1→2).  i번 블록의 (행, 열) = divmod(i-1, 2)
BED_ROWS, BED_COLS = 3, 2
# 화면(bed_hmi)에 칸을 놓는 모양. 위 그림과 같게: 윗줄 2 4 6, 아랫줄 1 3 5
DISPLAY_GRID = [[2, 4, 6],
                [1, 3, 5]]
# 기울기 방향 이름 (화면, 로그에 쓴다). 'col' = 열 방향, 'row' = 행 방향
AXIS_NAMES = {'col': '1→2번 방향', 'row': f'1→{BED_COLS + 1}번 방향'}

# 칸 중심 좌표 (베이스 x, y) [mm]. bed_sim(가짜 로봇)이 이 좌표로 가짜 교시 파일을 만든다.
# 실제 로봇은 교시 파일(bed.json)의 좌표를 쓰므로, 이 값은 교시할 때 참고용이다.
BED_XY = [(498.78, -179.35), (498.78, -129.35),     # 1번, 2번
          (576.28, -179.35), (576.28, -129.35),     # 3번, 4번
          (653.78, -179.35), (653.78, -129.35)]     # 5번, 6번
# 보관대: 1번 트레이 = rack_1~3 (y = 119.15), 2번 트레이 = rack_4~6 (y = 194.15), x 가 작은 쪽부터
RACK_XY = [(446.28, 119.15), (501.28, 119.15), (556.28, 119.15),
           (446.28, 194.15), (501.28, 194.15), (556.28, 194.15)]

# ── 블록 종류와 처음 재고 ─────────────────────────────────────────────
BLOCK_HEIGHTS = [25.0, 30.0, 35.0]          # 기울기 맞춤용 블록 높이 3가지
H_REF = 30.0                                # *_PICK_DZ 를 맞춘 기준 높이 (= 보통 큐브 높이)
# 자동 보수의 '높게' 큐브(키 큰 10A) 높이. ※ 실물 큐브를 재서 바꿀 것 (지금은 가정값)
TALL_HEIGHT = 40.0
INVENTORY_HEIGHTS = sorted(set(BLOCK_HEIGHTS + [TALL_HEIGHT]))   # 재고에 적을 수 있는 높이
# 처음 실행할 때만 쓰는 값. 이후에는 DATA_DIR/inventory.json 에 기록된 것을 쓴다.
BED_INIT = [30.0, 30.0, 30.0, 30.0, 30.0, 30.0]
# None = 빈 자리 (최소 1개 필요: 자동 보수에서 빼낸 큐브를 여기에 되돌려 놓는다).
#   자동 보수 시나리오: 같은 높이 큐브 5개(10A/40A 섞어서) + 빈 자리 1개
#   (기울기 맞춤을 시연하려면 재고 수정에서 25/35mm 블록으로 바꾼다)
RACK_INIT = [30.0, 30.0, 30.0, 30.0, 30.0, None]

# ── 이동 속도 ─────────────────────────────────────────────────────────
VELJ, ACCJ = 30, 30               # movej 일반 [deg/s, deg/s^2]
VELJ_SLOW, ACCJ_SLOW = 10, 20     # 접근점 ↔ 잡는 자세
VELX, ACCX = 60, 120              # 수직 이동 movel [mm/s, mm/s^2]
VELX_SLOW, ACCX_SLOW = 20, 60     # 블록을 잡으러 내려갈 때 / 놓을 때

# ── 그리퍼 (OnRobot RG2) ──────────────────────────────────────────────
# 'service': /onrobot/sendCommand 서비스 (m0609_rg2_bringup 이 띄우는 드라이버). 폭 지정 가능
# 'io'     : 컨트롤러 디지털 출력 (DO_CLOSE / DO_OPEN). 열기 / 닫기 / 너비 잡기(GRIP_DO_BOTH, 너비는 WebLogic 에서 정함)
GRIPPER_MODE = os.environ.get('BED_GRIPPER_MODE', 'service')   # 환경변수 BED_GRIPPER_MODE=io 로도 바꿀 수 있다
GRIP_WIDTH = {                    # 손가락 사이 폭 [mm] ('service' 모드에서만 의미 있음)
    'press': 0,                   # 누르기 측정: 완전히 닫고 손가락 끝으로 누른다
    'roller_open': 60, 'roller_grip': 20,
    'cover_open': 60, 'cover_grip': 10,
    'block_open': 70, 'block_grip': 35,     # 블록 폭보다 넓게 벌리고, 조금 좁게 잡는다
}
GRIP_WAIT = 1.2                   # 그리퍼 명령 후 대기 [s]
DO_CLOSE, DO_OPEN = 1, 2          # 'io' 모드: DO1=ON 닫힘, DO2=ON 열림
# 'io' 모드에서 이 동작은 DO1·DO2 를 둘 다 켠다 → WebLogic 의 IN1=1, IN2=1 프로그램 (블록 68 x 43 mm → 66 mm)
# 힘이 아니라 너비로 멈추니 무른 블록도 1.5 mm 만 눌려 휘지 않는다. 빈 튜플이면 예전처럼 완전히 닫는다
GRIP_DO_BOTH = ('block_grip',)
DO_BOTH_WIDTH = 66.0              # 위 WebLogic 프로그램의 너비 (시뮬레이터만 쓴다). 블록 68 mm (칸 70) → 2 mm 눌러 66 [mm]

# ── 누르기 측정 ───────────────────────────────────────────────────────
PRESS_APPROACH_DZ = -40.0         # 교시 높이에서 빠르게 내려오는 양. 끝난 높이가 가장 높은 블록(TALL_HEIGHT)보다 5mm 이상 위여야 함
SEARCH_STEP = 1.0                 # 접촉 전 한 스텝 [mm]
PRESS_STEP = 0.5                  # 접촉 후 한 스텝 [mm]
STEP_VEL, STEP_ACC = 10, 40       # 스텝 이동 속도
PRESS_SETTLE = 0.3                # 접근 후 영점 잡기 전 대기 [s]
STEP_SETTLE = 0.1                 # 스텝 후 대기 [s]
TARE_SAMPLES = 15                 # 영점 읽기 횟수
READ_SAMPLES = 3                  # 스텝마다 읽기 횟수
F_CONTACT = 3.0                   # 접촉 판단 [N]
F_PRETOUCH = 6.0                  # 누르기 전 '이미 닿음' 검사 [N]. 위·아래 두 높이의 힘 추정 차이라 접촉 판단보다 넉넉히
F_FIT_MIN = 5.0                   # 강성 계산에 쓰는 최소 힘 [N] (천이 늘어나는 초반 구간 제외)
F_TARGET = 20.0                   # 여기까지 누른다 [N]
F_LIMIT = 40.0                    # 넘으면 즉시 중단 [N]
SEARCH_MAX = 30.0                 # 접촉 없이 내려갈 수 있는 최대 거리 [mm]
PROBE_VEL = 0.0                   # >0 이면 찾기 전에 이 속도[mm/s]로 멈추지 않고 내려가며 접촉을 본다 (0 = 끔, 1 mm 계단만)
PROBE_ACC = 20.0                  # 그때 가속도
PROBE_BACK = 2.0                  # 닿아서 멈추면 이만큼 올라간 뒤 계단식(SEARCH_STEP → PRESS_STEP)으로 다시 잰다 [mm]
D_MAX = 25.0                      # 접촉 후 최대 압입 깊이 [mm]
MIN_FIT_POINTS = 4                # 강성 계산에 필요한 최소 점 수
MIN_R2 = 0.90                     # 직선 맞춤 품질이 이보다 낮으면 "측정 불량"으로 표시

# 에뮬레이터로 "동작만" 볼 때: BED_QUICK=1 이면 누르기를 크게, 기다림 없이 한다 (블록당 약 5초).
# 강성을 제대로 잴 수 없으므로 실제 로봇 측정에는 쓰지 않는다.
QUICK = os.environ.get('BED_QUICK') == '1'
if QUICK:
    SEARCH_STEP, PRESS_STEP = 5.0, 2.0
    STEP_VEL, STEP_ACC = 100, 400
    PRESS_SETTLE, STEP_SETTLE = 0.0, 0.0
    TARE_SAMPLES, READ_SAMPLES = 3, 1
    VELJ, ACCJ = 60, 120              # 이동도 빠르게 (동작 확인용)
    VELX, ACCX = 200, 600
    GRIP_WAIT = 0.2
    if os.environ.get('BED_EMU_FORCE'):
        # 에뮬레이터 가짜 힘(bed_emu)으로 강성까지 볼 때: 단단한 큐브는 5mm 만에 20N 이라 간격을 좁혀야 점이 모인다
        SEARCH_STEP, PRESS_STEP = 2.0, 0.5

# ── 불량 판정 ─────────────────────────────────────────────────────────
WARN_RATIO = 0.15                 # 기준 강성과 15% 이상 차이 → 주의
FAIL_RATIO = 0.30                 # 30% 이상 차이 → 불량 (교체 대상)

# ── 자동 보수 (빈칸 채우기 + 맞지 않는 모듈 교체) ─────────────────────
# 빈칸 판정: 닿지 않음, 또는 접촉 높이(3N 에 닿은 높이)가 가장 높은 칸보다 이만큼 이상 낮음.
#   강성으로 판정하지 않는 이유: 아주 무른 큐브(10A)가 '너무 깊이 들어감'이 되어도 빈칸으로 오판하지 않게.
#   실물에서 press_test 로 빈칸과 10A 의 접촉 높이를 재 보고 둘 사이 값으로 맞춘다.
EMPTY_DZ = 8.0
# 높은 큐브 판정: 표면이 보통 큐브들(중앙값)보다 이만큼 이상 높으면 '높음'. 기본은 높이 차이의 절반
TALL_DZ = (TALL_HEIGHT - H_REF) / 2
# 보관대 '살짝 닿기'(높이만 재기): 영점 잡을 때 잰 힘 노이즈(표준편차 σ)로 경계를 정한다
#   경계 = max(TOUCH_F_MIN, TOUCH_SIGMA × σ). 경계를 TOUCH_CONFIRM 번 연속 넘어야 닿은 것으로 본다 (순간 튐 무시)
TOUCH_F_MIN = 1.0                 # [N]
TOUCH_SIGMA = 5.0
TOUCH_CONFIRM = 2
# 자동(이웃 기준) 모드의 '맞지 않음' 판정: 다른 칸들의 중앙값과 강성 비율이 이 값 밖이면 (0.5 → 1/2 미만, 2배 초과)
MISMATCH_RATIO = 0.5
# 사용자 지정 모드에서 두 종류(부드러움/단단함)를 나눌 수 있는 최소 강성 비율 (가장 단단한 것 / 가장 무른 것)
CLASS_MIN_RATIO = 2.0
# 보관대(트레이)에서 잰 강성 → 침대(천 너머)에서 잴 강성으로 바꾸는 비율: k_침대 ≈ RACK_TO_BED × k_트레이.
#   천 장력과 주변 구속 때문에 같은 큐브도 값이 다르다. 실물로 한 번 돌리면 재측정 후 로그에 "제안 값"이 나온다.
#   큐브는 칸마다 목표 강성과 '비율'이 가장 가까운 것을 고르므로, 두 종류(10A/40A)가 약 4배 차이 나는 한
#   이 값이 2배 가까이 틀려도 고르는 결과는 바뀌지 않는다.
RACK_TO_BED = 1.0
RACK_PRESS_APPROACH_DZ = PRESS_APPROACH_DZ   # 보관대 큐브를 누를 때 빠르게 내려오는 양 (보관대 높이가 다르면 따로 맞춘다)
# 사용자 지정에서 칸마다 고르는 것: 단단하게(40A) / 부드럽게(10A) / 높게(키 큰 10A)
PROFILE_NAMES = {'hard': '단단하게', 'soft': '부드럽게', 'tall': '높게'}

# ── 롤러 (천 팽팽하게 / 풀기) ─────────────────────────────────────────
# False 면 천이 처음부터 팽팽하다고 보고 롤러 동작(2, 5, 10단계)을 건너뛴다. 롤러 교시 위치도 필요 없다.
ROLLER_ENABLED = os.environ.get('BED_ROLLER') == '1'     # 기본 False. 시험용으로 환경변수 BED_ROLLER=1 이면 켬
ROLLER_TURN_DEG = 90.0            # 한 번 잡고 돌리는 각도 (J6). + 방향이 팽팽하게
ROLLER_TURNS = 2                  # 다시 잡고 돌리는 횟수
ROLLER_VEL, ROLLER_ACC = 30, 30
J6_LIMIT = 350.0                  # J6 가 이 범위를 넘게 되면 돌리기 전에 멈춘다

# ── 커버 열기 / 닫기 (movec) ──────────────────────────────────────────
# False 면 커버가 없다고 보고 커버 동작(6, 9단계)을 건너뛴다. 커버 교시 위치(cover_*)도 필요 없다.
COVER_ENABLED = os.environ.get('BED_COVER') == '1'       # 기본 False. 시험용으로 환경변수 BED_COVER=1 이면 켬
COVER_VEL, COVER_ACC = 30, 60     # 천천히

# ── 블록 집기 / 놓기 ──────────────────────────────────────────────────
BED_PICK_DZ = -75.0               # bed_N 교시 높이 → 기준 높이(H_REF) 블록을 잡는 높이
RACK_PICK_DZ = -75.0              # rack_N 교시 높이 → 기준 높이 블록을 잡는 높이
PLACE_CLEAR = 2.0                 # 놓을 때는 잡은 높이보다 이만큼 위에서 놓는다
PLACE_SETTLE = 0.3                # 놓을 높이에 멈춘 뒤 흔들림이 가라앉게 기다렸다가 연다 [s]

# ── 기울기 맞춤 ───────────────────────────────────────────────────────
TILT_PLANE_TOL = 0.5              # 블록 윗면이 한 평면에서 이만큼 [mm] 이내여야 "평평한 기울기"로 인정
TILT_ALLOW_STEP = False           # True 면 계단 모양(완전한 평면이 아닌) 조합도 후보에 넣는다
TILT_OK_DEG = 1.0                 # 재측정 기울기가 목표와 이만큼 이내면 성공으로 표시

# ── 12단계 이름 (화면 표시용) ─────────────────────────────────────────
STEP_NAMES = [
    '', '홈 위치', '천 팽팽하게', '실리콘 측정', '판정 · 교체 계획', '천 느슨하게',
    '커버 열기', '블록 빼기', '블록 넣기', '커버 닫기', '천 팽팽하게', '재측정', '정상 확인',
]
for _n in ((2, 5, 10) if not ROLLER_ENABLED else ()) + ((6, 9) if not COVER_ENABLED else ()):
    STEP_NAMES[_n] += ' (생략)'
