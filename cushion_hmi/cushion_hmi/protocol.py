# HMI <-> 미션 노드 통신 규약
#
# 토픽은 모두 std_msgs/String 이고 내용은 JSON 이다.
# 커스텀 메시지(ament_cmake 패키지)를 따로 만들지 않아도 되고, 필드를 늘리기 쉽다.
# 실제 미션 노드를 만들 때도 이 파일을 import 해서 같은 이름·형식을 쓰면 된다.
#
#   /cushion/cmd     HMI -> 미션   {"cmd": "start", "mode": "full"}
#                                   {"cmd": "stop"} | {"cmd": "continue"} | {"cmd": "reset"}
#   /cushion/status  미션 -> HMI   0.5초마다 + 바뀔 때마다 발행 (HMI 는 이걸로 연결 여부 판단)
#       {
#         "state": "MEASURE", "mode": "full", "running": true,
#         "current_module": 3,                       # 지금 작업 중인 모듈 (없으면 null)
#         "modules": [{"id": 1, "status": "ok", "k": 6.1, "z0": 120.0}, ...],
#         "wait_operator": false, "prompt": "",      # 작업자 확인이 필요한 수동 단계
#         "alarm": "", "retry": 0, "spares": 2,      # 알람 문구, 재시도 횟수, 선반 재고
#         "source": "모의 미션",                     # (선택) 연결 표시에 쓸 이름
#         "control": true                            # (선택) false 면 HMI 버튼을 끈다 (표시 전용)
#       }
#     modules 항목에 선택 필드: "name"(교시점 이름), "fz", "fz_std"(정지 측정 평균·표준편차 [N])
#   /cushion/force   미션 -> HMI   샘플마다 {"module": 3, "z": 1.2, "fz": 7.4, "reset": false}
#                                   z = 접촉점부터 압입 깊이 [mm], fz = 힘 [N], reset = 새 측정 시작
#                                   z 대신 "t"(측정 시작부터 시간 [s]) 를 보내면 그래프 가로축이 시간이 된다
#   /cushion/log     미션 -> HMI   사람이 읽는 로그 한 줄 (String 그대로, JSON 아님)

import json

TOPIC_CMD = '/cushion/cmd'
TOPIC_STATUS = '/cushion/status'
TOPIC_FORCE = '/cushion/force'
TOPIC_LOG = '/cushion/log'
TOPIC_ORDER_CONFIG = '/cushion/order_config'

# HTML 선택값을 HMI에 표시할 때 사용할 강성값 [N/mm]
STIFFNESS_K = {'부드러움': 7.0, '단단함': 12.0}
HEIGHTS = ('NORMAL', 'HIGH')

# 연결 판단: 이 시간 동안 status 가 안 오면 끊김으로 본다
STATUS_TIMEOUT = 2.0

MODES = [
    ('calibrate', '기준값 측정'),
    ('diagnose', '진단만'),
    ('full', '진단 + 교체'),
]
MODE_KEYS = [k for k, _ in MODES]

# 플로우차트 순서대로 (상태 표시줄에 이 순서로 나온다)
FLOW_STATES = [
    ('INIT', '초기화'),
    ('TENSION', '천 조이기'),
    ('MEASURE', '측정'),
    ('JUDGE', '판정'),
    ('RELEASE', '천 풀기'),
    ('COVER_OPEN', '커버 오픈'),
    ('REPLACE', '모듈 교체'),
    ('COVER_CLOSE', '커버 클로즈'),
    ('REMEASURE', '재측정'),
    ('REPORT', '리포트'),
]
OTHER_STATES = [
    ('IDLE', '대기'),
    ('DONE', '완료'),
    ('FAULT', '이상'),
]
STATE_LABELS = dict(FLOW_STATES + OTHER_STATES)

MODULE_STATUS = {
    'unknown': '미측정',
    'measuring': '측정 중',
    'measured': '측정 완료',
    'ok': '정상',
    'defect': '불량',
    'replaced': '교체됨',
}

# 모듈 배치: 가로 2 x 세로 3.
# 번호는 위(선반 쪽) 왼쪽(로봇 쪽)부터 행 우선: 1 2 / 3 4 / 5 6
# poses.json 의 측정 위치 순서와 반드시 맞춘다.
ROWS = 3
COLS = 2


def module_ids():
    return list(range(1, ROWS * COLS + 1))


def normalize_order_slots(slots):
    """Validate order slots and normalize legacy stiffness-only values."""
    expected = {f'Slot {index}' for index in range(1, ROWS * COLS + 1)}
    if not isinstance(slots, dict) or set(slots) != expected:
        raise ValueError('slots에는 Slot 1부터 Slot 6까지 필요합니다')

    normalized = {}
    for name in sorted(expected, key=lambda slot: int(slot.split()[-1])):
        value = slots[name]
        if isinstance(value, str):
            stiffness = value
            height = 'NORMAL'
        elif isinstance(value, dict):
            stiffness = value.get('stiffness')
            height = value.get('height', 'NORMAL')
        else:
            raise ValueError(f'{name} 설정 형식이 올바르지 않습니다')

        if not isinstance(stiffness, str) or stiffness not in STIFFNESS_K:
            raise ValueError(f'{name} 강성 값이 올바르지 않습니다')
        if not isinstance(height, str) or height not in HEIGHTS:
            raise ValueError(f'{name} 높이 값이 올바르지 않습니다')
        if stiffness == '단단함' and height != 'NORMAL':
            raise ValueError(f'{name} 단단한 모듈은 기본 높이만 지원합니다')
        normalized[name] = {'stiffness': stiffness, 'height': height}
    return normalized


def module_rc(mid):
    """모듈 번호 -> (행, 열), 0부터."""
    return divmod(mid - 1, COLS)


def neighbors(mid):
    """상하좌우로 붙어 있는 모듈 번호."""
    r, c = module_rc(mid)
    out = []
    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        rr, cc = r + dr, c + dc
        if 0 <= rr < ROWS and 0 <= cc < COLS:
            out.append(rr * COLS + cc + 1)
    return out


def encode(d):
    return json.dumps(d, ensure_ascii=False)


def decode(s):
    """JSON 문자열 -> dict. 형식이 틀리면 None."""
    try:
        d = json.loads(s)
    except (ValueError, TypeError):
        return None
    return d if isinstance(d, dict) else None
