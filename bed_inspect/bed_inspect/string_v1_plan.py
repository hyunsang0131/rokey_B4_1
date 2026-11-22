# [v1 = 영상 보기 전 버전] 나사로 실 고정 / 고정해제: 좌표 계산. 설정은 string_v1_config.

import math

from bed_inspect import string_v1_config as sc


class StringPlanError(ValueError):
    pass


def hole_grip_z():
    return sc.HOLE_GRIP_Z if sc.HOLE_GRIP_Z is not None else sc.TABLE_Z + 6.0


def wind_z():
    if sc.SPIRAL_MODE == 'outward':
        if sc.OUTWARD_Z is None:
            raise StringPlanError('좁게→넓게(outward) 는 감는 나사 바로 위에서 시작하므로 STR1_OUTWARD_Z 를 '
                                  '감는 나사 머리보다 높게 줘야 합니다.')
        return sc.OUTWARD_Z
    return sc.TABLE_Z + sc.SCREW_LEN + sc.WIND_CLEAR


def pose(xy, z):
    return [float(xy[0]), float(xy[1]), float(z)] + [float(v) for v in sc.STR_ORI]


def around(center, r, deg):
    a = math.radians(deg)
    return (center[0] + r * math.cos(a), center[1] + r * math.sin(a))


def spiral_inward(z0):
    """넓게(30) → 좁게(20) 안쪽 나선. 목표점(감는 나사)이 중심, 0 까지 6바퀴 나선을 2바퀴에서 멈춘다."""
    if not (0 < sc.SPIRAL_R_IN < sc.SPIRAL_R_OUT):
        raise StringPlanError('반지름은 0 < 좁은 반지름 < 넓은 반지름 이어야 합니다.')
    if sc.SPIRAL_R_IN < sc.SPIRAL_MIN_GAP:
        raise StringPlanError(f'좁은 반지름 {sc.SPIRAL_R_IN} 이 안전 거리 {sc.SPIRAL_MIN_GAP} 보다 작습니다.')
    full_rev = sc.SPIRAL_TURNS * sc.SPIRAL_R_OUT / (sc.SPIRAL_R_OUT - sc.SPIRAL_R_IN)
    full_rise = sc.SPIRAL_RISE * full_rev / sc.SPIRAL_TURNS
    start_xy = around(sc.POST_XY, sc.SPIRAL_R_OUT, sc.SPIRAL_START_DIR_DEG)
    return {
        'mode': 'inward',
        'start': pose(start_xy, z0),
        'target': [sc.POST_XY[0], sc.POST_XY[1], z0 + full_rise],
        'rev': full_rev,
        'rise_full': full_rise,
        'stop_deg': 360.0 * sc.SPIRAL_TURNS,
        'end_r': sc.SPIRAL_R_IN,
        'end_z': z0 + sc.SPIRAL_RISE,
    }


def spiral_points(sp, step_deg=15.0):
    """안쪽 나선(감기 구간: 반지름 R_OUT → R_IN, z +RISE)을 점으로 (RViz 가상 모드용: 에뮬레이터는 목표점 중심 move_spiral 에서 멈춤)."""
    sign = 1.0 if sc.SPIRAL_CCW else -1.0
    total = sp['stop_deg']
    n = max(2, int(math.ceil(total / step_deg)))
    z0 = sp['start'][2]
    pts = []
    for i in range(1, n + 1):
        t = i / n
        r = sc.SPIRAL_R_OUT + (sp['end_r'] - sc.SPIRAL_R_OUT) * t
        xy = around(sc.POST_XY, r, sc.SPIRAL_START_DIR_DEG + sign * total * t)
        pts.append(pose(xy, z0 + (sp['end_z'] - z0) * t))
    return pts


def spiral_outward(z0):
    return {
        'mode': 'outward',
        'start': pose(sc.POST_XY, z0),
        'rev': sc.SPIRAL_TURNS,
        'rmax': sc.SPIRAL_R_OUT,
        'lmax': sc.SPIRAL_RISE,
        'stop_deg': None,
        'end_r': sc.SPIRAL_R_OUT,
        'end_z': z0 + sc.SPIRAL_RISE,
    }


def spiral_plan():
    z0 = wind_z()
    if sc.SPIRAL_MODE == 'inward':
        return spiral_inward(z0)
    if sc.SPIRAL_MODE == 'outward':
        return spiral_outward(z0)
    raise StringPlanError(f'SPIRAL_MODE 는 inward / outward: {sc.SPIRAL_MODE!r}')


def unwind_arc(z):
    r, d0 = sc.UNWIND_RADIUS, sc.SPIRAL_START_DIR_DEG
    step = -90.0 if sc.SPIRAL_CCW else 90.0
    return {
        'start': pose(around(sc.POST_XY, r, d0), z),
        'via': pose(around(sc.POST_XY, r, d0 + step), z),
        'end': pose(around(sc.POST_XY, r, d0 + 2 * step), z),
        'angle': 360.0 * sc.SPIRAL_TURNS,
    }


def travel_z():
    """감는 나사 ↔ 구멍 사이를 지날 때 TCP 높이: 나사 끝이 블록 윗면 + 여유 위."""
    return sc.TABLE_Z + sc.BLOCK_H + sc.BLOCK_CLEAR + sc.SCREW_LEN


def hole_dir():
    """감는 나사 → 구멍 방향 (단위 xy)."""
    d = (sc.HOLE_XY[0] - sc.POST_XY[0], sc.HOLE_XY[1] - sc.POST_XY[1])
    n = math.hypot(*d)
    return (d[0] / n, d[1] / n)


def fix_points(pick_xyz):
    if pick_xyz is None:
        raise StringPlanError('실 끝 나사를 잡을 위치가 없습니다 (--pick x y z 또는 STR1_PICK_XYZ).')
    sp = spiral_plan()
    hz = hole_grip_z()
    carry_z = max(pick_xyz[2] + sc.CARRY_DZ, sp['start'][2])
    # 고정(fix)은 블록 없이 실만 고정하는 경우 (11-22 형님: 블록은 해제 때만 있다고 봄). 블록 높이(200mm)까지 올라가면
    # 감긴 실이 감는 나사 위로 끌려 올라가 빠질 수 있음 → 낮게. 블록 있는 채로 고정하면 STR1_FIX_WITH_BLOCK=1
    tz = max(travel_z(), sp['end_z']) if sc.FIX_WITH_BLOCK else sp['start'][2]   # 11-22: 감은 뒤 구멍 쪽으로 갈 때 낮게 (감기 시작 높이, 나사 끝이 판 3mm 위)
    d = hole_dir()
    beyond = (sc.HOLE_XY[0] + d[0] * sc.OVERSHOOT_MAX, sc.HOLE_XY[1] + d[1] * sc.OVERSHOOT_MAX)
    return {
        'pick_app': pose(pick_xyz[:2], pick_xyz[2] + sc.APPROACH_DZ),
        'pick': pose(pick_xyz[:2], pick_xyz[2]),
        # 블록 위 높이까지 수직으로 뽑은 뒤 수평 이동 (시뮬 11-21: 대각선으로 가면 나사 끝이 블록에 걸림)
        'pick_up': pose(pick_xyz[:2], max(carry_z, tz)),
        'spiral_over': pose(sp['start'][:2], max(carry_z, tz)),
        'spiral_start': sp['start'],
        'beyond': pose(beyond, tz),                      # 구멍보다 더 멀리 (장력 느끼면 그 전에 멈춤)
        'hole_over': pose(sc.HOLE_XY, tz),
        # 나사 끝이 구멍 윗면 INSERT_GAP 위에 오는 높이 (그 뒤는 순응 제어 + 누르기로 내려감)
        'hole_touch': pose(sc.HOLE_XY, sc.TABLE_Z + sc.SCREW_LEN + sc.INSERT_GAP),
        'hole_app': pose(sc.HOLE_XY, max(hz + sc.APPROACH_DZ, tz)),
        'spiral': sp,
    }


def unwind_halves(z, r0=None):
    """11-22 형님: 해제는 감는 나사 좌표 기준 moveC 로, 실이 팽팽하게 유지되는 길로 푼다.
    한 바퀴 풀 때마다 실이 감는 나사 둘레만큼(2πr) 풀려 나오므로, 반 바퀴마다 반지름을 π·r 씩 키우며
    감은 반대 방향으로 반원 moveC 를 이어 간다 (2 x 감은 바퀴 수 개). 돌려주는 값: [(경유, 끝), ...], 시작점."""
    sign = -1.0 if sc.SPIRAL_CCW else 1.0               # 감은 반대
    d0 = sc.SPIRAL_START_DIR_DEG
    r0 = sc.SPIRAL_R_OUT if r0 is None else r0
    grow = math.pi * sc.POST_RADIUS                     # 반 바퀴에 풀려 나오는 실 길이
    n = int(round(2 * sc.SPIRAL_TURNS))
    start = pose(around(sc.POST_XY, r0, d0), z)
    halves, pts = [], []
    for k in range(n):
        a0 = d0 + sign * 180.0 * k
        r_a, r_b = r0 + grow * k, r0 + grow * (k + 1)
        via = pose(around(sc.POST_XY, (r_a + r_b) / 2, a0 + sign * 90.0), z)
        end = pose(around(sc.POST_XY, r_b, a0 + sign * 180.0), z)
        halves.append((via, end))
        pts += [via[:2], end[:2]]
    dmin = min(math.hypot(x, y) for x, y in pts)
    if dmin < sc.BASE_CLEAR_V1:
        raise StringPlanError(f'풀기 원호가 로봇 베이스 중심에서 {dmin:.0f}mm 까지 다가갑니다 (최소 {sc.BASE_CLEAR_V1:.0f}). '
                              'POST_RADIUS 나 감은 바퀴 수를 확인하세요.')
    return start, halves


def unwind_spiral_points(center_xyz, step_deg=15.0):
    """해제 move_spiral 과 같은 길 (가상/대체용): center 중심, 위에서 시계(감은 반대), 반지름 0→RMAX, z +RISE."""
    sign = -1.0 if sc.SPIRAL_CCW else 1.0
    total = 360.0 * sc.SPIRAL_TURNS
    n = max(2, int(math.ceil(total / step_deg)))
    pts = []
    for i in range(1, n + 1):
        t = i / n
        xy = around(center_xyz[:2], sc.UNWIND_SPIRAL_RMAX * t, sc.SPIRAL_START_DIR_DEG + sign * total * t)
        pts.append(pose(xy, center_xyz[2] + sc.UNWIND_SPIRAL_RISE * t))
    return pts


def unwind_turns(center_xy=None):
    """기본 move_spiral 전체 바퀴 수. 중심이 감는 나사에서 d 떨어져 있으면 반지름이 d 를 넘기 전 구간은
    감는 나사를 둘러싸지 않아 실이 안 풀림 → 감는 나사 둘레로 도는 바퀴 = 전체 x (1 - d/rmax).
    이게 감은 바퀴 수(SPIRAL_TURNS)와 같아지도록 전체 바퀴 수를 정한다 (더 돌아 반대로 감기지 않게)."""
    if center_xy is None:
        center_xy = spiral_plan()['start'][:2]
    d = math.hypot(center_xy[0] - sc.POST_XY[0], center_xy[1] - sc.POST_XY[1])
    rmax = unwind_rmax(center_xy)
    if rmax <= d + 1.0:
        raise StringPlanError(f'풀기 나선 최대 반지름 {rmax:.0f}mm 가 감는 나사까지 거리 {d:.0f}mm 보다 작아 감는 나사를 못 둘러쌉니다.')
    return sc.SPIRAL_TURNS / (1.0 - d / rmax)


def unwind_rmax(center_xy):
    """해제 move_spiral 최대 반지름 = 시작 위치 ~ 형님 교시 점 거리."""
    return math.hypot(sc.UNWIND_START_XY[0] - center_xy[0], sc.UNWIND_START_XY[1] - center_xy[1])


def unwind_outward_points(center_xyz, step_deg=10.0):
    """두산 기본 move_spiral 과 같은 길 (가상/대체/검사용): 현재 위치 중심, 반지름 0 → unwind_rmax, 감은 반대 방향, z +RISE."""
    rmax = unwind_rmax(center_xyz)
    sign = -1.0 if sc.SPIRAL_CCW else 1.0
    total = 360.0 * unwind_turns(center_xyz[:2])
    n = int(total / step_deg)
    pts = []
    for i in range(1, n + 1):
        t = i / n
        pts.append(pose(around(center_xyz[:2], rmax * t, sc.SPIRAL_START_DIR_DEG + sign * total * t),
                        center_xyz[2] + sc.UNWIND_SPIRAL_RISE * t))
    for x, y, *_ in pts:
        if math.hypot(x, y) < sc.BASE_CLEAR_V1:
            raise StringPlanError(f'풀기 나선이 로봇 베이스에 너무 가깝습니다 ({math.hypot(x, y):.0f}mm).')
    return pts


def unwind_grow(start_dir_deg, z0):
    """해제 풀기 길 (04:30 형님): 감는 나사 중심, 반지름 UNWIND_R0 → UNWIND_R1 로 커지며 z 는 UNWIND_SPIRAL_RISE 올라감,
    감은 반대 방향 SPIRAL_TURNS 바퀴. 반원 moveC 2 x 바퀴 개. 돌려주는 값: (시작 pose, [(경유, 끝), ...])"""
    sign = -1.0 if sc.SPIRAL_CCW else 1.0
    n = int(round(2 * sc.UNWIND_TURNS))
    turns = sc.UNWIND_TURNS

    def rz(t):
        # 반지름은 고르게 커지고, z 는 고르게 오르면서 두 번째 바퀴부터 바퀴마다 UNWIND_STEP_Z 씩 더 (각 반원 안에서 부드럽게)
        turn = t * turns
        extra = sc.UNWIND_STEP_Z * max(0.0, turn - 1.0)
        return sc.UNWIND_R0 + (sc.UNWIND_R1 - sc.UNWIND_R0) * t, z0 + sc.UNWIND_SPIRAL_RISE * t + extra
    start = pose(around(sc.POST_XY, sc.UNWIND_R0, start_dir_deg), z0)
    halves = []
    for k in range(n):
        a = start_dir_deg + sign * 180.0 * k
        rv, zv = rz((k + 0.5) / n)
        re_, ze = rz((k + 1.0) / n)
        halves.append((pose(around(sc.POST_XY, rv, a + sign * 90.0), zv), pose(around(sc.POST_XY, re_, a + sign * 180.0), ze)))
    for via, end in halves:
        for q in (via, end):
            if math.hypot(q[0], q[1]) < sc.BASE_CLEAR_V1:
                raise StringPlanError(f'풀기 원호가 로봇 베이스에 너무 가깝습니다 ({math.hypot(q[0], q[1]):.0f}mm).')
    return start, halves


def unwind_spiral_plan(start_xyz):
    """해제 move_spiral (11-22 04:12 형님): 감는 나사 기준, 교시로 연습한 시작점(UNWIND_START_XY, 반지름 약 92mm)에서
    감은 반대 방향으로 SPIRAL_TURNS 바퀴. 두산 안쪽형(목표점=중심)이라 반지름이 조금씩 줄어듦 (UNWIND_SHRINK 만큼),
    z 는 UNWIND_SPIRAL_RISE 올라감. 감을 때와 같은 방식(비동기 + 각도 보고 멈춤)."""
    r0 = math.hypot(start_xyz[0] - sc.POST_XY[0], start_xyz[1] - sc.POST_XY[1])
    r1 = max(sc.SPIRAL_MIN_GAP + 5.0, r0 - sc.UNWIND_SHRINK)
    turns = sc.SPIRAL_TURNS
    full_rev = turns * r0 / (r0 - r1)
    full_rise = sc.UNWIND_SPIRAL_RISE * full_rev / turns
    ccw = not sc.SPIRAL_CCW
    a0 = math.degrees(math.atan2(start_xyz[1] - sc.POST_XY[1], start_xyz[0] - sc.POST_XY[0]))
    sign = 1.0 if ccw else -1.0
    pts = []
    n = int(360 * turns / 10)
    for i in range(1, n + 1):
        t = i / n
        pts.append(pose(around(sc.POST_XY, r0 + (r1 - r0) * t, a0 + sign * 360.0 * turns * t),
                        start_xyz[2] + sc.UNWIND_SPIRAL_RISE * t))
    for x, y, *_ in pts:
        if math.hypot(x, y) < sc.BASE_CLEAR_V1:
            raise StringPlanError(f'풀기 나선이 로봇 베이스에 너무 가깝습니다 ({math.hypot(x, y):.0f}mm).')
    return {
        'mode': 'inward', 'ccw': ccw,
        'start': list(start_xyz[:3]) + list(sc.STR_ORI),
        'target': [sc.POST_XY[0], sc.POST_XY[1], start_xyz[2] + full_rise],
        'rev': full_rev, 'stop_deg': 360.0 * turns,
        'r0': r0, 'end_r': r1, 'end_z': start_xyz[2] + sc.UNWIND_SPIRAL_RISE,
        'points': pts,
    }


def unfix_points(place_xyz):
    """고정해제 (11-22 형님): 사이에 실리콘 블록이 있음.
    구멍에서 풀어 뽑기 → 블록 위 높이로 감기 시작점까지 → 감는 나사 기준 moveC 로 반지름을 키우며 반대로 풀기 (실 팽팽)
    → 위에서 봐서 바깥쪽으로 크게 돌며 살짝 팽팽하게 구멍 너머로 (블록 자리에서 실을 치움 → 블록 위치추정)
    → 구멍에 다시 꽂아 두기."""
    if place_xyz is None:
        raise StringPlanError('풀고 난 나사를 놓을 위치가 없습니다 (--place x y z 또는 STR1_PLACE_XYZ / STR1_PICK_XYZ).')
    hz = hole_grip_z()
    tz = travel_z()                                     # 나사 끝이 블록 윗면 + 여유 위
    start, halves = unwind_halves(tz)
    d = hole_dir()
    beyond = (sc.HOLE_XY[0] + d[0] * sc.OVERSHOOT_MAX, sc.HOLE_XY[1] + d[1] * sc.OVERSHOOT_MAX)
    mid = ((sc.POST_XY[0] + sc.HOLE_XY[0]) / 2, (sc.POST_XY[1] + sc.HOLE_XY[1]) / 2)
    out = (mid[0] + sc.WIDE_ARC_OUT, mid[1])            # 위에서 봐서 바깥쪽 (로봇 반대편, +x)
    return {
        'hole_app': pose(sc.HOLE_XY, max(hz + sc.APPROACH_DZ, tz)),
        'hole_grip': pose(sc.HOLE_XY, hz),
        'hole_pull': pose(sc.HOLE_XY, tz),
        # 풀기 move_spiral 중심 = 감는 나사 바로 위 (시뮬 11-22: 감기 시작 위치(감는 나사에서 30mm)를 중심으로
        # 반지름 15mm 로 돌면 원이 감는 나사를 한 번도 둘러싸지 않아 하나도 안 풀림 → 감는 나사 축 둘레로 돌아야 풀림)
        'unwind_start': pose(spiral_plan()['start'][:2], tz),   # 감기 시작 위치 (04:18 형님)
        'unwind_halves': halves,
        'wide_via': pose(out, tz),
        'beyond': pose(beyond, tz),
        'place_over': pose(place_xyz[:2], max(tz, place_xyz[2] + sc.CARRY_DZ)),
        'place': pose(place_xyz[:2], place_xyz[2]),
        'place_touch': pose(place_xyz[:2], sc.TABLE_Z + sc.SCREW_LEN + sc.INSERT_GAP),
        'place_app': pose(place_xyz[:2], place_xyz[2] + sc.APPROACH_DZ),
    }


class SpiralWatch:
    """안쪽 나선을 돌리는 동안 감는 나사 둘레 누적 각도 (위에서 반시계 +). 'stop' / 'too_close' / 'wrong_dir' / None."""

    def __init__(self, center, stop_deg, ccw=True, lead=None, min_gap=None, wrong_deg=None):
        self.c = center
        self.stop_deg = stop_deg
        self.sign = 1.0 if ccw else -1.0
        self.lead = sc.SPIRAL_STOP_LEAD_DEG if lead is None else lead
        self.min_gap = sc.SPIRAL_MIN_GAP if min_gap is None else min_gap
        self.wrong = sc.SPIRAL_WRONG_DIR_DEG if wrong_deg is None else wrong_deg
        self.prev = None
        self.swept = 0.0
        self.r = None

    def update(self, xy):
        dx, dy = xy[0] - self.c[0], xy[1] - self.c[1]
        self.r = math.hypot(dx, dy)
        if self.r < self.min_gap:
            return 'too_close'
        a = math.degrees(math.atan2(dy, dx))
        if self.prev is not None:
            d = (a - self.prev + 180.0) % 360.0 - 180.0
            self.swept += self.sign * d
        self.prev = a
        if self.swept < -self.wrong:
            return 'wrong_dir'
        if self.swept >= self.stop_deg - self.lead:
            return 'stop'
        return None


def rot_chunks(total_deg, chunk=None):
    c = sc.ROT_CHUNK_DEG if chunk is None else chunk
    n = max(1, math.ceil(abs(total_deg) / c - 1e-9))
    return [total_deg / n] * n


def j6_start(cur_j6, turn_deg, limit=None):
    """돌리기 전 J6 시작 각도: cur 에서 turn_deg 만큼 돌아도 ±limit 안에 들도록."""
    lim = sc.J6_LIMIT if limit is None else limit
    if abs(turn_deg) > 2 * lim:
        raise StringPlanError(f'한 번에 {turn_deg}도는 J6 범위(±{lim})를 넘습니다.')
    lo = -lim - min(0.0, turn_deg)
    hi = lim - max(0.0, turn_deg)
    return min(max(cur_j6, lo), hi)
