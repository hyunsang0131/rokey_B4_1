# 나사로 실 고정 / 고정해제: 좌표 계산 (로봇 없이 계산만, 단위 테스트 대상).
# 설정은 string_v2_config. 실제 동작은 string_demo.

import math

from bed_inspect import string_v2_config as sc


class StringPlanError(ValueError):
    pass


def hole_grip_z():
    """가운데 구멍에 꽂힌 나사를 잡는 TCP 높이 (나사 끝이 구멍 윗면에 닿은 높이)."""
    return sc.HOLE_GRIP_Z if sc.HOLE_GRIP_Z is not None else sc.TABLE_Z + 6.0


def wind_z():
    """감을 때 TCP 높이: 잡은 나사 끝이 탁자에 안 끌리는 높이."""
    if sc.SPIRAL_MODE == 'outward':
        if sc.OUTWARD_Z is None:
            raise StringPlanError('좁게→넓게(outward) 는 감는 나사 바로 위에서 시작하므로 OUTWARD_Z(STR_OUTWARD_Z) 를 '
                                  '감는 나사 머리보다 높게 줘야 합니다.')
        return sc.OUTWARD_Z
    return sc.TABLE_Z + sc.SCREW_LEN + sc.WIND_CLEAR


def pose(xy, z):
    return [float(xy[0]), float(xy[1]), float(z)] + [float(v) for v in sc.STR_ORI]


def around(center, r, deg):
    a = math.radians(deg)
    return (center[0] + r * math.cos(a), center[1] + r * math.sin(a))


def path_check(points, what):
    """감기/풀기 길이 로봇 베이스·도달 거리·실 고정 나사와 부딪히지 않는지. 문제 있으면 StringPlanError."""
    dmin = min(math.hypot(x, y) for x, y in points)
    dmax = max(math.hypot(x, y) for x, y in points)
    amin = min(math.hypot(x - sc.ANCHOR_XY[0], y - sc.ANCHOR_XY[1]) for x, y in points)
    errs = []
    if dmin < sc.BASE_CLEAR:
        errs.append(f'로봇 베이스 중심에서 {dmin:.0f}mm 까지 다가갑니다 (최소 {sc.BASE_CLEAR:.0f}mm). 감는 나사가 베이스에서 '
                    f'{math.hypot(*sc.POST_XY):.0f}mm 라 반지름은 대략 {math.hypot(*sc.POST_XY) - sc.BASE_CLEAR:.0f}mm 이하여야 '
                    f'한 바퀴를 돌 수 있습니다')
    if dmax > sc.REACH_MAX:
        errs.append(f'베이스에서 {dmax:.0f}mm 까지 멀어집니다 (최대 {sc.REACH_MAX:.0f}mm)')
    if amin < sc.ANCHOR_CLEAR:
        errs.append(f'실 고정 나사 {sc.ANCHOR_XY} 에 {amin:.0f}mm 까지 다가갑니다 (최소 {sc.ANCHOR_CLEAR:.0f}mm)')
    if errs:
        raise StringPlanError(f'{what}: ' + ' / '.join(errs))
    return dmin, dmax, amin


def spiral_xy(r0, r1, deg, start_deg, ccw, n=720):
    """반지름 r0 → r1 로 deg 만큼 도는 나선의 xy 점들 (검사용)."""
    s = 1.0 if ccw else -1.0
    out = []
    for i in range(n + 1):
        t = i / n
        a = math.radians(start_deg + s * deg * t)
        r = r0 + (r1 - r0) * t
        out.append((sc.POST_XY[0] + r * math.cos(a), sc.POST_XY[1] + r * math.sin(a)))
    return out


def unwind_radius():
    return sc.UNWIND_RADIUS if sc.UNWIND_RADIUS is not None else (sc.SPIRAL_R_OUT + sc.SPIRAL_R_IN) / 2.0


def spiral_inward(z0):
    """넓게(30) → 좁게(20) 안쪽 나선. 두산 move_spiral 안쪽형은 목표점이 중심이고 반지름이 0 까지 줄어든다.
    30→20 을 2바퀴에 하려면 0 까지는 2 x 30/(30-20) = 6바퀴, z 도 같은 비율로 더 올라가는 목표를 준다.
    그리고 2바퀴 돌았을 때 멈춘다 (SpiralWatch).
    돌려주는 값: dict(start=시작 posx, target=목표 xyz(중심), rev=전체 바퀴, stop_deg=멈출 각도, ...)"""
    if not (0 < sc.SPIRAL_R_IN < sc.SPIRAL_R_OUT):
        raise StringPlanError('반지름은 0 < 좁은 반지름 < 넓은 반지름 이어야 합니다.')
    if sc.SPIRAL_R_IN < sc.SPIRAL_MIN_GAP:
        raise StringPlanError(f'좁은 반지름 {sc.SPIRAL_R_IN} 이 안전 거리 {sc.SPIRAL_MIN_GAP} 보다 작습니다.')
    full_rev = sc.SPIRAL_TURNS * sc.SPIRAL_R_OUT / (sc.SPIRAL_R_OUT - sc.SPIRAL_R_IN)
    full_rise = sc.SPIRAL_RISE * full_rev / sc.SPIRAL_TURNS
    start_xy = around(sc.POST_XY, sc.SPIRAL_R_OUT, sc.SPIRAL_START_DIR_DEG)
    # 정지 명령 뒤 감속으로 조금 더 돌 수 있으니 20도 더 본다
    path_check(spiral_xy(sc.SPIRAL_R_OUT, sc.SPIRAL_R_IN - 20.0 * (sc.SPIRAL_R_OUT - sc.SPIRAL_R_IN) / 720.0,
                         360.0 * sc.SPIRAL_TURNS + 20.0, sc.SPIRAL_START_DIR_DEG, sc.SPIRAL_CCW), '감기(나선)')
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


def spiral_outward(z0):
    """좁게(0) → 넓게(30) 바깥 나선. 두산 바깥형은 현재 위치가 중심 → 감는 나사 바로 위에서 시작."""
    path_check(spiral_xy(0.0, sc.SPIRAL_R_OUT, 360.0 * sc.SPIRAL_TURNS, 0.0, True), '감기(바깥 나선)')
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
    """풀기: 감은 반대 방향(위에서 시계)으로 감는 나사 둘레 2바퀴. movec(시작 → 경유 → 끝, angle=720).
    감기가 구멍 쪽(-y)에서 시작/끝났으므로 풀기도 그쪽에서 시작한다."""
    r, d0 = unwind_radius(), sc.SPIRAL_START_DIR_DEG
    step = -90.0 if sc.SPIRAL_CCW else 90.0         # 감은 반대
    path_check(spiral_xy(r, r, 360.0 * sc.SPIRAL_TURNS, d0, not sc.SPIRAL_CCW), '풀기(원호)')
    return {
        'start': pose(around(sc.POST_XY, r, d0), z),
        'via': pose(around(sc.POST_XY, r, d0 + step), z),
        'end': pose(around(sc.POST_XY, r, d0 + 2 * step), z),
        'angle': 360.0 * sc.SPIRAL_TURNS,
    }


def tension_dir():
    if sc.TENSION_DIR is not None:
        d = sc.TENSION_DIR
    else:
        d = (sc.HOLE_XY[0] - sc.ANCHOR_XY[0], sc.HOLE_XY[1] - sc.ANCHOR_XY[1])
    n = math.hypot(*d)
    return (d[0] / n, d[1] / n)


def circle_through(t_xy, z, turns, ccw=True):
    """감는 나사 둘레를 t_xy 를 지나는 원으로 turns 바퀴 (movec 시작 → 경유 → 끝, angle). 베이스·실 고정 나사 검사."""
    r = math.hypot(t_xy[0] - sc.POST_XY[0], t_xy[1] - sc.POST_XY[1])
    d0 = math.degrees(math.atan2(t_xy[1] - sc.POST_XY[1], t_xy[0] - sc.POST_XY[0]))
    step = 90.0 if ccw else -90.0
    path_check(spiral_xy(r, r, 360.0 * turns, d0, ccw), f'감기(반지름 {r:.0f}mm 원)')
    return {
        'r': r,
        'start': pose(around(sc.POST_XY, r, d0), z),
        'via': pose(around(sc.POST_XY, r, d0 + step), z),
        'end': pose(around(sc.POST_XY, r, d0 + 2 * step), z),
        'angle': 360.0 * turns,
    }


def max_circle_r():
    """감는 나사 둘레를 한 바퀴 돌 수 있는 최대 반지름 (베이스 BASE_CLEAR 기준)."""
    return math.hypot(*sc.POST_XY) - sc.BASE_CLEAR


def fix_points(pick_xyz):
    """v2 고정 동작의 주요 점 (move_spiral 없음: 높이 올려 장력 찾고 크게 돌기)."""
    if pick_xyz is None:
        raise StringPlanError('실 끝 나사를 잡을 위치가 없습니다 (--pick x y z 또는 STR_PICK_XYZ).')
    hz = hole_grip_z()
    out_z = max(pick_xyz[2] + sc.CARRY_DZ, sc.TABLE_Z + sc.SCREW_LEN + sc.WIND_CLEAR)
    d = tension_dir()
    t_end = (pick_xyz[0] + d[0] * sc.TENSION_MAX, pick_xyz[1] + d[1] * sc.TENSION_MAX)
    return {
        'pick_app': pose(pick_xyz[:2], pick_xyz[2] + sc.APPROACH_DZ),
        'pick': pose(pick_xyz[:2], pick_xyz[2]),
        'pick_up': pose(pick_xyz[:2], out_z),
        'wind_up': pose(pick_xyz[:2], sc.WIND2_Z),
        'tension_end': pose(t_end, sc.WIND2_Z),
        'hole_over': pose(sc.HOLE_XY, sc.WIND2_Z),
        # 나사 끝이 구멍 윗면 INSERT_GAP 위에 오는 높이 (그 뒤는 순응 제어 + 누르기로 내려감)
        'hole_touch': pose(sc.HOLE_XY, sc.TABLE_Z + sc.SCREW_LEN + sc.INSERT_GAP),
        'hole_app': pose(sc.HOLE_XY, hz + sc.APPROACH_DZ),
    }


def unfix_points(place_xyz):
    """고정해제 동작의 주요 점."""
    if place_xyz is None:
        raise StringPlanError('풀고 난 나사를 놓을 위치가 없습니다 (--place x y z 또는 STR_PLACE_XYZ / STR_PICK_XYZ).')
    hz = hole_grip_z()
    pull_z = max(hz + sc.PULL_DZ, wind_z() + 10.0)
    arc = unwind_arc(sc.WIND2_Z)
    return {
        'hole_app': pose(sc.HOLE_XY, hz + sc.APPROACH_DZ),
        'hole_grip': pose(sc.HOLE_XY, hz),
        'hole_pull': pose(sc.HOLE_XY, pull_z),
        'unwind': arc,
        'place_over': pose(place_xyz[:2], max(pull_z, place_xyz[2] + sc.CARRY_DZ)),
        'place': pose(place_xyz[:2], place_xyz[2]),
        'place_app': pose(place_xyz[:2], place_xyz[2] + sc.APPROACH_DZ),
    }


class SpiralWatch:
    """안쪽 나선을 돌리는 동안 현재 xy 로 감는 나사 둘레 누적 각도를 잰다 (위에서 반시계 +).
    update() 가 'stop'(목표 각도 도달), 'too_close'(너무 가까움), 'wrong_dir'(반대로 돎), None 을 돌려준다."""

    def __init__(self, center, stop_deg, ccw=True, lead=None, min_gap=None, wrong_deg=None):
        self.c = center
        self.stop_deg = stop_deg
        self.sign = 1.0 if ccw else -1.0
        self.lead = sc.SPIRAL_STOP_LEAD_DEG if lead is None else lead
        self.min_gap = sc.SPIRAL_MIN_GAP if min_gap is None else min_gap
        self.wrong = sc.SPIRAL_WRONG_DIR_DEG if wrong_deg is None else wrong_deg
        self.prev = None
        self.swept = 0.0          # 원하는 방향으로 돈 각도 [deg]
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


def j6_start(cur_j6, turn_deg, limit=None):
    """돌리기 전 J6 시작 각도: cur 에서 turn_deg 만큼 돌아도 ±limit 안에 들도록 (필요한 만큼만 미리 돌림)."""
    lim = sc.J6_LIMIT if limit is None else limit
    if abs(turn_deg) > 2 * lim:
        raise StringPlanError(f'한 번에 {turn_deg}도는 J6 범위(±{lim})를 넘습니다.')
    lo = -lim - min(0.0, turn_deg)
    hi = lim - max(0.0, turn_deg)
    return min(max(cur_j6, lo), hi)


def rot_chunks(total_deg, chunk=None):
    """큰 회전을 chunk 이하 조각으로 (한 번에 180 넘게 주면 짧은 쪽으로 돌 수 있음)."""
    c = sc.ROT_CHUNK_DEG if chunk is None else chunk
    n = max(1, math.ceil(abs(total_deg) / c - 1e-9))
    return [total_deg / n] * n
