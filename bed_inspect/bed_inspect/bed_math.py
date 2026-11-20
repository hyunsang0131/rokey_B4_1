# 계산만 하는 함수 모음 (로봇, ROS 와 무관 → 로봇 없이 테스트 가능)
#
#   fit_stiffness   누르기 곡선(높이, 힘) → 강성 k, 표면 높이
#   judge           블록별 강성 → 정상 / 주의 / 불량
#   measured_tilt   6개 표면 높이 → 침대 기울기
#   plan_moves      현재 배치 → 목표 배치로 가는 블록 이동 순서
#   plan_replace    불량 블록 → 같은 높이 예비 블록으로 바꾸는 이동 순서
#   cover_path      데모 원호 3점 + 고리 잡는 점 → 커버 열기 movec 좌표 (경첩 축 기준 회전)
#   find_empties / judge_auto / judge_profile / plan_fill / plan_repair_moves   자동 보수
#
# 칸 번호는 0부터 센다(코드 안). 화면에 보여줄 때만 +1 한다.

import itertools
import math
import statistics
from collections import Counter

import numpy as np


class PlanError(Exception):
    """교체 계획을 세울 수 없음 (재고 부족, 보관대 빈 자리 없음 등)."""


# ── 강성 ──────────────────────────────────────────────────────────────
def fit_stiffness(curve, f_min, min_points=4):
    """누르기 곡선 [(z, 힘), ...] 을 직선 F = k * (z_surf - z) 로 맞춘다.

    f_min 이상인 점만 쓴다(천이 늘어나는 초반 구간 제외).
    반환: {'k': N/mm, 'z_surf': 힘이 0 이 되는 높이(=표면 높이), 'r2': 맞춤 품질, 'n': 점 수}
          점이 모자라거나 직선이 아니면 None
    """
    pts = [(z, f) for z, f in curve if f >= f_min]
    if len(pts) < min_points:
        return None
    z = np.array([p[0] for p in pts], dtype=float)
    f = np.array([p[1] for p in pts], dtype=float)
    if z.max() - z.min() < 1e-6:
        return None
    slope, intercept = np.polyfit(z, f, 1)      # f = slope * z + intercept
    k = -slope                                   # 내려갈수록(z 감소) 힘이 커진다
    if k <= 1e-6:
        return None
    pred = slope * z + intercept
    ss_res = float(((f - pred) ** 2).sum())
    ss_tot = float(((f - f.mean()) ** 2).sum())
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    return {'k': float(k), 'z_surf': float(intercept / k), 'r2': float(r2), 'n': len(pts)}


# ── 불량 판정 ─────────────────────────────────────────────────────────
def judge(blocks, calib, warn_ratio, fail_ratio):
    """블록별 판정을 blocks 의 각 dict 에 채운다 (ref, ref_src, dev, grade).

    blocks: [{'height': mm, 'k': N/mm 또는 None, 'error': None|'no_contact'|'too_deep'|...}, ...]
    calib : {높이: 기준 강성} (기준값 저장을 한 경우). 없는 높이는 아래 방법으로 대신한다.

    기준 강성 정하는 법
      1) calib 에 그 높이가 있으면 그 값
      2) 없으면 이번에 잰 블록들의 (k × 높이) 중앙값 ÷ 높이
         (같은 재질, 같은 단면이면 k × 높이 가 일정하다는 성질을 이용.
          높이가 전부 같으면 그냥 "k 의 중앙값"과 같다)
    grade: 'ok' 정상 / 'warn' 주의 / 'fail' 불량 / 'nodata' 측정 실패(다시 측정 필요)
    """
    kh = [b['k'] * b['height'] for b in blocks if b.get('k') is not None and b.get('height')]
    kh_med = statistics.median(kh) if kh else None
    for b in blocks:
        b['ref'] = b['ref_src'] = b['dev'] = None
        if b.get('error') in ('no_contact', 'too_deep'):
            b['grade'] = 'fail'          # 블록이 없거나 너무 물러서 바닥까지 들어감
            continue
        if b.get('k') is None or not b.get('height'):     # 높이 기록이 없으면(빈칸으로 기록됨) 기준을 정할 수 없다
            b['grade'] = 'nodata'
            continue
        ref = _calib_get(calib, b['height'])
        src = 'calib'
        if ref is None and kh_med is not None:
            ref, src = kh_med / b['height'], 'median'
        if ref is None or ref <= 0:
            b['grade'] = 'nodata'
            continue
        dev = (b['k'] - ref) / ref
        b['ref'], b['ref_src'], b['dev'] = float(ref), src, float(dev)
        b['grade'] = 'fail' if abs(dev) >= fail_ratio else 'warn' if abs(dev) >= warn_ratio else 'ok'
    return blocks


def _calib_get(calib, height):
    if not calib:
        return None
    for h, k in calib.items():
        if abs(float(h) - float(height)) < 1e-6:
            return float(k)
    return None


def make_calib(blocks, old=None):
    """측정 결과에서 높이별 기준 강성(중앙값)을 만든다. old 에 있던 다른 높이는 유지."""
    out = {str(float(h)): float(k) for h, k in (old or {}).items()}
    by_h = {}
    for b in blocks:
        if b.get('k') is not None:
            by_h.setdefault(float(b['height']), []).append(b['k'])
    for h, ks in by_h.items():
        out[str(h)] = float(statistics.median(ks))
    return out


# ── 기울기 ────────────────────────────────────────────────────────────
def fit_plane(xy, z):
    """점들을 평면 z = a*x + b*y + c 로 맞춘다. 반환 (a, b, c, rms 오차)."""
    A = np.array([[x, y, 1.0] for x, y in xy], dtype=float)
    zz = np.array(z, dtype=float)
    coef, *_ = np.linalg.lstsq(A, zz, rcond=None)
    res = zz - A @ coef
    return float(coef[0]), float(coef[1]), float(coef[2]), float(math.sqrt((res ** 2).mean()))


def bed_axes(xy, rows, cols):
    """칸 중심 좌표에서 침대의 열 방향(1→2번), 행 방향(1→(열 수+1)번) 단위벡터와 칸 간격을 구한다."""
    p = np.array(xy, dtype=float).reshape(rows, cols, 2)
    dc = [p[r, c + 1] - p[r, c] for r in range(rows) for c in range(cols - 1)]
    dr = [p[r + 1, c] - p[r, c] for r in range(rows - 1) for c in range(cols)]
    out = {}
    for name, d in (('col', dc), ('row', dr)):
        if not d:
            out[name] = (np.array([0.0, 0.0]), 0.0)
            continue
        m = np.mean(d, axis=0)
        n = float(np.linalg.norm(m))
        out[name] = (m / n if n > 1e-9 else np.array([0.0, 0.0]), n)
    return {'u_col': out['col'][0], 'pitch_col': out['col'][1],
            'u_row': out['row'][0], 'pitch_row': out['row'][1]}


def measured_tilt(xy, z, rows, cols):
    """잰 표면 높이로 침대 기울기를 구한다.

    xy: 6칸 중심 좌표(베이스 기준), z: 칸별 표면 높이 (None = 측정 실패 → 빼고 계산)
    반환 {'col_deg': 열 방향 기울기(+ 면 2번 쪽이 높음), 'row_deg': 행 방향(+ 면 다음 행 쪽이 높음),
          'rms': 평면에서 벗어난 정도 mm, 'n': 사용한 점 수}  /  점이 모자라면 None
    """
    ax = bed_axes(xy, rows, cols)
    use = [(p, v) for p, v in zip(xy, z) if v is not None]
    if len(use) < 3:
        return None
    a, b, _, rms = fit_plane([p for p, _ in use], [v for _, v in use])
    grad = np.array([a, b])
    return {'col_deg': math.degrees(math.atan(float(grad @ ax['u_col']))),
            'row_deg': math.degrees(math.atan(float(grad @ ax['u_row']))),
            'rms': rms, 'n': len(use)}


def plan_moves(bed, rack, target):
    """침대 배치 bed 를 target 으로 바꾸는 이동 순서.

    이동 하나 = {'src': ('bed'|'rack', 번호), 'dst': ('bed'|'rack', 번호), 'h': 높이}
    규칙: 빈 칸이 생기면 바로 채운다(보관대에서, 없으면 자리가 틀린 다른 칸에서 직접).
          채울 수 있는 빈 칸이 없으면 자리가 틀린 블록 하나를 보관대 빈 자리로 뺀다.
    그래서 보관대에는 빈 자리가 1개만 있으면 된다.
    """
    bed, rack, target = list(bed), list(rack), list(target)
    n = len(bed)
    moves = []

    def wrong(i):
        return bed[i] != target[i]

    def find(seq, pred):
        return next((i for i, v in enumerate(seq) if pred(i, v)), None)

    for _ in range(4 * n + 4):
        if not any(wrong(i) for i in range(n)):
            return moves
        moved = False
        for i in range(n):                              # 1) 빈 칸 채우기
            if bed[i] is not None or target[i] is None:
                continue
            j = find(rack, lambda _, v: v == target[i])
            if j is not None:
                moves.append({'src': ('rack', j), 'dst': ('bed', i), 'h': rack[j]})
                bed[i], rack[j] = rack[j], None
                moved = True
                break
            s = find(bed, lambda s, v: s != i and v == target[i] and wrong(s))
            if s is not None:
                moves.append({'src': ('bed', s), 'dst': ('bed', i), 'h': bed[s]})
                bed[i], bed[s] = bed[s], None
                moved = True
                break
        if moved:
            continue
        cand = [i for i in range(n) if bed[i] is not None and wrong(i)]   # 2) 틀린 블록 빼기
        if not cand:
            raise PlanError('필요한 높이의 블록이 모자랍니다.')
        e = find(rack, lambda _, v: v is None)
        if e is None:
            raise PlanError('보관대에 빈 자리가 없어서 블록을 뺄 수 없습니다. 한 자리를 비워 주세요.')
        cand.sort(key=lambda i: 0 if target[i] in rack else 1)   # 빼면 바로 채울 수 있는 칸 먼저
        i = cand[0]
        moves.append({'src': ('bed', i), 'dst': ('rack', e), 'h': bed[i]})
        rack[e], bed[i] = bed[i], None
    raise PlanError('이동 순서를 만들지 못했습니다.')


def plan_replace(bed, rack, faulty):
    """불량 칸(faulty, 0부터)의 블록을 버리고 같은 높이 예비 블록을 넣는 이동 순서.

    반환 (moves, skipped): 같은 높이 예비가 없는 칸은 건드리지 않고 skipped 에 넣는다.
    """
    rack = list(rack)
    moves, skipped = [], []
    for i in faulty:
        h = bed[i]
        j = next((j for j, v in enumerate(rack) if v is not None and v == h), None)
        if j is None:
            skipped.append(i)
            continue
        moves.append({'src': ('bed', i), 'dst': ('discard', 0), 'h': h})
        moves.append({'src': ('rack', j), 'dst': ('bed', i), 'h': h})
        rack[j] = None
    return moves, skipped


def apply_move(bed, rack, mv):
    """이동 하나를 재고(bed, rack 리스트)에 반영한다."""
    (sk, si), (dk, di) = mv['src'], mv['dst']
    src = bed if sk == 'bed' else rack
    h = src[si]
    src[si] = None
    if dk == 'bed':
        bed[di] = h
    elif dk == 'rack':
        rack[di] = h
    return h


def move_text(mv):
    """이동 하나를 사람이 읽는 문장으로."""
    names = {'bed': '침대 %d번', 'rack': '보관대 %d번'}
    (sk, si), (dk, di) = mv['src'], mv['dst']
    src = names[sk] % (si + 1)
    dst = '버림' if dk == 'discard' else names[dk] % (di + 1)
    return f'{src} ({mv["h"]:g}mm) → {dst}'


# ── 커버 원호 (movec) ─────────────────────────────────────────────────
# 좌표 표기는 두산 posx 와 같다: [x, y, z, A, B, C] (mm, deg), 자세는 ZYZ 오일러각.

def zyz_matrix(a, b, c):
    """두산 posx 자세 (A, B, C) [deg] → 3x3 회전행렬 (Rz(A)·Ry(B)·Rz(C)). 3번째 열이 툴 Z축."""
    a, b, c = (math.radians(v) for v in (a, b, c))
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    rz = lambda c_, s_: np.array([[c_, -s_, 0.0], [s_, c_, 0.0], [0.0, 0.0, 1.0]])   # noqa: E731
    ry = np.array([[cb, 0.0, sb], [0.0, 1.0, 0.0], [-sb, 0.0, cb]])
    return rz(ca, sa) @ ry @ rz(cc, sc)


def zyz_from_matrix(m):
    """회전행렬 → 두산 posx 자세 (A, B, C) [deg] (zyz_matrix 의 역). B 는 0~180."""
    m = np.asarray(m, dtype=float)
    cb = max(-1.0, min(1.0, float(m[2, 2])))
    b = math.acos(cb)
    if math.sin(b) > 1e-6:
        a = math.atan2(m[1, 2], m[0, 2])
        c = math.atan2(m[2, 1], -m[2, 0])
    else:                                   # 특이자세: A 와 C 가 합쳐진다. C = 0 으로 둔다
        c = 0.0
        a = math.atan2(m[1, 0], m[0, 0]) if cb > 0 else math.atan2(-m[0, 1], -m[0, 0])
    return [math.degrees(a), math.degrees(b), math.degrees(c)]


def rot_angle_deg(m):
    """회전행렬이 몇 도 회전인지."""
    return math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(m)) - 1.0) / 2.0))))


def _rot_power(m, t):
    """회전행렬 m 의 t 배 회전 (같은 축, 각도 x t)."""
    ang = math.radians(rot_angle_deg(m))
    if ang < 1e-9:
        return np.eye(3)
    ax = np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])
    ax = ax / np.linalg.norm(ax)
    k = np.array([[0.0, -ax[2], ax[1]], [ax[2], 0.0, -ax[0]], [-ax[1], ax[0], 0.0]])
    th = ang * t
    return np.eye(3) + math.sin(th) * k + (1.0 - math.cos(th)) * (k @ k)


def tool_z(pose):
    """posx 의 툴 Z축(그리퍼가 향하는 방향) 단위벡터, 베이스 기준."""
    return zyz_matrix(*pose[3:6])[:, 2]


def tilt_deg(pose):
    """툴 Z축이 '바로 아래(-Z)'에서 몇 도 기울었는지."""
    return math.degrees(math.acos(max(-1.0, min(1.0, -tool_z(pose)[2]))))


def circle_3pt(p1, p2, p3):
    """세 점을 지나는 원 → (중심, 반지름, 회전축 단위벡터). 축 방향은 p1→p2 로 도는 쪽이 + (오른손 법칙)."""
    p1, p2, p3 = (np.asarray(p, dtype=float) for p in (p1, p2, p3))
    a, b = p1 - p3, p2 - p3
    axb = np.cross(a, b)
    nn = float(axb @ axb)
    if nn < 1e-6 * float(a @ a) * float(b @ b):
        raise ValueError('원호 세 점이 한 직선 위에 있습니다 (원이 정해지지 않음).')
    center = p3 + np.cross(float(a @ a) * b - float(b @ b) * a, axb) / (2.0 * nn)
    axis = np.cross(p1 - center, p2 - center)
    axis = axis / np.linalg.norm(axis)
    return center, float(np.linalg.norm(p1 - center)), axis


def _angle_about(axis, u, v):
    """축 axis 를 기준으로 u → v 로 돈 각도 [deg], 0~360."""
    return math.degrees(math.atan2(float(axis @ np.cross(u, v)), float(u @ v))) % 360.0


def rotate_about(p, center, axis, deg):
    """점 p 를 (center 를 지나는 axis 축) 둘레로 deg 만큼 돌린다 (로드리게스 공식)."""
    v = np.asarray(p, dtype=float) - center
    t = math.radians(deg)
    k = axis
    return center + v * math.cos(t) + np.cross(k, v) * math.sin(t) + k * float(k @ v) * (1.0 - math.cos(t))


def hinge_from_demo(demo_start, demo_via):
    """데모 시작점(뚜껑 닫힘)과 경유점(원호 끝)으로 경첩 위치 (y, z) 를 구한다. 경첩 축은 베이스 X 방향.

    가정: 닫힌 뚜껑은 수평이고 경첩은 고리와 같은 높이에 있다 → 경첩 z = 시작점 z.
          시작점과 경유점은 경첩에서 같은 거리(원 위) → 경첩 y 는 두 점의 수직이등분선과 z=시작점z 의 교점.
    """
    ys, zs = float(demo_start[1]), float(demo_start[2])
    yv, zv = float(demo_via[1]), float(demo_via[2])
    if abs(ys - yv) < 1.0:
        raise ValueError('데모 시작점과 경유점의 y 가 같아 경첩을 정할 수 없습니다.')
    h = zv - zs
    yc = (ys * ys - yv * yv - h * h) / (2.0 * (ys - yv))
    return yc, zs


def _lid_angle(y, z, yc, zc):
    """경첩에서 본 고리 각도 [deg]: 0 = 닫힘(고리가 +y 쪽 수평), 90 = 수직, 90 넘으면 경첩 너머로 넘어감."""
    return math.degrees(math.atan2(z - zc, y - yc))


def cover_path(demo_start, demo_via, demo_end, grip_xyz, grip_ori, above_dz, retreat,
               rest_deg=110.0, line_segments=1, close_margin_deg=0.0, min_open_deg=92.0, max_axial_offset=15.0,
               max_grip_angle=10.0, min_radius=50.0, hinge_at_grip=False):
    """커버 열기/닫기 좌표를 만든다 (데모 방식: 원호 + y·z 직선).

    데모 동작: 고리를 수직으로 내려가 잡고 → 시작점에서 경유점까지 원호(movec) → 경유점에서 끝점까지
              x 는 그대로 두고 y, z 만 바꾸는 직선 이동 → 놓으면 뚜껑이 베드 옆 받침(약 110도)에 걸친다.
    경첩: hinge_from_demo (시작점과 경유점이 같은 원 위, 경첩은 닫힌 고리와 같은 높이). 축은 베이스 X.
    고리를 다른 곳(grip_xyz, 보통 y 가 다름)에서 잡으면 같은 경첩 둘레로 반지름만 바뀐다:
      경유점 = 같은 각도(원 위),  끝점 = 데모 끝점과 같은 각도 + 같은 비율의 거리(데모에서 원보다 조금 밖으로 당긴 만큼).
    x 는 처음부터 끝까지 grip_xyz 의 x 그대로.
    자세: 잡을 때 grip_ori(수직), 경유점/끝점은 데모 자세, 원호 중간점은 잡는 자세와 경유 자세의 중간.
          grip_ori 가 데모 시작 자세와 다르면(예: 그리퍼를 툴 Z 둘레로 돌려 잡음) 데모 자세를 그대로 쓰지 않고,
          데모에서 시작 → 경유/끝 사이에 일어난 자세 변화(베이스 기준 회전)를 grip_ori 에 똑같이 적용한다.
          그래야 고리를 잡은 채 손목이 돌지 않는다. 중간 자세는 회전을 고르게 나눠서 만든다.
    hinge_at_grip: True 면 경첩 높이를 grip_xyz 의 z 에 맞춘다 (데모 좌표 전체를 z 로 평행 이동한 것과 같음).
          "경첩은 닫힌 고리와 같은 높이" 가정을 새 시작점에 그대로 적용하는 것. False 면 데모 시작점 높이.

    경첩 보호용 검사 (어기면 ValueError):
      - x 가 데모 시작점(경첩 두 개의 가운데)에서 max_axial_offset 넘게 벗어나면 거부
      - 잡는 점이 닫힌 뚜껑 높이에서 max_grip_angle 넘게 벗어나면 거부 (고리 높이 오입력)
      - 끝점 각도는 min_open_deg 이상(수직을 넘겨야 놓았을 때 받침 쪽으로 넘어감), rest_deg 미만(받침을 밀지 않음)

    돌려주는 dict (좌표는 posx 6개 리스트):
      above, grip   잡기 전 수직 위 / 잡는 자세 (원호 시작)
      mid, via      원호 중간점(movec 의 첫 점) / 원호 끝 = 직선 시작
      line          경유점 → 끝점 직선 구간의 목표점 목록 (마지막이 end). line_segments=1 이면 [end] (데모처럼 한 번에).
                    n 개로 나누면 경첩 둘레를 따라가는 n 개의 짧은 직선 (각도/거리/자세를 고르게 나눔, x 고정).
                    한 번에 가면 직선이 원 안쪽으로 파고든다 (반지름 190mm, 67도 구간이면 가운데에서 약 32mm).
      end           직선 끝 = 놓는 자리
      end_app       end 에서 툴 -Z 로 retreat 만큼 뺀 자리 (놓고 빠질 때)
      rest          놓은 뒤 뚜껑이 받침(rest_deg)에 걸쳤을 때의 고리 자리 (닫을 때 잡는 곳, 자세는 끝점 자세)
      rest_app      rest 에서 툴 -Z 로 retreat 만큼 뺀 자리 (닫을 때 들어가는 곳)
      close_end     닫기 원호 끝: 완전히 닫히기 close_margin_deg 전
      확인용: hinge [x, y, z], radius, via_deg, end_deg, rest_deg, end_reach, off_axial, grip_angle, end_tilt_deg
    """
    if hinge_at_grip:
        dz = float(grip_xyz[2]) - float(demo_start[2])
        demo_start, demo_via, demo_end = ([p[0], p[1], p[2] + dz] + list(p[3:6])
                                          for p in (demo_start, demo_via, demo_end))
    yc, zc = hinge_from_demo(demo_start, demo_via)
    r_demo = math.hypot(demo_start[1] - yc, demo_start[2] - zc)
    via_deg = _lid_angle(demo_via[1], demo_via[2], yc, zc)
    end_deg = _lid_angle(demo_end[1], demo_end[2], yc, zc)
    end_reach = math.hypot(demo_end[1] - yc, demo_end[2] - zc) / r_demo
    if not 5.0 < via_deg < end_deg:
        raise ValueError(f'데모 좌표 순서가 이상합니다 (경유 {via_deg:.1f}도, 끝 {end_deg:.1f}도).')
    if end_deg < min_open_deg:
        raise ValueError(f'끝점 각도 {end_deg:.1f}도가 {min_open_deg:.0f}도보다 작습니다. '
                         '수직을 넘지 않고 놓으면 뚜껑이 닫히며 떨어집니다.')
    if end_deg >= rest_deg:
        raise ValueError(f'끝점 각도 {end_deg:.1f}도가 받침 각도 {rest_deg:.0f}도 이상입니다. 로봇이 받침을 밀게 됩니다.')
    if not 0.0 <= close_margin_deg < via_deg:
        raise ValueError(f'닫기 여유 각도 {close_margin_deg}도가 이상합니다 (0 이상, 경유 {via_deg:.1f}도 미만).')

    gx, gy, gz = (float(v) for v in grip_xyz)
    off_axial = gx - float(demo_start[0])
    if abs(off_axial) > max_axial_offset:
        raise ValueError(f'잡는 점 x 가 경첩 두 개의 가운데(데모 시작점 x {demo_start[0]})에서 {off_axial:+.1f}mm '
                         f'벗어났습니다 (허용 ±{max_axial_offset:.0f}mm). 한쪽 경첩에 힘이 몰립니다.')
    grip_angle = _lid_angle(gy, gz, yc, zc)
    if abs(grip_angle) > max_grip_angle:
        raise ValueError(f'잡는 점이 닫힌 뚜껑 면에서 {grip_angle:+.1f}도 벗어났습니다 (허용 ±{max_grip_angle:.0f}도). '
                         '고리 높이(z)를 확인하세요.')
    r = math.hypot(gy - yc, gz - zc)
    if r < min_radius:
        raise ValueError(f'잡는 점이 경첩에 너무 가깝습니다 (반지름 {r:.1f}mm < {min_radius:.0f}mm).')

    def at(deg, reach=1.0):
        t = math.radians(deg)
        return [gx, yc + r * reach * math.cos(t), zc + r * reach * math.sin(t)]

    def pose(xyz, ori):
        return [round(float(v), 3) for v in xyz] + [round(float(o), 3) for o in ori]

    def back(p):
        return pose(np.asarray(p[:3]) - tool_z(p) * retreat, p[3:6])

    if int(line_segments) < 1:
        raise ValueError('line_segments 는 1 이상이어야 합니다.')
    n = int(line_segments)
    r_s, r_g = zyz_matrix(*demo_start[3:6]), zyz_matrix(*grip_ori)
    if rot_angle_deg(r_g @ r_s.T) > 0.5:
        # 잡는 자세가 데모와 다름 → 데모의 자세 변화(베이스 기준)를 잡는 자세에 적용
        rel_v = zyz_matrix(*demo_via[3:6]) @ r_s.T
        rel_e = zyz_matrix(*demo_end[3:6]) @ r_s.T
        def ori_of(m):      # 같은 자세를 잡는 자세와 같은 B 부호로 표기 (읽기 쉽게. 두 표기는 같은 자세)
            a, b, c = zyz_from_matrix(m)
            if float(grip_ori[1]) < 0.0 < b:
                a, b, c = (a + 360.0) % 360.0 - 180.0, -b, (c + 360.0) % 360.0 - 180.0
            return [a, b, c]
        via_ori = ori_of(rel_v @ r_g)
        end_ori = ori_of(rel_e @ r_g)
        mid_ori = ori_of(_rot_power(rel_v, 0.5) @ r_g)
        rel_ve = rel_e @ rel_v.T
        line_ori = [ori_of(_rot_power(rel_ve, i / n) @ rel_v @ r_g) for i in range(1, n)]
    else:
        via_ori, end_ori = list(demo_via[3:6]), list(demo_end[3:6])
        mid_ori = [(a + b) / 2.0 for a, b in zip(grip_ori, demo_via[3:6])]
        line_ori = [[a + (b - a) * (i / n) for a, b in zip(demo_via[3:6], demo_end[3:6])] for i in range(1, n)]
    grip = pose([gx, gy, gz], grip_ori)
    end = pose(at(end_deg, end_reach), end_ori)
    rest = pose(at(rest_deg), end_ori)
    line = []
    for i in range(1, n):
        t = i / n
        line.append(pose(at(via_deg + (end_deg - via_deg) * t, 1.0 + (end_reach - 1.0) * t), line_ori[i - 1]))
    line.append(end)
    return {
        'above': pose([gx, gy, gz + above_dz], grip_ori), 'grip': grip,
        'mid': pose(at((grip_angle + via_deg) / 2.0), mid_ori), 'via': pose(at(via_deg), via_ori),
        'line': line, 'end': end, 'end_app': back(end), 'rest': rest, 'rest_app': back(rest),
        'close_end': pose(at(grip_angle + close_margin_deg), grip_ori),
        'hinge': [gx, yc, zc], 'radius': r, 'demo_radius': r_demo, 'via_deg': via_deg, 'end_deg': end_deg,
        'rest_deg': float(rest_deg), 'close_deg': float(close_margin_deg), 'end_reach': end_reach,
        'off_axial': off_axial, 'grip_angle': grip_angle, 'end_tilt_deg': tilt_deg(end),
    }


def _rel_oris(demo_start, demo_via, demo_end, grip_ori, line_fracs=()):
    """데모 시작 → 경유/끝 자세 변화(베이스 기준 회전)를 grip_ori 에 적용한 자세들.
    돌려줌: (원호 중간, 경유, 끝, [경유→끝 사이 line_fracs 자세들]). 표기는 grip_ori 와 같은 B 부호."""
    r_s, r_g = zyz_matrix(*demo_start[3:6]), zyz_matrix(*grip_ori)

    def ori_of(m):
        a, b, c = zyz_from_matrix(m)
        if float(grip_ori[1]) < 0.0 < b:
            a, b, c = (a + 360.0) % 360.0 - 180.0, -b, (c + 360.0) % 360.0 - 180.0
        return [round(a, 3), round(b, 3), round(c, 3)]
    rel_v = zyz_matrix(*demo_via[3:6]) @ r_s.T
    rel_e = zyz_matrix(*demo_end[3:6]) @ r_s.T
    rel_ve = rel_e @ rel_v.T
    return (ori_of(_rot_power(rel_v, 0.5) @ r_g), ori_of(rel_v @ r_g), ori_of(rel_e @ r_g),
            [ori_of(_rot_power(rel_ve, t) @ rel_v @ r_g) for t in line_fracs], ori_of)


def cover_path_delta(demo_start, demo_via, demo_end, start, arc_radius, above_dz, retreat,
                     rest_deg=110.0, close_margin_deg=2.0, keep_x=True, min_open_deg=90.0, lid_len=None,
                     exit_clear=50.0, arc_deg=None, end_deg_on_circle=None, via_dir_deg=None, end_dir_deg=None,
                     single_arc=False, hinge_rel=None):
    """데모의 좌표 차이를 새 시작점에 그대로 옮긴 커버 열기/닫기 좌표 (2026-11-19 형님 방식).

    start: 새 시작점 posx 6개 (고리를 잡은 TCP).
    경유점 = 시작점 + (데모 경유점 - 데모 시작점), 끝점 = 시작점 + (데모 끝점 - 데모 시작점).
      keep_x=True 면 x 는 시작점 x 그대로 (데모의 x 차이 -9.4mm 는 경첩 축 방향이라 뺌).
    원호: 시작점과 경유점을 지나는 반지름 arc_radius 원 (경첩 쪽 = y 가 작은 쪽 중심). 원호 중간점은 이 원 위.
    경유점 → 끝점: 데모처럼 한 번에 y·z 직선.
    자세: 데모의 자세 변화를 시작 자세에 적용 (원호 동안 거의 그대로, 끝점에서 경첩 축 둘레로 기욺).
    닫기: 받침(rest_deg)에 걸친 고리를 잡고 같은 원 위로 원호 한 번(close_mid → close_end, 닫히기 close_margin_deg 전).
    arc_deg: 주면 경유점을 데모 차이 대신 같은 원 위 '시작점에서 arc_deg 만큼 돈 자리'에 둔다 (원 중심은 그대로
      데모 경유 차이로 정함 → 경첩 높이는 데모와 맞춤). 자세는 데모 경유 자세 변화 그대로.
    각도는 모두 '뚜껑 각도' = 닫힌 상태(시작점)에서 경첩 둘레로 연 각도 (0=닫힘, 90=수직). rest_deg, min_open_deg 도 같음.
    (시작점이 경첩보다 낮아서 경첩에서 본 고리 방향은 이 값보다 grip_angle 만큼 작다)
    end_dir_deg: 주면 끝점을 '경첩에서 본 고리 방향이 수평에서 이 각도'인 원 위 자리에 둔다 (end_deg_on_circle 보다 우선).
    hinge_rel: 주면 경첩 축 중심 = 시작점 + hinge_rel [dx, dy, dz] (형님 실측: [0, -130, 0]). 반지름은 그 거리로 정해지고
      arc_radius 와 데모 경유 차이로 중심을 구하는 계산은 쓰지 않는다.
    single_arc: True 면 시작점 → 끝점을 moveC 한 번으로 (경유점 = 끝점, 직선 없음). 중간점은 원 위 가운데 각도.
    rest_deg=None 이면 받침 자리 = 끝점 (놓은 자리에 뚜껑이 그대로 걸친다고 봄).
    via_dir_deg: 주면 경유점을 '경첩에서 본 고리 방향이 수평(+y)에서 이 각도'인 자리에 둔다 (arc_deg 보다 우선).
    end_deg_on_circle: 주면 끝점을 데모 차이 대신 같은 원 위 이 뚜껑 각도에 둔다.
      고리를 쥔 채로 갈 수 있는 자리 (원 밖으로 당기지 않음). 자세는 데모 끝 자세 변화 그대로.
    lid_len: 경첩에서 뚜껑 끝까지 길이 [mm]. 주면 받침(rest_deg)에 걸친 뚜껑 끝을 피해 도는 우회 경로를 만든다:
      열기 후  end_app → exit_out(뚜껑 끝보다 exit_clear 만큼 바깥(-y)으로) → exit_up(뚜껑 끝보다 exit_clear 위로)
               → exit_over(시작점 위 같은 높이, 잡는 자세) → 홈
      닫기 전  홈 → exit_over → rest_up(rest_app 위 같은 높이) → rest_app → rest  (rest_app 은 뚜껑 안쪽(+y)이라 뚜껑을 안 넘음)
    돌려주는 dict 는 cover_path 와 같은 키 + close_direct=True, close_mid, end_out(끝점이 원에서 벗어난 거리),
      exit_up / exit_over / rest_up (exit_z 를 줬을 때).
    """
    s = [float(v) for v in start[:3]]
    grip_ori = [float(v) for v in start[3:6]]
    dv = [float(a) - float(b) for a, b in zip(demo_via[:3], demo_start[:3])]
    de = [float(a) - float(b) for a, b in zip(demo_end[:3], demo_start[:3])]
    if keep_x:
        dv[0] = de[0] = 0.0
    via_xyz = [a + b for a, b in zip(s, dv)]
    end_xyz = [a + b for a, b in zip(s, de)]
    # 시작점과 경유점을 지나는 반지름 R 원의 중심 (y-z 평면)
    R = float(arc_radius)
    p1, p2 = np.array(s[1:3]), np.array(via_xyz[1:3])
    chord = p2 - p1
    half = float(np.linalg.norm(chord)) / 2.0
    if half < 1.0 or half >= R:
        raise ValueError(f'반지름 {R:.0f}mm 로는 시작점과 경유점(거리 {2 * half:.1f}mm)을 지나는 원이 안 됩니다.')
    m = (p1 + p2) / 2.0
    perp = np.array([-chord[1], chord[0]]) / (2.0 * half)
    h = math.sqrt(R * R - half * half)
    c1, c2 = m + perp * h, m - perp * h
    yc, zc = (c1 if c1[0] < c2[0] else c2).tolist()
    if hinge_rel is not None:
        yc, zc = s[1] + float(hinge_rel[1]), s[2] + float(hinge_rel[2])
        R = math.hypot(s[1] - yc, s[2] - zc)
    grip_angle = _lid_angle(s[1], s[2], yc, zc)

    def on_circle(deg):
        t = math.radians(deg)
        return [s[0] if keep_x else via_xyz[0], yc + R * math.cos(t), zc + R * math.sin(t)]
    if via_dir_deg is not None:
        via_xyz = on_circle(float(via_dir_deg))
    elif arc_deg is not None:
        via_xyz = on_circle(grip_angle + float(arc_deg))
    if end_dir_deg is not None:
        end_xyz = on_circle(float(end_dir_deg))
    elif end_deg_on_circle is not None:
        end_xyz = on_circle(grip_angle + float(end_deg_on_circle))
    if single_arc:
        via_xyz = list(end_xyz)
    via_dir = _lid_angle(via_xyz[1], via_xyz[2], yc, zc)          # 경첩에서 본 고리 방향 (0 = +y 수평)
    end_dir = _lid_angle(end_xyz[1], end_xyz[2], yc, zc)
    via_deg, end_deg = via_dir - grip_angle, end_dir - grip_angle  # 뚜껑 각도 (0 = 닫힘)
    end_out = math.hypot(end_xyz[1] - yc, end_xyz[2] - zc) - R
    if rest_deg is None:
        rest_deg = end_deg
    if not 0.0 < via_deg <= end_deg:
        raise ValueError(f'각도 순서가 이상합니다 (경유 {via_deg:.1f}, 끝 {end_deg:.1f}도).')
    if end_deg < min_open_deg:
        raise ValueError(f'끝점 각도 {end_deg:.1f}도가 {min_open_deg:.0f}도보다 작습니다. 놓으면 뚜껑이 닫히며 떨어집니다.')
    # 아래 두 가지는 데모 좌표를 그대로 옮긴 결과라 막지 않고 경고로 남긴다 (실물 전에 확인할 것)
    warnings = []
    if end_deg > rest_deg + 1e-6:
        warnings.append(f'끝점 방향이 경첩에서 {end_deg:.1f}도로 받침 {rest_deg:.0f}도보다 더 넘어감')
    if end_out > 10.0:
        warnings.append(f'끝점이 반지름 {R:.0f}mm 원보다 {end_out:.0f}mm 바깥 (고리를 쥔 채로는 갈 수 없는 거리, 데모에서는 고리가 손가락에서 미끄러졌다고 봐야 함)')
    x = s[0]

    def pose(xyz, ori):
        return [round(float(v), 3) for v in xyz] + [round(float(o), 3) for o in ori]

    def back(p):
        return pose(np.asarray(p[:3]) - tool_z(p) * retreat, p[3:6])

    def at(lid_deg):            # 뚜껑 각도 → 고리 자리
        t = math.radians(grip_angle + lid_deg)
        return [x, yc + R * math.cos(t), zc + R * math.sin(t)]

    mid_ori, via_ori, end_ori, _, ori_of = _rel_oris(demo_start, demo_via, demo_end, grip_ori)
    if single_arc:              # 원호 한 번: 끝 자세까지 원호 동안 고르게 기운다
        via_ori = end_ori
        r_e0, r_g0 = zyz_matrix(*end_ori), zyz_matrix(*grip_ori)
        mid_ori = ori_of(_rot_power(r_e0 @ r_g0.T, 0.5) @ r_g0)
    end = pose(end_xyz, end_ori)
    rest = pose(at(rest_deg), end_ori)
    close_deg = close_margin_deg
    # 닫기 원호 중간 자세: 끝 자세와 잡는 자세의 가운데 (회전을 반으로)
    r_e, r_g = zyz_matrix(*end_ori), zyz_matrix(*grip_ori)
    close_mid_ori = ori_of(_rot_power(r_g @ r_e.T, 0.5) @ r_e)
    end_app, rest_app = back(end), back(rest)
    exits = {}
    if lid_len:
        t = math.radians(rest_deg)
        tip_y, tip_z = yc + lid_len * math.cos(t), zc + lid_len * math.sin(t)      # 받침에 걸친 뚜껑 끝
        y_lid = yc + (end_app[2] - zc) * math.cos(t) / math.sin(t)       # 빠진 높이에서 뚜껑 면의 y
        if end_app[1] > y_lid:          # 뚜껑 안쪽(+y, 침대 쪽)으로 빠졌음 → 뚜껑이 바깥으로 기울어 있으니 바로 위로 올라가면 됨
            out_y = end_app[1]
        else:                           # 뚜껑 바깥(-y)으로 빠졌음 → 뚜껑 끝보다 더 바깥으로 나간 뒤 올라감
            out_y = min(end_app[1], tip_y - exit_clear)
        up_z = max(tip_z + exit_clear, end_app[2], rest_app[2], s[2] + above_dz)
        exits = {'exit_out': pose([end_app[0], out_y, end_app[2]], end_ori),
                 'exit_up': pose([end_app[0], out_y, up_z], end_ori),
                 'exit_over': pose([x, s[1], up_z], end_ori),   # 자세는 그대로 두고 평행 이동만 (직선 중 손목 회전 X)
                 'rest_up': pose([rest_app[0], rest_app[1], up_z], end_ori),
                 'lid_tip': [x, round(tip_y, 3), round(tip_z, 3)]}
    return {
        **exits,
        'above': pose([x, s[1], s[2] + above_dz], grip_ori), 'grip': pose(s, grip_ori),
        'mid': pose(at(via_deg / 2.0), mid_ori), 'via': pose(via_xyz, via_ori),
        'line': [] if single_arc else [end], 'end': end, 'single_arc': bool(single_arc), 'end_app': end_app, 'rest': rest, 'rest_app': rest_app,
        'close_direct': True, 'close_mid': pose(at((rest_deg + close_deg) / 2.0), close_mid_ori),
        'close_end': pose(at(close_deg), grip_ori),
        'hinge': [x, yc, zc], 'radius': R, 'demo_radius': R, 'via_deg': via_deg, 'end_deg': end_deg,
        'rest_deg': float(rest_deg), 'close_deg': float(close_margin_deg), 'end_reach': (R + end_out) / R,
        'end_out': end_out, 'off_axial': x - float(demo_start[0]), 'grip_angle': grip_angle,
        'end_tilt_deg': tilt_deg(end), 'warnings': warnings,
    }


def cover_path_taught(start, lift, end, above_dz, retreat, inset=8.0, close_margin_deg=2.0, lid_len=None,
                      exit_clear=50.0, radius=None):
    """형님이 실물에서 교시한 3점으로 커버 열기 (2026-11-19).

    start: 고리를 잡는 시작점 (손가락을 고리 가운데 구멍에 넣은 자리), lift: 원호 1이 깔끔하게 가는 마지막 점(약 70~80도),
    end: 끝점(약 104도, 근처에서 놓음). 셋 다 posx 6개 (TCP GripperDA_v1, BASE).
    경첩: 시작점과 같은 높이(z), x 축 방향이라고 보고 start 와 lift 로 중심을 구한다 (둘 다 경첩에서 같은 거리).
    원호 1 (start → lift): 경첩 원 위 moveC. 중간점은 원 위 가운데 각도, 자세는 두 교시 자세의 가운데.
    원호 2 (lift → end): 80도를 넘으면 뚜껑 무게가 경첩 너머로 넘어가 고리가 손가락에 걸려 들린다 → 원보다 inset 만큼
      안쪽(경첩 쪽)을 지나는 moveC 로 고리를 당기지 않고 밀듯이 간다. 중간점 = 원 위 가운데 각도에서 inset 만큼 안쪽.
    닫기: end 에서 다시 잡고 원호 2 거꾸로(end → lift) → 원호 1 거꾸로 (lift → 닫히기 close_margin_deg 전).
    """
    s, q, e = ([float(v) for v in pt] for pt in (start, lift, end))
    ys, zs, yq, zq = s[1], s[2], q[1], q[2]
    h = zq - zs
    if abs(ys - yq) < 1.0:
        raise ValueError('시작점과 lift 점의 y 가 같아 경첩을 정할 수 없습니다.')
    if radius:          # 실측 반지름: 경첩 = 시작점에서 -y 로 radius (같은 높이)
        yc, zc, R = ys - float(radius), zs, float(radius)
    else:               # 시작점과 lift 점이 경첩에서 같은 거리라고 보고 계산
        yc = (ys * ys - yq * yq - h * h) / (2.0 * (ys - yq))
        zc = zs
        R = math.hypot(ys - yc, 0.0)
    x = s[0]

    def ang(p):
        return math.degrees(math.atan2(p[2] - zc, p[1] - yc))

    def at(deg, r=R):
        t = math.radians(deg)
        return [x, yc + r * math.cos(t), zc + r * math.sin(t)]

    def pose(xyz, ori):
        return [round(float(v), 3) for v in xyz] + [round(float(o), 3) for o in ori]

    def mid_ori(a, b):
        ra, rb = zyz_matrix(*a), zyz_matrix(*b)
        m = zyz_from_matrix(_rot_power(rb @ ra.T, 0.5) @ ra)
        if float(a[1]) < 0.0 < m[1]:
            m = [(m[0] + 360.0) % 360.0 - 180.0, -m[1], (m[2] + 360.0) % 360.0 - 180.0]
        return m

    def back(p):
        return pose(np.asarray(p[:3]) - tool_z(p) * retreat, p[3:6])

    a_lift, a_end = ang(q), ang(e)
    e_out = math.hypot(e[1] - yc, e[2] - zc) - R
    if not 0.0 < a_lift < a_end:
        raise ValueError(f'교시 각도 순서가 이상합니다 (lift {a_lift:.1f}도, 끝 {a_end:.1f}도).')
    warnings = []
    if abs(e_out) > 10.0:
        warnings.append(f'끝점이 경첩 원(반지름 {R:.0f}mm)에서 {e_out:+.0f}mm 벗어나 있음')
    if a_end < 90.0:
        warnings.append(f'끝점 {a_end:.1f}도가 수직을 안 넘음 (놓으면 뚜껑이 닫히는 쪽으로 떨어질 수 있음)')
    grip, lift_p, end_p = pose(s[:3], s[3:6]), pose(q[:3], q[3:6]), pose(e[:3], e[3:6])
    r2 = (R + R + e_out) / 2.0 - float(inset)
    arc2_mid = pose(at((a_lift + a_end) / 2.0, r2), mid_ori(q[3:6], e[3:6]))
    arc2_mid[0] = round((q[0] + e[0]) / 2.0, 3)                 # 교시 x 가 몇 mm 다르면 중간점 x 는 가운데로
    mid1 = pose(at(a_lift / 2.0), mid_ori(s[3:6], q[3:6]))
    mid1[0] = round((s[0] + q[0]) / 2.0, 3)
    end_app = back(end_p)
    close_end = pose(at(close_margin_deg), s[3:6])
    exits = {}
    if lid_len:
        t = math.radians(a_end)                       # 놓은 자리 근처에 뚜껑이 걸친다고 봄 (받침 각도 모름)
        tip_y, tip_z = yc + lid_len * math.cos(t), zc + lid_len * math.sin(t)
        up_z = max(tip_z + exit_clear, end_app[2], s[2] + above_dz)
        exits = {'exit_out': end_app, 'exit_up': end_app[:2] + [round(up_z, 3)] + end_app[3:],
                 'exit_over': pose([x, s[1], up_z], e[3:6]), 'rest_up': end_app[:2] + [round(up_z, 3)] + end_app[3:],
                 'lid_tip': [x, round(tip_y, 3), round(tip_z, 3)]}
    return {
        **exits,
        'above': pose([x, s[1], s[2] + above_dz], s[3:6]), 'grip': grip,
        'mid': mid1, 'via': lift_p,
        'arc2_mid': arc2_mid, 'line': [], 'end': end_p, 'end_app': end_app,
        'rest': end_p, 'rest_app': end_app, 'close_taught': True,
        'close_mid': mid1, 'close_end': close_end,
        'hinge': [x, yc, zc], 'radius': R, 'via_deg': a_lift, 'end_deg': a_end, 'rest_deg': a_end,
        'grip_angle': 0.0, 'end_out': e_out, 'warnings': warnings, 'end_tilt_deg': tilt_deg(end_p),
    }
# ── 자동 보수: 빈칸 채우기 + 맞지 않는 모듈 교체 ──────────────────────
def grid_neighbors(i, rows, cols):
    """i번 칸(0부터)의 상하좌우 이웃 칸 번호."""
    r, c = divmod(i, cols)
    out = []
    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        rr, cc = r + dr, c + dc
        if 0 <= rr < rows and 0 <= cc < cols:
            out.append(rr * cols + cc)
    return out


def find_empties(blocks, empty_dz, tall_dz=None):
    """빈칸과 높은 큐브 찾기. blocks 의 각 dict 에 'grade' 를 'empty' / 'nodata' / None(아직 모름) 으로 채운다.

    기준 높이 = 강성을 잰 칸들의 표면 높이 중앙값 (보통 높이 큐브가 대부분이라는 가정).
    빈칸: 닿지 않음(no_contact), 또는 접촉 높이(z0)가 기준보다 empty_dz 이상 낮음.
    높은 큐브: 표면(없으면 접촉 높이)이 기준보다 tall_dz 이상 높음 → b['tall']=True (tall_dz 가 None 이면 보지 않음)
    너무 깊이 들어갔지만(too_deep) 빈칸이 아닌 칸은 '아주 무른 큐브'로 보고 b['too_soft']=True.
    그 밖의 오류로 강성을 못 잰 칸은 'nodata'.
    반환: 빈칸 번호 목록 (0부터)
    """
    surf = [b['z_surf'] for b in blocks if b.get('k') is not None and b.get('z_surf') is not None]
    if not surf:
        surf = [b['z0'] for b in blocks if b.get('z0') is not None]
    z_ref = statistics.median(surf) if surf else None
    out = []
    for i, b in enumerate(blocks):
        b['ref'] = b['ref_src'] = b['dev'] = None
        b['too_soft'] = b['tall'] = False
        low = z_ref is not None and b.get('z0') is not None and z_ref - b['z0'] >= empty_dz
        if b.get('error') == 'no_contact' or low:
            b['grade'] = 'empty'
            out.append(i)
            continue
        top = b.get('z_surf') if b.get('z_surf') is not None else b.get('z0')
        b['tall'] = bool(tall_dz is not None and z_ref is not None and top is not None and top - z_ref >= tall_dz)
        if b.get('error') == 'too_deep':
            b['too_soft'], b['grade'] = True, None
        elif b.get('k') is None and not b['tall']:
            b['grade'] = 'nodata'
        else:
            b['grade'] = None
    return out


def judge_auto(blocks, ratio):
    """자동(이웃 기준) 판정: 빈칸/측정 실패가 아닌 칸을 '다른 칸들의 중앙값'과 비교한다.

    높은 큐브이거나, 강성 비율이 ratio 미만 또는 1/ratio 초과면 'mismatch'(이 침대에 맞지 않는 모듈), 아니면 'ok'.
    중앙값을 쓰는 이유: 맞지 않는 칸 하나가 기준을 끌어내리지 않게.
    반환: 맞지 않는 칸 번호 목록 (0부터)
    """
    out = []
    for i, b in enumerate(blocks):
        if b['grade'] in ('empty', 'nodata'):
            continue
        others = [o['k'] for j, o in enumerate(blocks) if j != i and o.get('k') is not None
                  and o['grade'] not in ('empty', 'nodata') and not o.get('tall')]
        ref = statistics.median(others) if others else None
        if b.get('too_soft') or b.get('tall'):
            b['grade'] = 'mismatch'
            out.append(i)
            continue
        if ref is None:
            b['grade'] = 'ok'
            continue
        r = b['k'] / ref
        b['ref'], b['ref_src'], b['dev'] = float(ref), 'others', float(r - 1.0)
        bad = r < ratio or r > 1.0 / ratio
        b['grade'] = 'mismatch' if bad else 'ok'
        if bad:
            out.append(i)
    return out


def split_classes(ks, min_ratio):
    """강성 값들을 두 종류(부드러움/단단함)로 나누는 경계값. 로그 눈금에서 가장 크게 벌어진 곳의 가운데.

    가장 단단한 것 / 가장 무른 것 이 min_ratio 미만이면 한 종류뿐이라 None.
    """
    ks = sorted(k for k in ks if k is not None and k > 0)
    if len(ks) < 2 or ks[-1] / ks[0] < min_ratio:
        return None
    gaps = [(math.log(ks[n + 1] / ks[n]), n) for n in range(len(ks) - 1)]
    _, n = max(gaps)
    return float(math.sqrt(ks[n] * ks[n + 1]))


def class_of(k, thr, too_soft=False, tall=False):
    """종류: 'tall'(높은 큐브) / 'soft' / 'hard'. 높이가 먼저, 그다음 강성."""
    if tall:
        return 'tall'
    if too_soft:
        return 'soft'
    return 'soft' if k < thr else 'hard'


def class_centers(items, thr):
    """종류별 대표 강성(기하평균). items = [(k, tall), ...]. {'soft', 'hard', 'tall': k 또는 None}."""
    out = {}
    for c in ('soft', 'hard', 'tall'):
        v = [k for k, tall in items if k is not None and k > 0 and class_of(k, thr, tall=tall) == c]
        out[c] = float(math.exp(statistics.mean(math.log(k) for k in v))) if v else None
    return out


def judge_profile(blocks, profile, thr, centers):
    """사용자 지정 판정: 칸마다 원하는 종류(profile[i] = 'soft'|'hard'|'tall')와 잰 종류를 비교한다.

    다르면 'mismatch', 같으면 'ok'. 반환: 맞지 않는 칸 번호 목록 (0부터)
    """
    out = []
    for i, b in enumerate(blocks):
        if b['grade'] in ('empty', 'nodata'):
            continue
        b['cls'] = class_of(b.get('k'), thr, b.get('too_soft'), b.get('tall'))
        b['want'] = profile[i]
        ref = centers.get(profile[i])
        if ref and b.get('k') is not None:
            b['ref'], b['ref_src'], b['dev'] = float(ref), 'profile', float(b['k'] / ref - 1.0)
        b['grade'] = 'ok' if b['cls'] == profile[i] else 'mismatch'
        if b['grade'] == 'mismatch':
            out.append(i)
    return out


def plan_profile(todo, profile, cands, centers):
    """사용자 지정: 채울 칸마다 원하는 종류의 보관대 큐브를 고른다.

    cands: {보관대 자리: (종류, 침대 기준 강성)}. 같은 종류 안에서는 그 종류의 대표 강성과 비율이 가장 가까운 것.
    종류가 모자라면 PlanError (어느 칸을 못 채우는지 알려 준다).
    반환: [(칸, 자리), ...]
    """
    left = dict(cands)
    assign, short = [], []
    for i in todo:
        want = profile[i]
        pool = [j for j, (c, _) in left.items() if c == want]
        if not pool:
            short.append(i)
            continue
        ref = centers.get(want)
        j = min(pool, key=lambda j: (abs(math.log(left[j][1] / ref)) if ref and left[j][1] else 0.0, j))
        assign.append((i, j))
        del left[j]
    if short:
        names = {'hard': '단단한', 'soft': '부드러운', 'tall': '높은'}
        raise PlanError('보관대에 원하는 큐브가 모자랍니다: '
                        + ', '.join(f'{i + 1}번({names[profile[i]]})' for i in short))
    return assign


def plan_repair_moves(bed, rack, assign, mismatches):
    """보수 이동 순서. assign = [(칸, 보관대 자리), ...], mismatches = 맞지 않는 모듈이 있는 칸 (0부터).

    빈칸을 먼저 채우고(보관대에 빈 자리가 늘어난다), 맞지 않는 칸은
    '그 모듈을 보관대 빈 자리로 되돌리기 → 고른 큐브 넣기' 순서로 한다. 보관대에 빈 자리가 없으면 버린다.
    """
    bed, rack = list(bed), list(rack)
    mism = set(mismatches)
    moves = []
    for i, j in sorted(assign, key=lambda a: (a[0] in mism, a[0])):
        if i in mism:
            e = next((n for n, v in enumerate(rack) if v is None), None)
            dst = ('rack', e) if e is not None else ('discard', 0)
            moves.append({'src': ('bed', i), 'dst': dst, 'h': bed[i]})
            if e is not None:
                rack[e] = bed[i]
            bed[i] = None
        moves.append({'src': ('rack', j), 'dst': ('bed', i), 'h': rack[j]})
        bed[i], rack[j] = rack[j], None
    return moves


def plan_fill(empties, filled_k, cands, rows, cols, target=None, slot_target=None):
    """채울 칸(빈칸, 맞지 않는 모듈을 뺄 칸)에 넣을 보관대 큐브를 고른다.

    empties : 채울 칸 번호 목록 (0부터)
    filled_k: {칸 번호: 침대에서 잰 강성} 그대로 두는 칸
    cands   : {보관대 자리: 침대 기준으로 환산한 강성} 눌러 본 후보 큐브
    target  : 목표 강성. 없으면 그대로 두는 칸 평균
    slot_target: {칸 번호: 목표 강성} 사용자 지정 모드. 주면 그 칸은 이웃 대신 이 값에 맞춘다
    고르는 순서: ① 각 빈칸에 들어갈 큐브가 그 칸의 기준 강성과 비율로 가장 가까운 것
                   (기준 = 채워진 이웃 칸들의 평균, 이웃이 없으면 목표)
                ② 채운 뒤 전체 평균이 목표에 가까운 것
                ③ 보관대 앞자리 우선
    ①을 '비율(로그) 거리'로 보는 이유: 트레이↔침대 환산 비율(RACK_TO_BED)이 틀려도 모든 후보에 같은 배율로
    곱해지므로, 두 종류가 4배 차이 나는 한 환산이 2배 가까이 틀려도 고르는 결과가 바뀌지 않는다.
    (처음에는 '전체 평균을 목표에 맞추기'를 1순위로 했는데, 환산 오차를 메우려고 일부러 다른 종류를 섞는 문제가 있었다)
    반환 {'assign': [(칸, 자리), ...], 'target', 'mean_after', 'err'(평균 차이 비율)}
    """
    empties = list(empties)
    if not empties:
        return {'assign': [], 'target': target, 'mean_after': None, 'err': 0.0}
    if len(cands) < len(empties):
        raise PlanError(f'채울 칸은 {len(empties)}개인데 넣을 수 있는 보관대 큐브가 {len(cands)}개뿐입니다.')
    if target is None:
        vals = list(filled_k.values()) or list((slot_target or {}).values())
        if not vals:
            raise PlanError('그대로 두는 칸이 없어 목표 강성을 정할 수 없습니다.')
        target = statistics.mean(vals)

    if any(k <= 0 for k in cands.values()):
        raise PlanError('강성이 0 이하인 후보가 있습니다.')

    def nb_mean(i):
        if slot_target and i in slot_target:
            return slot_target[i]
        ks = [filled_k[j] for j in grid_neighbors(i, rows, cols) if j in filled_k]
        return statistics.mean(ks) if ks else target

    best = None
    for combo in itertools.combinations(sorted(cands), len(empties)):
        total = sum(filled_k.values()) + sum(cands[j] for j in combo)
        mean_after = total / (len(filled_k) + len(combo))
        err = abs(mean_after - target) / target
        for perm in itertools.permutations(combo):
            local = sum(abs(math.log(cands[j] / nb_mean(i))) for i, j in zip(empties, perm))
            key = (round(local, 4), round(err, 4), perm)
            if best is None or key < best[0]:
                best = (key, perm, mean_after, err)
    _, perm, mean_after, err = best
    return {'assign': list(zip(empties, perm)), 'target': float(target),
            'mean_after': float(mean_after), 'err': float(err)}


def cover_path_one_arc(start, end, radius, above_dz, retreat, close_margin_deg=2.0, lid_len=None, exit_clear=50.0,
                       hinge_below=True, keep_vertical=False, exit_z_min=0.0):
    """교시한 시작점·끝점 두 개를 반지름 radius 원호 하나(moveC 한 번)로 잇는 커버 열기 (2026-11-20 형님).

    원 중심(경첩) = 두 점에서 radius 거리인 두 후보 중 아래쪽(hinge_below) 것. 축은 베이스 X.
    중간점 = 원 위 가운데 각도, x 는 두 점의 평균, 자세는 두 교시 자세의 가운데.
    닫기: 끝점에서 다시 잡고 같은 원으로 (중간점 → 닫히기 close_margin_deg 전) moveC 한 번.
    """
    s, e = [float(v) for v in start], [float(v) for v in end]
    if keep_vertical:
        # 툴 Z 를 처음부터 끝까지 바로 아래(수직)로: 시작 자세의 방향(yaw)은 그대로, 기울기만 0 으로
        a, _, c = s[3:6]
        vert = [a, -180.0, c]
        s, e = s[:3] + vert, e[:3] + vert
    R = float(radius)
    p1, p2 = np.array(s[1:3]), np.array(e[1:3])
    ch = p2 - p1
    L = float(np.linalg.norm(ch))
    if L >= 2.0 * R:
        raise ValueError(f'시작점과 끝점 거리 {L:.1f}mm 가 지름 {2 * R:.0f}mm 보다 커서 반지름 {R:.0f}mm 원호로 못 잇습니다.')
    m = (p1 + p2) / 2.0
    perp = np.array([-ch[1], ch[0]]) / L
    h = math.sqrt(R * R - (L / 2.0) ** 2)
    c1, c2 = m + perp * h, m - perp * h
    yc, zc = (min if hinge_below else max)((c1, c2), key=lambda c: c[1])
    a_s = math.degrees(math.atan2(s[2] - zc, s[1] - yc))
    a_e = math.degrees(math.atan2(e[2] - zc, e[1] - yc))
    x = (s[0] + e[0]) / 2.0

    def at(deg):
        t = math.radians(deg)
        return [x, yc + R * math.cos(t), zc + R * math.sin(t)]

    def pose(xyz, ori):
        return [round(float(v), 3) for v in xyz] + [round(float(o), 3) for o in ori]

    def mid_ori(a, b):
        ra, rb = zyz_matrix(*a), zyz_matrix(*b)
        mo = zyz_from_matrix(_rot_power(rb @ ra.T, 0.5) @ ra)
        if float(a[1]) < 0.0 < mo[1]:
            mo = [(mo[0] + 360.0) % 360.0 - 180.0, -mo[1], (mo[2] + 360.0) % 360.0 - 180.0]
        return mo

    def back(p):
        return pose(np.asarray(p[:3]) - tool_z(p) * retreat, p[3:6])

    grip, end_p = pose(s[:3], s[3:6]), pose(e[:3], e[3:6])
    mid = pose(at((a_s + a_e) / 2.0), s[3:6] if keep_vertical else mid_ori(s[3:6], e[3:6]))
    end_app = back(end_p)
    close_end = pose(at(a_s + close_margin_deg), s[3:6])
    close_end[0] = round(s[0], 3)
    lid = a_e - a_s                                   # 닫힌 상태에서 연 각도
    warnings = [] if lid > 90.0 else [f'끝점이 닫힌 상태에서 {lid:.1f}도만 열린 자리라 수직을 안 넘음 (놓으면 닫힐 수 있음)']
    exits = {}
    if lid_len:
        # 놓은 뒤 뚜껑 면 각도 = 닫힌 상태(수평)에서 연 각도 (형님: 90도보다 조금 큼). 뚜껑 끝 = 경첩 + 길이 x 그 방향
        t = math.radians(lid)
        tip_y, tip_z = yc + lid_len * math.cos(t), zc + lid_len * math.sin(t)
        up_z = max(tip_z + exit_clear, end_app[2], s[2] + above_dz, float(exit_z_min or 0.0))
        exits = {'exit_out': end_app, 'exit_up': end_app[:2] + [round(up_z, 3)] + end_app[3:],
                 'exit_over': pose([s[0], s[1], up_z], e[3:6]), 'rest_up': end_app[:2] + [round(up_z, 3)] + end_app[3:],
                 'lid_tip': [x, round(tip_y, 3), round(tip_z, 3)], 'lid_len': float(lid_len)}
    return {
        **exits,
        'above': pose([s[0], s[1], s[2] + above_dz], s[3:6]), 'grip': grip,
        'mid': mid, 'via': end_p, 'line': [], 'end': end_p, 'end_app': end_app,
        'rest': end_p, 'rest_app': end_app, 'close_direct': True, 'close_mid': mid, 'close_end': close_end,
        'hinge': [x, yc, zc], 'radius': R, 'via_deg': lid, 'end_deg': lid, 'rest_deg': lid, 'grip_angle': a_s,
        'end_out': 0.0, 'warnings': warnings, 'end_tilt_deg': tilt_deg(end_p), 'single_arc': True,
    }
