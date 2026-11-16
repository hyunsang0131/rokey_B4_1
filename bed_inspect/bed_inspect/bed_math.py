# 계산만 하는 함수 모음 (로봇, ROS 와 무관 → 로봇 없이 테스트 가능)
#
#   fit_stiffness   누르기 곡선(높이, 힘) → 강성 k, 표면 높이
#   judge           블록별 강성 → 정상 / 주의 / 불량
#   measured_tilt   6개 표면 높이 → 침대 기울기
#   plan_tilt       목표 기울기 → 3가지 높이로 만들 수 있는 가장 가까운 배치
#   plan_moves      현재 배치 → 목표 배치로 가는 블록 이동 순서
#   plan_replace    불량 블록 → 같은 높이 예비 블록으로 바꾸는 이동 순서
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
    kh = [b['k'] * b['height'] for b in blocks if b.get('k') is not None]
    kh_med = statistics.median(kh) if kh else None
    for b in blocks:
        b['ref'] = b['ref_src'] = b['dev'] = None
        if b.get('error') in ('no_contact', 'too_deep'):
            b['grade'] = 'fail'          # 블록이 없거나 너무 물러서 바닥까지 들어감
            continue
        if b.get('k') is None:
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
    """칸 중심 좌표에서 침대의 열 방향(1→3번), 행 방향(1→4번) 단위벡터와 칸 간격을 구한다."""
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
    반환 {'col_deg': 열 방향 기울기(+ 면 3번 쪽이 높음), 'row_deg': 행 방향(+ 면 4번 쪽이 높음),
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


def nominal_tilt(heights, rows, cols, pitch_col, pitch_row):
    """블록 높이 배치(공칭값)로 만들어지는 기울기. 반환 (col_deg, row_deg, rms)."""
    xy = [(c * pitch_col, r * pitch_row) for r in range(rows) for c in range(cols)]
    a, b, _, rms = fit_plane(xy, heights)
    return math.degrees(math.atan(a)), math.degrees(math.atan(b)), rms


def tilt_candidates(bed, rack, heights, rows, cols, pitch_col, pitch_row,
                    allow_step=False, plane_tol=0.5):
    """지금 가진 블록(침대 + 보관대)으로 만들 수 있는 모든 배치와 그 기울기.

    3가지 높이 × 6칸 = 729가지를 전부 계산한다(순식간).
    allow_step=False 면 윗면이 한 평면에 놓이는 배치만 남긴다.
    """
    n = rows * cols
    have = Counter(h for h in list(bed) + list(rack) if h is not None)
    out = []
    for combo in itertools.product(heights, repeat=n):
        need = Counter(combo)
        if any(need[h] > have.get(h, 0) for h in need):
            continue                                    # 블록 개수가 모자람
        col_deg, row_deg, rms = nominal_tilt(combo, rows, cols, pitch_col, pitch_row)
        if not allow_step and rms > plane_tol:
            continue
        out.append({'heights': list(combo), 'col_deg': col_deg, 'row_deg': row_deg, 'rms': rms})
    return out


def achievable_angles(cands):
    """화면에 보여줄 "가능한 각도" 목록. {'col': [...], 'row': [...]} (다른 축은 0 인 것만)."""
    col = sorted({round(c['col_deg'], 1) + 0.0 for c in cands if abs(c['row_deg']) < 0.05})
    row = sorted({round(c['row_deg'], 1) + 0.0 for c in cands if abs(c['col_deg']) < 0.05})
    return {'col': col, 'row': row}


def plan_tilt(target_col, target_row, bed, rack, heights, rows, cols, pitch_col, pitch_row,
              allow_step=False, plane_tol=0.5):
    """목표 기울기에 가장 가까운 배치와, 거기까지 가는 블록 이동 순서를 구한다.

    고르는 순서: ① 목표 각도와 차이가 가장 작은 것 ② 윗면이 더 평평한 것
                ③ 옮기는 블록 수가 적은 것 ④ 평균 높이 변화가 작은 것
    반환 {'heights', 'col_deg', 'row_deg', 'rms', 'err', 'changed': 바뀌는 칸 수, 'moves'}
    """
    cands = tilt_candidates(bed, rack, heights, rows, cols, pitch_col, pitch_row,
                            allow_step, plane_tol)
    if not cands:
        raise PlanError('지금 가진 블록으로 만들 수 있는 배치가 없습니다.')
    cur_mean = statistics.mean([h for h in bed if h is not None] or [0.0])

    def key(c):
        err = math.hypot(c['col_deg'] - target_col, c['row_deg'] - target_row)
        changed = sum(1 for a, b in zip(bed, c['heights']) if a != b)
        return (round(err, 3), round(c['rms'], 3), changed,
                round(abs(statistics.mean(c['heights']) - cur_mean), 3))

    last_err = None
    for c in sorted(cands, key=key):
        try:
            moves = plan_moves(bed, rack, c['heights'])
        except PlanError as e:
            last_err = e                                # 이 배치는 옮길 방법이 없음 → 다음 후보
            continue
        k = key(c)
        return dict(c, err=k[0], changed=k[2], moves=moves)
    raise PlanError(str(last_err))


# ── 블록 이동 순서 ────────────────────────────────────────────────────
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
