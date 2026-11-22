# 계산 모듈 검증:  python3 -m pytest test/test_bed_math.py -v
import itertools
import math
import random

import pytest

from bed_inspect import bed_math as bm

H = [25.0, 30.0, 35.0]
ROWS, COLS = 2, 3
PC, PR = 100.0, 120.0


def curve(k, z_surf, f_to=20.0, step=0.5, noise=0.0, toe=0.0, seed=0):
    """가짜 누르기 곡선. toe: 접촉 초반에 힘이 덜 나오는 구간(mm)."""
    rnd = random.Random(seed)
    out, z = [], z_surf + 5.0
    while True:
        pen = z_surf - z
        f = 0.0 if pen <= 0 else k * pen if pen > toe else k * pen * pen / toe if toe else k * pen
        f += rnd.gauss(0, noise)
        out.append((z, f))
        if f >= f_to:
            return out
        z -= step


# ── 강성 ──
def test_stiffness_exact():
    r = bm.fit_stiffness(curve(2.0, 100.0), f_min=5.0)
    assert r['k'] == pytest.approx(2.0, abs=1e-9)
    assert r['z_surf'] == pytest.approx(100.0, abs=1e-9)
    assert r['r2'] == pytest.approx(1.0)


@pytest.mark.parametrize('k', [0.8, 1.5, 2.0, 3.5])
def test_stiffness_with_noise(k):
    """힘 노이즈 0.3N 에서 강성 오차 5% 이내, 표면 높이 오차 0.5mm 이내 (100번 반복)."""
    errs, zerrs = [], []
    for seed in range(100):
        r = bm.fit_stiffness(curve(k, 100.0, noise=0.3, seed=seed), f_min=5.0)
        errs.append(abs(r['k'] - k) / k)
        zerrs.append(abs(r['z_surf'] - 100.0))
    assert sorted(errs)[94] < 0.05
    assert sorted(zerrs)[94] < 0.5


def test_stiffness_too_few_points():
    assert bm.fit_stiffness([(100, 0), (99.5, 6), (99, 7)], f_min=5.0) is None
    assert bm.fit_stiffness([], f_min=5.0) is None


# ── 판정 ──
def blocks(ks, hs=None):
    hs = hs or [30.0] * len(ks)
    return [{'slot': i, 'height': h, 'k': k, 'error': None} for i, (k, h) in enumerate(zip(ks, hs))]


def test_judge_median_same_height():
    b = bm.judge(blocks([2.0, 2.05, 1.95, 2.0, 1.2, 2.4]), {}, 0.15, 0.30)
    assert [x['grade'] for x in b] == ['ok', 'ok', 'ok', 'ok', 'fail', 'warn']
    assert b[4]['dev'] == pytest.approx(-0.4)
    assert b[0]['ref_src'] == 'median'


def test_judge_mixed_heights_no_false_alarm():
    """높이가 달라 강성이 다른 것은 불량이 아니다 (k × 높이 로 비교)."""
    hs = [25.0, 30.0, 35.0, 25.0, 30.0, 35.0]
    ks = [60.0 / h for h in hs]
    b = bm.judge(blocks(ks, hs), {}, 0.15, 0.30)
    assert all(x['grade'] == 'ok' for x in b)
    ks[2] *= 0.6
    b = bm.judge(blocks(ks, hs), {}, 0.15, 0.30)
    assert [x['grade'] for x in b] == ['ok', 'ok', 'fail', 'ok', 'ok', 'ok']


def test_judge_calib_catches_all_bad():
    """6개가 전부 나빠져도 기준값이 있으면 잡는다 (중앙값만으로는 못 잡음)."""
    ks = [1.2] * 6
    assert all(x['grade'] == 'ok' for x in bm.judge(blocks(ks), {}, 0.15, 0.30))
    assert all(x['grade'] == 'fail' for x in bm.judge(blocks(ks), {'30.0': 2.0}, 0.15, 0.30))


def test_judge_errors():
    b = blocks([2.0, None, None, 2.0, 2.0, 2.0])
    b[1]['error'] = 'no_contact'
    b[2]['error'] = 'bad_fit'
    b = bm.judge(b, {}, 0.15, 0.30)
    assert b[1]['grade'] == 'fail' and b[2]['grade'] == 'nodata'


def test_make_calib():
    c = bm.make_calib(blocks([2.0, 2.2, 2.4, 1.0, 1.1, 1.2], [30, 30, 30, 35, 35, 35]), {'25.0': 3.0})
    assert c == {'25.0': 3.0, '30.0': 2.2, '35.0': 1.1}


# ── 기울기 측정 ──
def grid_xy(x0=500.0, y0=-100.0, rot_deg=0.0):
    c, s = math.cos(math.radians(rot_deg)), math.sin(math.radians(rot_deg))
    return [(x0 + c * col * PC - s * row * PR, y0 + s * col * PC + c * row * PR)
            for row in range(ROWS) for col in range(COLS)]


@pytest.mark.parametrize('rot', [0.0, 37.0, 90.0, 180.0, -120.0])
def test_measured_tilt_any_bed_direction(rot):
    """침대가 로봇 기준으로 어느 방향으로 놓여 있어도 같은 기울기가 나와야 한다."""
    xy = grid_xy(rot_deg=rot)
    z = [80.0 + h for h in [25, 30, 35, 25, 30, 35]]          # 열 방향으로 5mm 씩 높아짐
    t = bm.measured_tilt(xy, z, ROWS, COLS)
    assert t['col_deg'] == pytest.approx(math.degrees(math.atan(5 / PC)), abs=1e-6)
    assert t['row_deg'] == pytest.approx(0.0, abs=1e-6)
    z = [80.0 + h for h in [25, 25, 25, 35, 35, 35]]          # 행 방향으로 10mm
    t = bm.measured_tilt(xy, z, ROWS, COLS)
    assert t['row_deg'] == pytest.approx(math.degrees(math.atan(10 / PR)), abs=1e-6)
    assert t['col_deg'] == pytest.approx(0.0, abs=1e-6)


def test_measured_tilt_missing_points():
    xy = grid_xy()
    z = [105, None, 115, 105, 110, None]
    t = bm.measured_tilt(xy, z, ROWS, COLS)
    assert t['n'] == 4 and t['col_deg'] == pytest.approx(math.degrees(math.atan(5 / PC)), abs=1e-6)
    assert bm.measured_tilt(xy, [1, None, None, None, None, 2], ROWS, COLS) is None


# ── 이동 순서 ──
def simulate(bed, rack, moves):
    """이동을 하나씩 따라가며 물리적으로 가능한지 확인."""
    bed, rack = list(bed), list(rack)
    for m in moves:
        (sk, si), (dk, di) = m['src'], m['dst']
        src = bed if sk == 'bed' else rack
        assert src[si] is not None and src[si] == m['h'], '없는 블록을 집음'
        if dk != 'discard':
            dst = bed if dk == 'bed' else rack
            assert dst[di] is None, '블록이 있는 자리에 놓음'
        bm.apply_move(bed, rack, m)
    return bed, rack


def test_plan_moves_all_configs_with_one_empty_rack_slot():
    """현재/목표 배치 조합을 무작위로 3000개 시험: 빈 자리 1개로 항상 성공하고 블록이 사라지지 않는다."""
    rnd = random.Random(7)
    combos = list(itertools.product(H, repeat=6))
    for _ in range(3000):
        bed, target = list(rnd.choice(combos)), list(rnd.choice(combos))
        rack = [25.0] * 6 + [30.0] * 6 + [35.0] * 6 + [None]
        rnd.shuffle(rack)
        moves = bm.plan_moves(bed, rack, target)
        end_bed, end_rack = simulate(bed, rack, moves)
        assert end_bed == target
        assert sorted(x for x in end_bed + end_rack if x) == sorted(x for x in bed + rack if x)
        changed = sum(a != b for a, b in zip(bed, target))
        assert len(moves) <= 2 * changed


def test_plan_moves_reverse_tilt_minimal_stock():
    """예비 블록 없이(보관대 빈 자리 1개) 기울기 방향 뒤집기: 침대 안에서 직접 옮긴다."""
    bed, target = [25, 30, 35, 25, 30, 35], [35, 30, 25, 35, 30, 25]
    moves = bm.plan_moves(bed, [None], target)
    assert simulate(bed, [None], moves)[0] == target
    assert len(moves) == 6


def test_plan_moves_errors():
    with pytest.raises(bm.PlanError):
        bm.plan_moves([30] * 6, [25, 25], [25, 25, 30, 30, 30, 30])      # 빈 자리 없음
    with pytest.raises(bm.PlanError):
        bm.plan_moves([30] * 6, [None], [25, 30, 30, 30, 30, 30])        # 25mm 없음


def test_plan_replace():
    bed = [25, 30, 35, 25, 30, 35]
    rack = [30, None, 35, 30]
    moves, skipped = bm.plan_replace(bed, rack, [1, 3, 4])
    assert skipped == [3]                                              # 25mm 예비 없음
    assert [bm.move_text(m) for m in moves] == [
        '침대 2번 (30mm) → 버림', '보관대 1번 (30mm) → 침대 2번',
        '침대 5번 (30mm) → 버림', '보관대 4번 (30mm) → 침대 5번']
    end_bed, end_rack = simulate(bed, rack, moves)
    assert end_bed == bed and end_rack == [None, None, 35, None]


# ── 커버 원호 ──
DEMO_S = [575.41, -16.09, 117.92, 135.46, -179.99, 45.19]
DEMO_V = [566.00, -58.40, 249.02, 133.59, -179.99, 43.43]
DEMO_E = [566.00, -291.64, 345.38, 89.57, -142.37, 3.34]


def _path(grip, above=80.0, retreat=60.0):
    return bm.cover_path(DEMO_S, DEMO_V, DEMO_E, grip, DEMO_S[3:], above, retreat)


def test_zyz_matrix():
    """두산 ZYZ: B=180 이면 툴 Z 가 바로 아래, 회전행렬은 직교."""
    assert bm.tool_z([0, 0, 0, 30.0, 180.0, 10.0]) == pytest.approx([0, 0, -1], abs=1e-12)
    assert bm.tool_z([0, 0, 0, 0.0, 90.0, 0.0]) == pytest.approx([1, 0, 0], abs=1e-12)
    assert bm.tool_z([0, 0, 0, 90.0, 90.0, 0.0]) == pytest.approx([0, 1, 0], abs=1e-12)
    r = bm.zyz_matrix(*DEMO_E[3:])
    assert r @ r.T == pytest.approx(bm.np.eye(3), abs=1e-12)
    assert bm.tilt_deg(DEMO_S) == pytest.approx(0.0, abs=0.02)
    assert bm.tilt_deg(DEMO_E) == pytest.approx(180 - 142.37, abs=1e-9)


def test_circle_3pt():
    c, r, ax = bm.circle_3pt([10, 0, 5], [0, 10, 5], [-10, 0, 5])
    assert c == pytest.approx([0, 0, 5]) and r == pytest.approx(10) and ax == pytest.approx([0, 0, 1])
    with pytest.raises(ValueError):
        bm.circle_3pt([0, 0, 0], [1, 1, 1], [2, 2, 2])


def _ang(p, path):
    """경첩에서 본 고리 각도 [deg] (0 = 닫힘)."""
    _, yc, zc = path['hinge']
    return math.degrees(math.atan2(p[2] - zc, p[1] - yc))


def _dist(p, path):
    _, yc, zc = path['hinge']
    return math.hypot(p[1] - yc, p[2] - zc)


def test_hinge_from_demo():
    """경첩: 닫힌 고리 높이, 1/3/5번 줄(y -179) 바깥. 시작점과 경유점이 경첩에서 같은 거리."""
    yc, zc = bm.hinge_from_demo(DEMO_S, DEMO_V)
    assert zc == DEMO_S[2] and -250 < yc < -230
    assert math.hypot(DEMO_S[1] - yc, DEMO_S[2] - zc) == pytest.approx(math.hypot(DEMO_V[1] - yc, DEMO_V[2] - zc))


def test_cover_path_reproduces_demo():
    """데모 시작점을 잡으면 데모 경유점/끝점의 y, z 가 그대로 나와야 한다 (x 는 시작점 x 로 고정)."""
    p = _path(DEMO_S[:3])
    assert p['via'][1:3] == pytest.approx(DEMO_V[1:3], abs=1e-3) and p['via'][3:] == DEMO_V[3:]
    assert p['end'][1:3] == pytest.approx(DEMO_E[1:3], abs=1e-3) and p['end'][3:] == DEMO_E[3:]
    assert p['radius'] == pytest.approx(p['demo_radius'])


def test_cover_path_grip_at_y_minus_50():
    """y -50 에서 잡기: x 는 끝까지 그대로, 원호 3점은 경첩 둘레 같은 원, 끝점은 데모와 같은 각도/비율."""
    g = [576.28, -50.0, 117.92]
    p = _path(g)
    for k in ('above', 'grip', 'mid', 'via', 'end', 'rest', 'close_end'):
        assert p[k][0] == g[0]                                          # x 유지
    c, r, ax = bm.circle_3pt(p['grip'][:3], p['mid'][:3], p['via'][:3])
    assert c == pytest.approx(p['hinge'], abs=0.05) and abs(ax[0]) == pytest.approx(1.0)   # 좌표 0.001mm 반올림 영향
    assert r == pytest.approx(p['radius'], abs=0.05) and p['radius'] == pytest.approx(g[1] - p['hinge'][1])
    assert _ang(p['mid'], p) == pytest.approx(p['via_deg'] / 2, abs=1e-3)
    assert _ang(p['via'], p) == pytest.approx(_ang(DEMO_V, _path(DEMO_S[:3])), abs=1e-3)
    assert _ang(p['end'], p) == pytest.approx(p['end_deg'], abs=1e-3) and 92 < p['end_deg'] < 110
    assert _dist(p['end'], p) / p['radius'] == pytest.approx(p['end_reach'], abs=1e-4)
    assert _ang(p['rest'], p) == pytest.approx(110.0, abs=1e-3)
    assert _dist(p['rest'], p) == pytest.approx(p['radius'], abs=1e-3)
    assert bm.tilt_deg(p['grip']) < 0.1 and bm.tilt_deg(p['via']) < 0.1     # 원호 동안 툴은 거의 수직
    # 직선 구간(경유 → 끝): x 는 그대로, y 는 줄고 z 는 늘어난다
    assert p['end'][1] < p['via'][1] and p['end'][2] > p['via'][2]



def test_cover_path_new_start_2026_11_19():
    """형님이 정한 시작점(그리퍼가 데모보다 툴 Z 둘레로 약 92도 돈 자세, z 27mm 낮음).
    경첩은 시작점 높이로 평행 이동, 자세는 데모의 변화만 적용 → 원호 동안 손목이 돌지 않고, 끝점에서 경첩 축(X) 둘레로만 기운다."""
    start = [575.85, -50.43, 91.21, 121.19, -175.44, 123.18]
    p = bm.cover_path(DEMO_S, DEMO_V, DEMO_E, start[:3], start[3:], 80.0, 60.0, line_segments=3, hinge_at_grip=True)
    assert p['grip'] == pytest.approx(start)
    assert p['hinge'][2] == pytest.approx(start[2]) and p['grip_angle'] == pytest.approx(0.0, abs=1e-9)
    ref = _path(start[:2] + [DEMO_S[2]])                    # 같은 y 에서 데모 높이로 잡은 경로를 z 만 내린 것과 같아야 한다
    dz = start[2] - DEMO_S[2]
    for k in ('mid', 'via', 'end', 'rest'):
        assert p[k][1] == pytest.approx(ref[k][1], abs=1e-3) and p[k][2] == pytest.approx(ref[k][2] + dz, abs=1e-3)
    g = bm.zyz_matrix(*start[3:])
    for k in ('mid', 'via'):                                # 원호 동안 자세는 거의 그대로
        assert bm.rot_angle_deg(bm.zyz_matrix(*p[k][3:]) @ g.T) < 0.2
    rel = bm.zyz_matrix(*p['end'][3:]) @ g.T                # 끝점: 데모와 같은 37.8도, 축은 베이스 X
    assert bm.rot_angle_deg(rel) == pytest.approx(bm.rot_angle_deg(bm.zyz_matrix(*DEMO_E[3:]) @ bm.zyz_matrix(*DEMO_S[3:]).T),
                                                  abs=0.01)
    ax = bm.np.array([rel[2, 1] - rel[1, 2], rel[0, 2] - rel[2, 0], rel[1, 0] - rel[0, 1]])
    assert abs(ax[0]) / bm.np.linalg.norm(ax) > 0.99
    angs = [bm.rot_angle_deg(bm.zyz_matrix(*q[3:]) @ g.T) for q in [p['via']] + p['line']]
    assert angs == sorted(angs)                             # 직선 구간 동안 고르게 기운다


def test_zyz_from_matrix_roundtrip():
    for ori in ([121.19, -175.44, 123.18], [89.57, -142.37, 3.34], [10.0, 45.0, -30.0], [0.0, 180.0, 0.0]):
        m = bm.zyz_matrix(*ori)
        assert bm.zyz_matrix(*bm.zyz_from_matrix(m)) == pytest.approx(m, abs=1e-9)

def test_cover_path_approach_points():
    p = _path([576.28, -50.0, 117.92], above=80.0, retreat=60.0)
    assert p['above'] == pytest.approx(p['grip'][:2] + [p['grip'][2] + 80.0] + p['grip'][3:])
    for a, b in (('end', 'end_app'), ('rest', 'rest_app')):
        d = bm.np.array(p[a][:3]) - bm.np.array(p[b][:3])
        assert bm.np.linalg.norm(d) == pytest.approx(60.0, abs=0.01)
        assert d / 60.0 == pytest.approx(bm.tool_z(p[a]), abs=1e-4)      # 툴 Z 축을 따라 빠진다
        assert p[b][2] > p[a][2]


def test_cover_path_errors():
    with pytest.raises(ValueError):                                      # 순서가 바뀜
        bm.cover_path(DEMO_S, DEMO_E, DEMO_V, DEMO_S[:3], DEMO_S[3:], 80, 60)
    with pytest.raises(ValueError):                                      # 경첩 바로 옆
        _path([576.28, -230.0, 117.92])


def test_cover_path_hinge_safety_checks():
    """경첩 보호 검사: 가운데에서 벗어난 x, 틀린 고리 높이, 받침보다 더 젖히는 끝점은 거부."""
    g = [576.28, -50.0, 117.92]
    _path(g)                                                             # 기본값은 통과
    with pytest.raises(ValueError, match='가운데'):                      # x 20mm 벗어남 → 한쪽 경첩에 힘이 몰림
        _path([g[0] + 20.0, g[1], g[2]])
    with pytest.raises(ValueError, match='고리 높이'):                   # z 를 50mm 잘못 넣음
        _path([g[0], g[1], g[2] + 50.0])
    with pytest.raises(ValueError, match='받침'):                       # 받침(100도)보다 끝점이 더 열림
        bm.cover_path(DEMO_S, DEMO_V, DEMO_E, g, DEMO_S[3:], 80, 60, rest_deg=100.0)
    with pytest.raises(ValueError, match='떨어집니다'):                 # 끝점이 수직을 안 넘음
        bm.cover_path(DEMO_S, DEMO_V, DEMO_E, g, DEMO_S[3:], 80, 60, min_open_deg=105.0)


def test_cover_path_close_margin():
    """닫기 끝점은 완전히 닫히기 close_margin 도 전: 같은 원 위, 잡는 점보다 위."""
    p = bm.cover_path(DEMO_S, DEMO_V, DEMO_E, [576.28, -50.0, 117.92], DEMO_S[3:], 80, 60, close_margin_deg=2.0)
    assert _dist(p['close_end'], p) == pytest.approx(p['radius'], abs=1e-2)
    assert _ang(p['close_end'], p) == pytest.approx(2.0, abs=1e-2)
    assert p['close_end'][2] > p['grip'][2]
    assert abs(p['grip_angle']) < 1.0 and abs(p['off_axial']) < 1.0


def test_cover_path_line_segments():
    """직선 구간을 나누면 중간점은 경첩 원 근처, 각도는 고르게, x 고정. 1 이면 [end] 하나."""
    g = [576.28, -50.0, 117.92]
    assert bm.cover_path(DEMO_S, DEMO_V, DEMO_E, g, DEMO_S[3:], 80, 60, line_segments=1)['line'] == \
        [_path(g)['end']]
    p = bm.cover_path(DEMO_S, DEMO_V, DEMO_E, g, DEMO_S[3:], 80, 60, line_segments=3)
    assert len(p['line']) == 3 and p['line'][-1] == p['end']
    angs = [p['via_deg']] + [_ang(q, p) for q in p['line']]
    steps = [b - a for a, b in zip(angs, angs[1:])]
    assert steps == pytest.approx([steps[0]] * 3, abs=1e-3)
    for q in p['line']:
        assert q[0] == g[0] and p['radius'] - 0.01 <= _dist(q, p) <= p['radius'] * p['end_reach'] + 0.01
# ── 실제 배치 (bed_config.BED_XY) ─────────────────────────────────────
def test_config_layout():
    """설정의 칸 좌표: 3행 x 2열, 행 간격 77.5mm(x), 열 간격 50mm(y), 번호 1 2 / 3 4 / 5 6."""
    from bed_inspect import bed_config as cfg
    ax = bm.bed_axes(cfg.BED_XY, cfg.BED_ROWS, cfg.BED_COLS)
    assert abs(ax['pitch_row'] - 77.5) < 1e-6 and abs(ax['pitch_col'] - 50.0) < 1e-6
    assert abs(ax['u_row'][0] - 1.0) < 1e-9 and abs(ax['u_col'][1] - 1.0) < 1e-9
    # 1→3→5 쪽으로 5mm 씩 높아지는 표면 → 행 방향 +3.69도, 열 방향 0도
    z = [100.0 + 5.0 * (i // cfg.BED_COLS) for i in range(6)]
    t = bm.measured_tilt(cfg.BED_XY, z, cfg.BED_ROWS, cfg.BED_COLS)
    assert abs(t['row_deg'] - math.degrees(math.atan(5 / 77.5))) < 1e-6 and abs(t['col_deg']) < 1e-6



# ── 자동 보수 ─────────────────────────────────────────────────────────
def _blk(slot, k, error=None, z0=100.0, z_surf=None):
    if z_surf is None and k is not None and z0 is not None:
        z_surf = z0 + 3.0 / k                       # 3N 에 닿았다고 보고 표면 높이를 맞춘다
    return {'slot': slot, 'height': 30.0, 'k': k, 'error': error, 'z0': z0, 'z_surf': z_surf}


def test_find_empties_by_contact_height():
    """빈칸 = 닿지 않음 또는 접촉 높이가 낮음. 끝까지 들어간 아주 무른 큐브(접촉 높이 정상)는 빈칸이 아니다."""
    bl = [_blk(1, 8.0, z0=100.0), _blk(2, 2.0, z0=98.5), _blk(3, None, 'no_contact', z0=None),
          _blk(4, None, 'too_deep', z0=99.0), _blk(5, 8.0, z0=99.8), _blk(6, 3.0, z0=85.0),
          ]
    empties = bm.find_empties(bl, 8.0)
    assert empties == [2, 5]
    assert [b['grade'] for b in bl] == [None, None, 'empty', None, None, 'empty']
    assert bl[3]['too_soft'] and not bl[1]['too_soft']
    bl.append(_blk(7, None, 'bad_fit'))
    bm.find_empties(bl, 8.0)
    assert bl[6]['grade'] == 'nodata'


def test_judge_auto_flags_odd_one():
    """40A 4칸 사이의 10A 1칸만 '맞지 않음'. 10A 옆의 40A 는 정상(중앙값 기준이라 끌려가지 않음)."""
    H, S = 8.0, 2.0
    bl = [_blk(1, H), _blk(2, H * 1.05), _blk(3, None, 'no_contact', z0=None), _blk(4, S), _blk(5, H), _blk(6, H * 0.95)]
    bm.find_empties(bl, 8.0)
    assert bm.judge_auto(bl, 0.5) == [3]
    assert [b['grade'] for b in bl] == ['ok', 'ok', 'empty', 'mismatch', 'ok', 'ok']
    assert bl[0]['ref_src'] == 'others' and abs(bl[3]['dev'] + 0.75) < 1e-9
    # 끝까지 들어간 큐브는 강성이 없어도 맞지 않음
    bl = [_blk(1, H), _blk(2, H), _blk(3, None, 'too_deep'), _blk(4, H), _blk(5, H), _blk(6, H)]
    bm.find_empties(bl, 8.0)
    assert bm.judge_auto(bl, 0.5) == [2]


def test_split_and_profile():
    H, S = 8.0, 2.0
    thr = bm.split_classes([H, H * 1.1, S, S * 0.9, H * 0.95, S * 1.2], 2.0)
    assert S * 1.2 < thr < H * 0.95
    assert bm.split_classes([H, H * 1.3, H * 0.8], 2.0) is None             # 한 종류뿐
    c = bm.class_centers([(H, False), (H, False), (S, False), (S, False)], thr)
    assert abs(c['hard'] - H) < 1e-9 and abs(c['soft'] - S) < 1e-9
    bl = [_blk(1, H), _blk(2, S), _blk(3, None, 'no_contact', z0=None), _blk(4, H), _blk(5, None, 'too_deep'), _blk(6, H)]
    bm.find_empties(bl, 8.0)
    prof = ['hard', 'hard', 'soft', 'soft', 'soft', 'hard']
    assert bm.judge_profile(bl, prof, thr, c) == [1, 3]
    assert [b['grade'] for b in bl] == ['ok', 'mismatch', 'empty', 'mismatch', 'ok', 'ok']
    assert bl[1]['cls'] == 'soft' and bl[1]['want'] == 'hard' and bl[4]['cls'] == 'soft'


def test_find_tall_and_empty_with_tall_cubes():
    """키 큰 큐브(표면 +10mm)는 '높음', 빈칸은 여전히 빈칸. 키 큰 큐브가 있어도 보통 10A 를 빈칸으로 오판하지 않는다."""
    bl = [_blk(1, 8.0, z0=100.0), _blk(2, 0.75, z0=106.0), _blk(3, None, 'no_contact', z0=None),
          _blk(4, 2.0, z0=98.5), _blk(5, 8.0, z0=99.6), _blk(6, None, 'too_deep', z0=107.0)]
    assert bm.find_empties(bl, 8.0, 5.0) == [2]
    assert [b['tall'] for b in bl] == [False, True, False, False, False, True]
    assert bl[5]['grade'] is None                                   # 끝까지 들어간 키 큰 큐브도 측정 실패가 아님
    # 자동 보수: 키 큰 큐브는 주변과 맞지 않음
    assert bm.judge_auto(bl, 0.5) == [1, 3, 5]                      # 2번(높음), 4번(무름), 6번(높음)


def test_profile_three_kinds():
    """고객 맞춤: 단단하게/부드럽게/높게. 원하는 종류의 큐브를 고르고, 모자라면 어느 칸인지 알린다."""
    H, S, T = 8.0, 2.0, 1.5
    thr = bm.split_classes([H, S, H, S], 2.0)
    bl = [_blk(1, H), _blk(2, H), _blk(3, None, 'no_contact', z0=None), _blk(4, S), _blk(5, H), _blk(6, H)]
    bm.find_empties(bl, 8.0, 5.0)
    c = bm.class_centers([(H, False), (S, False), (T, True)], thr)
    prof = ['tall', 'tall', 'soft', 'soft', 'hard', 'hard']
    assert bm.judge_profile(bl, prof, thr, c) == [0, 1]
    cands = {0: ('tall', T), 1: ('hard', H), 2: ('soft', S), 3: ('hard', H), 4: ('tall', T * 1.1)}
    assert bm.plan_profile([0, 1, 2], prof, cands, c) == [(0, 0), (1, 4), (2, 2)]
    with pytest.raises(bm.PlanError, match='3번'):
        bm.plan_profile([0, 1, 2], prof, {0: ('tall', T), 4: ('tall', T), 1: ('hard', H)}, c)


def test_plan_fill_slot_target():
    """사용자 지정: 칸마다 원하는 강성에 맞춘다 (이웃이 단단해도 부드러운 큐브)."""
    H, S = 8.0, 2.0
    p = bm.plan_fill([2, 3], {0: H, 1: H, 4: H, 5: H}, {0: S, 1: H, 2: S, 3: H}, 3, 2, slot_target={2: S, 3: H})
    assert dict(p['assign']) == {2: 0, 3: 1}


def test_plan_repair_moves():
    """빈칸 먼저 채우고, 맞지 않는 모듈은 보관대 빈 자리로 되돌린 뒤 고른 큐브를 넣는다."""
    bed = [30.0, 30.0, None, 30.0, 30.0, 30.0]
    rack = [30.0, 30.0, 30.0, 30.0, 30.0, None]
    moves = bm.plan_repair_moves(bed, rack, [(3, 1), (2, 3)], [3])
    assert [bm.move_text(m) for m in moves] == [
        '보관대 4번 (30mm) → 침대 3번',
        '침대 4번 (30mm) → 보관대 4번', '보관대 2번 (30mm) → 침대 4번']     # 방금 비운 자리로 되돌림
    end_bed, end_rack = simulate(bed, rack, moves)
    assert end_bed == [30.0] * 6 and end_rack == [30.0, None, 30.0, 30.0, 30.0, None]
    # 보관대에 빈 자리가 없으면 버린다
    moves = bm.plan_repair_moves([30.0] * 6, [30.0, 30.0], [(0, 1)], [0])
    assert moves[0]['dst'] == ('discard', 0)


def test_plan_fill_uniform_and_mixed():
    H, S = 8.0, 2.0
    # 4칸 모두 단단 → 단단한 큐브 2개
    p = bm.plan_fill([2, 3], {0: H, 1: H, 4: H, 5: H}, {0: S, 1: H, 2: S, 3: H, 4: S}, 3, 2)
    assert sorted(j for _, j in p['assign']) == [1, 3]
    # 섞임(1H 2S 5H 6S) → 3번 칸(이웃 1, 5 = H)에 H, 4번 칸(이웃 2, 6 = S)에 S
    p = bm.plan_fill([2, 3], {0: H, 1: S, 4: H, 5: S}, {0: S, 1: H, 2: S, 3: H}, 3, 2)
    assert dict(p['assign']) == {2: 1, 3: 0} and p['err'] < 1e-9
    # 트레이 환산이 1.25배 틀려도(트레이 값이 더 높게 나옴) 단단한 칸 옆에는 단단한 큐브 (가짜 로봇 시험에서 찾은 문제)
    p = bm.plan_fill([2, 3], {0: H, 1: H, 4: H, 5: H}, {0: S * 1.25, 1: H * 1.25, 2: S * 1.25, 3: H * 1.25}, 3, 2)
    assert sorted(j for _, j in p['assign']) == [1, 3]


def test_plan_fill_not_enough_cubes():
    with pytest.raises(bm.PlanError):
        bm.plan_fill([2, 3], {0: 8.0}, {0: 8.0}, 3, 2)


# ── 주문: 침대 안 큐브 자리 바꾸기 (2026-11-20) ──
def _cell(cls, k=None):
    return {'cls': cls, 'h': 20.0, 'k': k}


def _run(bed, tray, moves):
    from bed_inspect import bed_math as bm
    bed, tray = list(bed), list(tray)
    for m in moves:
        (sk, si), (dk, di) = m['src'], m['dst']
        src = bed if sk == 'bed' else tray
        c, src[si] = src[si], None
        (bed if dk == 'bed' else tray)[di] = c
    return bed, tray


def test_order_swaps_within_bed_without_tray_soft():
    """침대에 부드러운 큐브 2개가 있고 자리만 바꾸면 될 때: 트레이에 부드러운 큐브가 없어도 계획이 선다."""
    from bed_inspect import bed_math as bm
    bed = [_cell('soft'), _cell('soft'), _cell('hard'), _cell('hard'), _cell('hard'), _cell('tall')]
    tray = [_cell('hard', 12), _cell('hard', 12), None, _cell('tall'), None, _cell('hard', 11)]
    profile = ['hard', 'hard', 'soft', 'soft', 'hard', 'tall']
    moves, choice = bm.plan_profile_moves(bed, tray, profile)
    b2, _ = _run(bed, tray, moves)
    assert [c['cls'] for c in b2] == profile
    # 트레이의 원래 큐브 중 부드러운 것은 없으므로, 침대의 부드러운 큐브 2개가 (잠깐 트레이를 거쳐서라도) 자리를 바꿨다
    assert len(moves) <= 6


def test_order_shortage_counts_bed_and_tray():
    from bed_inspect import bed_math as bm
    bed = [_cell('hard')] * 6
    tray = [_cell('hard'), None, None, None, None, None]
    with pytest.raises(bm.PlanError, match='모자랍니다'):
        bm.plan_profile_moves(bed, tray, ['soft'] + ['hard'] * 5)


def test_order_keeps_unknown_slot():
    from bed_inspect import bed_math as bm
    bed = [{'cls': None, 'h': 20.0, 'k': None, 'keep': True}] + [_cell('hard')] * 5
    tray = [_cell('soft', 7), None, None, None, None, None]
    moves, _ = bm.plan_profile_moves(bed, tray, ['soft', 'hard', 'hard', 'hard', 'hard', 'soft'])
    assert all(m['src'] != ('bed', 0) and m['dst'] != ('bed', 0) for m in moves)


def test_order_uses_bed_empty_first_then_tray():
    """침대에 빈칸이 있으면: 침대 안에서 자리가 틀린 큐브를 빈칸으로 먼저 옮기고, 모자란 것만 트레이에서 (트레이 빈자리 없어도 됨)."""
    from bed_inspect import bed_math as bm
    bed = [_cell('soft'), None, _cell('hard'), _cell('hard'), None, _cell('hard')]
    tray = [_cell('hard', 12), _cell('soft', 7), _cell('hard', 12), _cell('hard', 12), _cell('tall'), _cell('tall')]  # 트레이 꽉 참
    profile = ['hard', 'soft', 'hard', 'hard', 'hard', 'hard']
    moves, choice = bm.plan_profile_moves(bed, tray, profile)
    assert moves[0] == {'src': ('bed', 0), 'dst': ('bed', 1), 'h': 20.0}    # 침대 1번 소프트 → 빈칸 2번 먼저
    assert all(m['dst'][0] == 'bed' for m in moves)                          # 트레이 빈자리는 안 씀
    b2, _ = _run(bed, tray, moves)
    assert [c['cls'] for c in b2] == profile


def test_find_empties_tall_with_external_reference():
    """트레이에 큐브가 하나뿐이면 자기 자신이 기준이 돼 높은 큐브를 못 알아봄 → 침대에서 옮긴 기준(z_ref)을 쓰면 알아봄 (11-22)."""
    from bed_inspect import bed_math as bm
    one = [{'k': None, 'z0': None, 'z_surf': None, 'error': 'no_contact'} for _ in range(5)]
    one.append({'k': 5.4, 'z0': 110.0, 'z_surf': 110.0, 'error': None})
    bm.find_empties([dict(b) for b in one], 8.0, 4.5)
    t = [dict(b) for b in one]
    bm.find_empties(t, 8.0, 4.5)
    assert not t[5]['tall']                                   # 기준 없으면 못 알아봄
    t = [dict(b) for b in one]
    bm.find_empties(t, 8.0, 4.5, z_ref=100.6)
    assert t[5]['tall']
