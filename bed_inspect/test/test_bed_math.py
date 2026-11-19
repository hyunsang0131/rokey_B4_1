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


# ── 기울기 계획 ──
FULL_RACK = [25.0] * 6 + [35.0] * 6 + [30.0] * 6 + [None]


def plan(tc, tr, bed=None, rack=None, **kw):
    return bm.plan_tilt(tc, tr, bed or [30.0] * 6, FULL_RACK if rack is None else rack,
                        H, ROWS, COLS, PC, PR, **kw)


def test_achievable_angles():
    c = bm.tilt_candidates([30.0] * 6, FULL_RACK, H, ROWS, COLS, PC, PR)
    a = bm.achievable_angles(c)
    t1 = round(math.degrees(math.atan(5 / PC)), 1)
    r1 = round(math.degrees(math.atan(5 / PR)), 1)
    r2 = round(math.degrees(math.atan(10 / PR)), 1)
    assert a['col'] == [-t1, 0.0, t1]
    assert a['row'] == [-r2, -r1, 0.0, r1, r2]
    assert all(x['rms'] <= 0.5 for x in c)


def test_plan_tilt_picks_nearest():
    t1 = math.degrees(math.atan(5 / PC))            # 2.86도
    p = plan(3.0, 0.0)
    assert p['heights'] == [25, 30, 35, 25, 30, 35] and p['col_deg'] == pytest.approx(t1)
    p = plan(-10.0, 0.0)                            # 만들 수 없는 큰 각도 → 가장 가까운 것
    assert p['heights'] == [35, 30, 25, 35, 30, 25]
    p = plan(1.0, 0.0)                              # 0 에 더 가깝다 → 그대로
    assert p['heights'] == [30.0] * 6 and p['moves'] == []
    p = plan(1.6, 0.0)                              # 2.86 에 더 가깝다
    assert p['heights'] == [25, 30, 35, 25, 30, 35]
    p = plan(0.0, 4.0)                              # 행 방향: 2.39 / 4.76 중 4.76
    assert p['heights'] == [25, 25, 25, 35, 35, 35]
    p = plan(0.0, -2.0)                             # 행 방향 -2.39: 교체 수가 적은 조합
    assert p['row_deg'] == pytest.approx(-math.degrees(math.atan(5 / PR)))
    assert p['changed'] == 3


def test_plan_tilt_is_global_optimum():
    """고른 배치보다 목표에 더 가까운 배치가 하나도 없는지 전수 확인."""
    rnd = random.Random(1)
    for _ in range(60):
        tc, tr = rnd.uniform(-6, 6), rnd.choice([0.0, rnd.uniform(-6, 6)])
        p = plan(tc, tr)
        best = min(math.hypot(c['col_deg'] - tc, c['row_deg'] - tr)
                   for c in bm.tilt_candidates([30.0] * 6, FULL_RACK, H, ROWS, COLS, PC, PR))
        assert p['err'] == pytest.approx(best, abs=1e-3)


def test_plan_tilt_respects_stock():
    """35mm 블록이 1개뿐이면 열 방향 기울기(35mm 2개 필요)는 못 만든다 → 가능한 것 중 가장 가까운 것."""
    rack = [25.0, 25.0, 25.0, 35.0, None]
    p = plan(3.0, 0.0, rack=rack)
    assert p['heights'].count(35.0) <= 1
    assert p['heights'].count(25.0) <= 3
    assert abs(p['col_deg']) < 0.01                 # 열 방향은 불가 → 0도(평평)


def test_plan_tilt_step_option():
    p = plan(1.4, 0.0, allow_step=True)             # 계단 허용: 2.86 과 0 사이 각도를 만든다
    assert 0.5 < p['col_deg'] < 2.5 and p['rms'] > 0.5


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
    # 기울기용 재고(25/35mm 블록)로 만들 수 있는 각도: 행 방향 ±3.7도, 열 방향은 0도뿐
    cands = bm.tilt_candidates(cfg.BED_INIT, [30.0, 25.0, 25.0, 35.0, 35.0, None], cfg.BLOCK_HEIGHTS,
                               cfg.BED_ROWS, cfg.BED_COLS,
                               ax['pitch_col'], ax['pitch_row'])
    assert bm.achievable_angles(cands) == {'col': [0.0], 'row': [-3.7, 0.0, 3.7]}


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
