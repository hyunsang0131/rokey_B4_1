import math

import pytest

from bed_inspect import string_v2_config as sc
from bed_inspect import string_v2_plan as plan


@pytest.fixture(autouse=True)
def _mode(monkeypatch):
    monkeypatch.setattr(sc, 'SPIRAL_MODE', 'inward')
    monkeypatch.setattr(sc, 'OUTWARD_Z', None)
    # 기하 시험은 작은 반지름으로 (300/260 은 베이스와 부딪혀 거부됨 → test_big_radius_rejected)
    monkeypatch.setattr(sc, 'SPIRAL_R_OUT', 30.0)
    monkeypatch.setattr(sc, 'SPIRAL_R_IN', 20.0)
    monkeypatch.setattr(sc, 'UNWIND_RADIUS', 25.0)


def dist_xy(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def test_inward_spiral_numbers():
    sp = plan.spiral_plan()
    assert sp['mode'] == 'inward'
    # 30 → 20 을 2바퀴에: 0 까지는 6바퀴, z 는 6/2 x 8 = 24 위가 목표
    assert sp['rev'] == pytest.approx(6.0)
    assert sp['target'][:2] == list(sc.POST_XY)
    assert sp['target'][2] - sp['start'][2] == pytest.approx(3 * sc.SPIRAL_RISE)
    assert dist_xy(sp['start'], sc.POST_XY) == pytest.approx(sc.SPIRAL_R_OUT)
    assert sp['stop_deg'] == pytest.approx(720.0)
    assert sp['end_z'] - sp['start'][2] == pytest.approx(sc.SPIRAL_RISE)
    # 시작점은 구멍 쪽(-y)
    assert sp['start'][1] == pytest.approx(sc.POST_XY[1] - sc.SPIRAL_R_OUT)


def test_wind_height_keeps_screw_off_table():
    z = plan.wind_z()
    assert z - sc.SCREW_LEN > sc.TABLE_Z


def test_outward_needs_height(monkeypatch):
    monkeypatch.setattr(sc, 'SPIRAL_MODE', 'outward')
    with pytest.raises(plan.StringPlanError):
        plan.spiral_plan()
    monkeypatch.setattr(sc, 'OUTWARD_Z', 400.0)
    sp = plan.spiral_plan()
    assert sp['start'][:3] == [sc.POST_XY[0], sc.POST_XY[1], 400.0]
    assert sp['rmax'] == sc.SPIRAL_R_OUT and sp['lmax'] == sc.SPIRAL_RISE


def test_fix_points_order():
    p = plan.fix_points([sc.HOLE_XY[0], sc.HOLE_XY[1], 71.08])
    assert p['pick_app'][2] > p['pick'][2]
    assert p['pick_up'][2] - sc.SCREW_LEN > sc.TABLE_Z          # 뽑았을 때 나사 끝이 탁자 위
    assert p['wind_up'][2] == sc.WIND2_Z
    assert p['tension_end'][2] == sc.WIND2_Z
    d = plan.tension_dir()
    assert d[1] == pytest.approx(1.0)                           # 실 고정 나사 → 구멍 방향 = +y
    assert p['hole_over'][:2] == p['hole_touch'][:2] == [sc.HOLE_XY[0], sc.HOLE_XY[1]]
    assert plan.hole_grip_z() < p['hole_touch'][2] < p['hole_over'][2]
    for v in p.values():
        assert v[3:] == sc.STR_ORI


def test_circle_through_and_limit():
    c = plan.circle_through((sc.POST_XY[0] + 100.0, sc.POST_XY[1]), 200.0, 2.0)
    assert c['r'] == pytest.approx(100.0) and c['angle'] == pytest.approx(720.0)
    assert c['via'][1] == pytest.approx(sc.POST_XY[1] + 100.0)    # 반시계: +x 에서 +y 로
    assert plan.max_circle_r() == pytest.approx(math.hypot(*sc.POST_XY) - sc.BASE_CLEAR)
    with pytest.raises(plan.StringPlanError, match='베이스'):
        plan.circle_through((sc.POST_XY[0], sc.POST_XY[1] + 260.0), 200.0, 2.0)


def test_fix_requires_pick():
    with pytest.raises(plan.StringPlanError):
        plan.fix_points(None)


def test_unwind_is_clockwise_two_turns():
    p = plan.unfix_points([300.0, -20.0, 300.0])
    arc = p['unwind']
    assert arc['angle'] == pytest.approx(720.0)
    a = [math.atan2(arc[n][1] - sc.POST_XY[1], arc[n][0] - sc.POST_XY[0]) for n in ('start', 'via', 'end')]
    d = (math.degrees(a[1] - a[0]) + 180) % 360 - 180
    assert d == pytest.approx(-90.0)          # 위에서 시계 (감은 반대)
    for n in ('start', 'via', 'end'):
        assert dist_xy(arc[n], sc.POST_XY) == pytest.approx(sc.UNWIND_RADIUS)
    assert p['hole_pull'][2] > p['hole_grip'][2]


def _spiral_xy(r0, r1, turns, start_deg, ccw, n=400):
    for i in range(n + 1):
        t = i / n
        ang = math.radians(start_deg + (1 if ccw else -1) * 360 * turns * t)
        r = r0 + (r1 - r0) * t
        yield (sc.POST_XY[0] + r * math.cos(ang), sc.POST_XY[1] + r * math.sin(ang))


def test_watch_stops_after_two_turns():
    w = plan.SpiralWatch(sc.POST_XY, 720.0, ccw=True, lead=8.0, min_gap=12.0, wrong_deg=30.0)
    res = None
    for xy in _spiral_xy(30, 0, 6, -90, True, n=3000):
        res = w.update(xy)
        if res:
            break
    assert res == 'stop'
    assert w.swept == pytest.approx(712.0, abs=1.5)
    assert w.r == pytest.approx(20.0 + 10 * 8 / 360, abs=0.3)   # 2바퀴 조금 전 → 반지름 20 조금 위


def test_watch_wrong_direction():
    w = plan.SpiralWatch(sc.POST_XY, 720.0, ccw=True, lead=8.0, min_gap=12.0, wrong_deg=30.0)
    res = None
    for xy in _spiral_xy(30, 0, 6, -90, False, n=3000):
        res = w.update(xy)
        if res:
            break
    assert res == 'wrong_dir'


def test_watch_too_close():
    w = plan.SpiralWatch(sc.POST_XY, 720.0, min_gap=12.0)
    assert w.update((sc.POST_XY[0] + 5, sc.POST_XY[1])) == 'too_close'


def test_big_radius_rejected(monkeypatch):
    # 감는 나사가 베이스에서 약 285mm → 반지름 300/260 원은 로봇 베이스를 지나감
    monkeypatch.setattr(sc, 'SPIRAL_R_OUT', 300.0)
    monkeypatch.setattr(sc, 'SPIRAL_R_IN', 260.0)
    with pytest.raises(plan.StringPlanError, match='베이스'):
        plan.spiral_plan()


def test_feasible_radius_limit(monkeypatch):
    monkeypatch.setattr(sc, 'SPIRAL_R_OUT', 80.0)
    monkeypatch.setattr(sc, 'SPIRAL_R_IN', 70.0)
    sp = plan.spiral_plan()
    assert sp['rev'] == pytest.approx(16.0)


def test_j6_start():
    assert plan.j6_start(0.0, 330.0) == pytest.approx(25.0) or plan.j6_start(0.0, 330.0) == 0.0
    assert plan.j6_start(100.0, 330.0) == pytest.approx(25.0)
    assert plan.j6_start(-100.0, 330.0) == -100.0
    assert plan.j6_start(-100.0, -330.0) == pytest.approx(-25.0)
    with pytest.raises(plan.StringPlanError):
        plan.j6_start(0.0, 800.0)


def test_rot_chunks():
    c = plan.rot_chunks(330.0, 110.0)
    assert len(c) == 3 and sum(c) == pytest.approx(330.0)
    assert all(abs(v) <= 110.0 + 1e-9 for v in c)
    assert plan.rot_chunks(-90.0, 110.0) == [-90.0]
