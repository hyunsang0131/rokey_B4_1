import pytest

from bed_inspect import string_v1_config as sc
from bed_inspect import string_v1_plan as plan


def test_v1_values_unchanged():
    assert (sc.SPIRAL_R_OUT, sc.SPIRAL_R_IN) == (30.0, 20.0)
    assert sc.TIGHTEN_TURN_DEG == 360.0 and sc.TIGHTEN_REPS == 1 and sc.LOOSEN_MAX_REPS == 1   # 11-22 한 바퀴 방식
    sp = plan.spiral_plan()
    assert sp['rev'] == pytest.approx(sc.SPIRAL_TURNS * 30 / 10)
    p = plan.fix_points([300.0, -20.0, 300.0])
    assert p['hole_touch'][2] == pytest.approx(sc.TABLE_Z + sc.SCREW_LEN + sc.INSERT_GAP)
    assert sc.PICK_XYZ == [sc.HOLE_XY[0], sc.HOLE_XY[1], 71.08]
    assert plan.fix_points(sc.PICK_XYZ)['pick'][2] == pytest.approx(71.08)
    assert plan.rot_chunks(180.0) == [90.0, 90.0]
    u = plan.unfix_points([300.0, -20.0, 300.0])
    assert len(u['unwind_halves']) == int(round(2 * sc.SPIRAL_TURNS))


def test_v1_spiral_points():
    import math
    sp = plan.spiral_plan()
    pts = plan.spiral_points(sp)
    assert len(pts) == int(360 * sc.SPIRAL_TURNS / 15)
    end = pts[-1]
    assert math.hypot(end[0] - sc.POST_XY[0], end[1] - sc.POST_XY[1]) == pytest.approx(sc.SPIRAL_R_IN)
    assert end[2] == pytest.approx(sp['end_z'])


def test_v1_unfix_pulls_beyond_hole():
    u = plan.unfix_points(sc.PICK_XYZ)
    d = plan.hole_dir()
    # 구멍보다 감는 나사 반대쪽으로 OVERSHOOT_MAX 더
    assert u['beyond'][0] == pytest.approx(sc.HOLE_XY[0] + d[0] * sc.OVERSHOOT_MAX)
    assert u['beyond'][1] == pytest.approx(sc.HOLE_XY[1] + d[1] * sc.OVERSHOOT_MAX)
    assert u['beyond'][1] < sc.HOLE_XY[1]
