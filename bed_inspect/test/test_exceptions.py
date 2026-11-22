"""예외 상황 (2026-11-20): 충돌 반대 방향, 충돌 시 작업 다시 하기, 재고 부족 판단."""
import types

import pytest

from bed_inspect import bed_config as cfg
from bed_inspect.bed_robot import BedCollision, BedError


def test_retreat_opposite_motion():
    c = BedCollision('이동', pos=[100, 50, 200, 0, 180, 0], start=[100, 0, 200, 0, 180, 0], target=None, state=5)
    assert c.retreat_dir() == pytest.approx([0.0, -1.0, 0.0])        # +y 로 가다 충돌 → -y 로 물러남


def test_retreat_uses_target_when_barely_moved():
    c = BedCollision('하강', pos=[0, 0, 100.5, 0, 0, 0], start=[0, 0, 101, 0, 0, 0], target=[0, 0, 50], state=5)
    assert c.retreat_dir() == pytest.approx([0.0, 0.0, 1.0])         # 아래로 가려다 충돌 → 위로


def test_retreat_default_up():
    assert BedCollision('?', None, None, None, 5).retreat_dir() == [0.0, 0.0, 1.0]


def _proc():
    import importlib
    bp = importlib.import_module('bed_inspect.bed_process')
    p = types.SimpleNamespace(calls=[])
    p._on_collision = lambda e, kind, n: p.calls.append((kind, n))
    p._collision_safe = types.MethodType(bp.BedProcess._collision_safe, p)
    p._can_restock = types.MethodType(bp.BedProcess._can_restock, p)
    return p


def test_collision_retries_task():
    p = _proc()
    tries = []

    def task(x):
        tries.append(x)
        if len(tries) < 3:
            raise BedCollision('누르기', [0, 0, 0], [0, 0, 1], None, 5)
        return 'ok'
    assert p._collision_safe(task, 'press')(7) == 'ok'
    assert tries == [7, 7, 7] and p.calls == [('press', 1), ('press', 2)]


def test_collision_gives_up_after_max():
    p = _proc()

    def task():
        raise BedCollision('옮기기', None, None, None, 5)
    with pytest.raises(BedError, match='반복'):
        p._collision_safe(task, 'block')()
    assert len(p.calls) == cfg.COLLISION_RETRY_MAX


def test_can_restock_only_for_shortage():
    p = _proc()
    p._restock_n = 0
    p._order_mode = False                    # 기본 공정: 재고 부족 예외 없음 (11-22 형님)
    assert not p._can_restock(Exception('트레이에 기준에 맞는 큐브가 모자랍니다: 1번'))
    p._order_mode = True                     # 주문 모드에서만
    assert p._can_restock(Exception('트레이에 기준에 맞는 큐브가 모자랍니다: 1번'))
    assert not p._can_restock(Exception('트레이에 빈 자리가 없습니다'))
    p._restock_n = cfg.RESTOCK_MAX
    assert not p._can_restock(Exception('모자랍니다'))


def test_wait_still_waits_for_shaking_to_stop(monkeypatch):
    import itertools
    from bed_inspect import bed_robot
    seq = itertools.chain([[0, 0, 100 + (0.5 if i % 2 else 0)] for i in range(6)], itertools.repeat([0, 0, 100.0]))
    r = bed_robot.BedRobot.__new__(bed_robot.BedRobot)
    r.log = lambda m: None
    r.cur_xyz = lambda: list(next(seq))
    monkeypatch.setattr(bed_robot.time, 'sleep', lambda s: None)
    clock = itertools.count(0, 0.1)
    monkeypatch.setattr(bed_robot.time, 'monotonic', lambda: next(clock))
    waited = r.wait_still('시험', tol=0.1, hold=1.0, timeout=10)
    assert waited >= 1.5                    # 흔들리는 동안(0.6s)은 안정으로 보지 않는다


def test_restock_unfixes_string_first_and_only_once(monkeypatch):
    """예외 상황 2 (11-22): 재고 부족이면 cube_search 전에 실 고정해제, 두 번째 보충 때는 다시 안 함."""
    import importlib
    import sys
    bp = importlib.import_module('bed_inspect.bed_process')
    calls = []
    p = types.SimpleNamespace(_restock_n=0, _string_open=False, map={'tray': None}, log=lambda m: None,
                              step=lambda *a, **k: None, publish=lambda **k: None,
                              _home_raw=lambda: calls.append('home'))
    p._string_op = lambda a: (calls.append(a), setattr(p, '_string_open', a == 'unfix'))

    class FakeJob:
        def __init__(self, io, c, log, live=None):
            self.log = log

        def pick_place(self, cell=None):
            calls.append('search')
            self.log('완료: 트레이에 넣음')
    fake = types.SimpleNamespace(input=None, load_cfg=lambda: {}, SearchError=RuntimeError,
                                 RobotIO=lambda *a: types.SimpleNamespace(prepare=lambda: None), Job=FakeJob)
    monkeypatch.setitem(sys.modules, 'pjt_test1', types.SimpleNamespace(cube_search=fake))
    monkeypatch.setitem(sys.modules, 'pjt_test1.cube_search', fake)
    p.robot = types.SimpleNamespace(node=None)
    monkeypatch.setattr(cfg, 'STRING_RESTOCK', True)
    bp.BedProcess._restock(p, '모자랍니다')
    assert calls.index('unfix') < calls.index('search')
    assert p._string_open and p.map['tray'] is None
    calls.clear()
    bp.BedProcess._restock(p, '모자랍니다')
    assert 'unfix' not in calls and 'search' in calls


def test_restock_uses_tray_map_empty_cell(monkeypatch):
    """예외 상황 2 (11-22): 트레이를 다시 확인하지 않고 맵의 빈칸에 넣고, 그 칸만 다시 재도록 표시."""
    import importlib
    import sys
    bp = importlib.import_module('bed_inspect.bed_process')
    got = {}
    full = {'k': 9.0, 'empty': False, 'tall': False, 'too_soft': False, 'src': 'm', 'z': 100.0}
    empty = dict(full, k=None, empty=True)
    p = types.SimpleNamespace(_restock_n=0, _string_open=True, log=lambda m: None, step=lambda *a, **k: None,
                              publish=lambda **k: None, _home_raw=lambda: None,
                              map={'tray': [dict(full), dict(empty), dict(full), dict(empty), dict(full), dict(full)]})

    class FakeJob:
        def __init__(self, io, c, log, live=None):
            self.log = log

        def pick_place(self, cell=None):
            got['cell'] = cell
            self.log('완료')
    fake = types.SimpleNamespace(input=None, load_cfg=lambda: {}, SearchError=RuntimeError,
                                 RobotIO=lambda *a: types.SimpleNamespace(prepare=lambda: None), Job=FakeJob)
    monkeypatch.setitem(sys.modules, 'pjt_test1', types.SimpleNamespace(cube_search=fake))
    monkeypatch.setitem(sys.modules, 'pjt_test1.cube_search', fake)
    p.robot = types.SimpleNamespace(node=None)
    bp.BedProcess._restock(p, '모자랍니다')
    assert got['cell'] == 2
    assert p.map['tray'][1] is None and p.map['tray'][3]['empty']


def test_bridge_map_tolerates_unmeasured_cell():
    from bed_inspect.cushion_bridge import map_cells
    c = {'k': 9.0, 'empty': False, 'tall': False}
    bed, tray = map_cells({'bed': [c] * 6, 'tray': [c, None, c, c, c, c]})
    assert tray[1]['type'] == '?'


def test_restock_checks_tray_space_before_unfix(monkeypatch):
    """트레이 맵에 빈칸이 없으면 실을 풀기 전에 멈춘다."""
    import importlib
    bp = importlib.import_module('bed_inspect.bed_process')
    calls = []
    full = {'k': 9.0, 'empty': False, 'tall': False, 'too_soft': False, 'src': 'm', 'z': 100.0}
    p = types.SimpleNamespace(_restock_n=0, _string_open=False, log=lambda m: None, step=lambda *a, **k: None,
                              publish=lambda **k: None, _home_raw=lambda: None, map={'tray': [dict(full)] * 6})
    p._string_op = lambda a: calls.append(a)
    monkeypatch.setattr(cfg, 'STRING_RESTOCK', True)
    with pytest.raises(BedError, match='빈칸'):
        bp.BedProcess._restock(p, '모자랍니다')
    assert calls == []
