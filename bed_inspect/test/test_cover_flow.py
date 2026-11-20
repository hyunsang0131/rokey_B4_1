# 커버 열기/닫기 동작 검증 (ROS 없이):  python3 -m pytest test/test_cover_flow.py -v
#
# 실제 코드 경로를 그대로 쓴다:  BedTasks.cover_open/close → BedRobot → (가짜 DSR_ROBOT2) → bed_sim 의 서비스 처리 함수
# rclpy / dsr_msgs2 / onrobot 메시지만 가짜로 바꿔 끼워서, bed_sim 의 가짜 세계(경첩, 고리, 위반 기록)로 확인한다.

import importlib
import sys
import threading
import types

import pytest

from bed_inspect import bed_config as cfg


class _Obj:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _stub_modules(monkeypatch):
    class Node:
        def __init__(self, *a, **k):
            pass

        def create_service(self, *a, **k):
            return None

        create_publisher = create_subscription = create_timer = create_service

        def get_logger(self):
            return _Obj(info=lambda *a: None, error=lambda *a: None, warn=lambda *a: None)

    mods = {
        'rclpy': types.ModuleType('rclpy'),
        'rclpy.node': types.ModuleType('rclpy.node'),
        'rclpy.callback_groups': types.ModuleType('rclpy.callback_groups'),
        'rclpy.executors': types.ModuleType('rclpy.executors'),
        'rclpy.utilities': types.ModuleType('rclpy.utilities'),
        'std_msgs': types.ModuleType('std_msgs'),
        'std_msgs.msg': types.ModuleType('std_msgs.msg'),
        'dsr_msgs2': types.ModuleType('dsr_msgs2'),
        'dsr_msgs2.srv': types.ModuleType('dsr_msgs2.srv'),
        'onrobot_rg_msgs': types.ModuleType('onrobot_rg_msgs'),
        'onrobot_rg_msgs.srv': types.ModuleType('onrobot_rg_msgs.srv'),
        'DR_common2': types.ModuleType('DR_common2'),
    }
    mods['rclpy.node'].Node = Node
    mods['rclpy.callback_groups'].ReentrantCallbackGroup = object
    mods['rclpy.executors'].MultiThreadedExecutor = object
    mods['rclpy.utilities'].remove_ros_args = lambda a: a
    mods['std_msgs.msg'].Float64MultiArray = _Obj
    mods['std_msgs.msg'].String = _Obj
    mods['dsr_msgs2'].srv = mods['dsr_msgs2.srv']
    mods['dsr_msgs2.srv'].__getattr__ = lambda name: object
    mods['onrobot_rg_msgs.srv'].SetCommand = object
    mods['DR_common2'].posj = list
    mods['DR_common2'].posx = list
    for k, v in mods.items():
        monkeypatch.setitem(sys.modules, k, v)
    monkeypatch.delitem(sys.modules, 'bed_inspect.bed_sim', raising=False)
    return importlib.import_module('bed_inspect.bed_sim')


class FakeDR:
    """DSR_ROBOT2 와 같은 이름/인자의 함수 → bed_sim 의 서비스 처리 함수로 연결."""
    DR_BASE, DR_TOOL = 0, 1
    DR_MV_MOD_ABS, DR_MV_MOD_REL = 0, 1
    DR_STATE_IDLE, DR_STATE_INIT, DR_STATE_BUSY = 0, 1, 2

    def __init__(self, sim):
        self.sim = sim
        self.calls = []

    def _ret(self, res):
        return 0 if res.success else -1

    def get_robot_state(self):
        return 2 if self.sim.moving else self.sim.robot_state

    def get_last_alarm(self):
        return -1

    def movej(self, pos, vel=None, acc=None, mod=0):
        self.calls.append(('movej', list(pos)))
        return self._ret(self.sim.s_movej(_Obj(pos=list(pos), vel=vel, mode=mod), _Obj()))

    def movel(self, pos, vel=None, acc=None, ref=0, mod=0):
        self.calls.append(('movel', list(pos)))
        return self._ret(self.sim.s_movel(_Obj(pos=list(pos), vel=[vel, vel], ref=ref, mode=mod), _Obj()))

    def amovel(self, pos, vel=None, acc=None, ref=0, mod=0):
        self.calls.append(('amovel', list(pos)))
        req = _Obj(pos=list(pos), vel=[vel, vel], ref=ref, mode=mod, sync_type=1)
        return self._ret(self.sim.s_movel(req, _Obj()))

    def _movec(self, name, pos1, pos2, vel, ref, mod, sync):
        self.calls.append((name, list(pos1), list(pos2)))
        req = _Obj(pos=[_Obj(data=list(pos1)), _Obj(data=list(pos2))], vel=[vel, vel], ref=ref, mode=mod,
                   sync_type=sync)
        return self._ret(self.sim.s_movec(req, _Obj()))

    def movec(self, pos1, pos2, vel=None, acc=None, ref=0, mod=0):
        return self._movec('movec', pos1, pos2, vel, ref, mod, 0)

    def amovec(self, pos1, pos2, vel=None, acc=None, ref=0, mod=0):
        return self._movec('amovec', pos1, pos2, vel, ref, mod, 1)

    def check_motion(self):
        return self.sim.s_check_motion(None, _Obj()).status

    def task_compliance_ctrl(self, stx, time=0):
        assert len(stx) == 6
        self.calls.append(('comp_on',))
        return self._ret(self.sim.s_compliance_on(None, _Obj()))

    def release_compliance_ctrl(self):
        self.calls.append(('comp_off',))
        return self._ret(self.sim.s_compliance_off(None, _Obj()))

    def get_tool_force(self, ref=0):
        return self.sim.s_force(None, _Obj()).tool_force

    def get_current_posx(self, ref=0):
        return list(self.sim.pose), 2

    def get_current_posj(self):
        return self.sim.s_posj(None, _Obj()).pos

    def set_digital_output(self, idx, val):
        self.calls.append(('do', idx, val))
        self.sim.s_do(_Obj(index=idx, value=val), _Obj())


def _make_world(monkeypatch, speed, ring_slack=None):
    if cfg.COVER_MODE != 'one_arc':                 # 교시한 빠져나오는 길은 지금 방식(one_arc) 좌표에 맞춘 것
        monkeypatch.setattr(cfg, 'COVER_EXIT_POSJ', None)
    sim_mod = _stub_modules(monkeypatch)
    monkeypatch.setattr(cfg, 'GRIPPER_MODE', 'io')
    monkeypatch.setattr(cfg, 'GRIP_WAIT', 0.0)
    monkeypatch.setattr(cfg, 'COVER_POLL', 0.005)
    opt = _Obj(seed=1, fault='', fault_factor=0.5, speed=speed, noise=0.3, sign=1.0,
               bed=list(cfg.BED_INIT), rack=list(cfg.RACK_INIT),
               bed_kind=sim_mod.parse_kind(None, len(cfg.P_BED)), rack_kind=sim_mod.parse_kind(None, len(cfg.P_RACK)),
               ring_slack=sim_mod.RING_SLACK if ring_slack is None else ring_slack)
    sim = sim_mod.BedSim(opt)
    from bed_inspect.bed_robot import BedRobot
    from bed_inspect.bed_tasks import BedTasks
    points = {n: {'name': n, 'posj': sim_mod.ikin(p), 'posx': p} for n, p in sim_mod.POSES.items()}
    dr = FakeDR(sim)
    stop = threading.Event()
    robot = BedRobot(dr, None, lambda t: None, stop)
    robot.halt = lambda: (dr.calls.append(('halt',)), sim.s_stop(None, _Obj()))
    logs = []
    tasks = BedTasks(robot, points, logs.append)
    return _Obj(sim=sim, mod=sim_mod, dr=dr, tasks=tasks, stop=stop, logs=logs)


@pytest.fixture
def world(monkeypatch):
    return _make_world(monkeypatch, 1e3)


@pytest.fixture
def slow_world(monkeypatch):
    """원호가 실제 시간으로 약 1초 걸리게: 이동 중 힘 감시가 실제로 도는지 본다."""
    return _make_world(monkeypatch, 25.0)


def _moves(dr):
    return [c for c in dr.calls if c[0] in ('movej', 'movel', 'amovec', 'movec', 'amovel')]


def test_open_then_close(slow_world):
    s, t = slow_world.sim, slow_world.tasks
    t.cover_open()
    assert s.violations == []
    assert s.cover_open and s.held is None and not s.compliance      # 열리고, 놓았고, 순응 제어 꺼짐
    t.cover_close()
    assert s.violations == []
    assert not s.cover_open and s.held is None and not s.compliance
    assert s.max_cover_force < 5.0                                   # 원호가 경첩과 맞으니 뚜껑에 힘이 거의 안 걸림
    assert any('최대 외력' in m for m in slow_world.logs)


def test_motion_sequence(monkeypatch):
    """(이전 hinge 방식) 열기: 홈 → 고리 위 → 수직 하강 → 잡기 → [순응] amovec(중간, 경유) → [순응] amovel 끝점 → 놓기 → 툴 -Z 로 빠지기 → 홈."""
    monkeypatch.setattr(cfg, 'COVER_MODE', 'hinge')
    world = _make_world(monkeypatch, 1e3)
    p, dr = world.tasks.cover, world.dr
    world.tasks.cover_open()
    moves = _moves(dr)
    n = len(p['line'])
    assert n == cfg.COVER_LINE_SEGMENTS and p['line'][-1] == p['end']
    assert [c[0] for c in moves] == ['movej', 'movel', 'movel', 'amovec'] + ['amovel'] * n + ['movel', 'movej']
    assert moves[1][1] == p['above'] and moves[2][1] == p['grip']
    assert moves[3][1:] == (p['mid'], p['via'])
    assert [m[1] for m in moves[4:4 + n]] == p['line']
    assert moves[4 + n][1] == p['end_app']
    assert all(pt[0] == p['grip'][0] for pt in p['line'])
    assert p['above'][:2] == p['grip'][:2] and p['above'][3:] == p['grip'][3:]   # 순수 수직 하강
    assert p['end'][0] == p['via'][0] == p['grip'][0]                             # x 유지, y·z 만 변함
    i = dr.calls.index(moves[3])
    assert dr.calls[i - 3:i] == [('do', cfg.DO_OPEN, 0), ('do', cfg.DO_CLOSE, 1), ('comp_on',)]   # 끄는 것부터 (WebLogic)
    assert dr.calls[i + 1:i + 4] == [('comp_off',), ('comp_on',), moves[4]]
    j = dr.calls.index(moves[3 + n])
    assert dr.calls[j + 1:j + 3] == [('comp_off',), ('do', cfg.DO_CLOSE, 0)]          # 마지막 직선 뒤에 놓기


def test_close_reverses_and_stops_short(monkeypatch):
    """(이전 hinge 방식) 닫기: 받침에 걸친 고리를 잡고 y·z 직선으로 경유점 → 원호 거꾸로, 완전히 닫히기 2도 전에서 놓는다."""
    monkeypatch.setattr(cfg, 'COVER_MODE', 'hinge')
    world = _make_world(monkeypatch, 1e3)
    p, dr = world.tasks.cover, world.dr
    world.tasks.cover_open()
    assert world.sim.cover_open
    dr.calls.clear()
    world.tasks.cover_close()
    moves = _moves(dr)
    n = len(p['line'])
    assert [c[0] for c in moves] == ['movej', 'movel', 'movel'] + ['amovel'] * (n + 1) + ['amovec', 'movel', 'movej']
    assert moves[1][1] == p['rest_app'] and moves[2][1] == p['rest']
    assert [m[1] for m in moves[3:4 + n]] == p['line'][::-1] + [p['via']]
    assert moves[4 + n][1:] == (p['mid'], p['close_end'])
    assert moves[5 + n][1] == p['above']
    gap = sum((a - b) ** 2 for a, b in zip(p['close_end'][:3], p['grip'][:3])) ** 0.5
    assert 5.0 < gap < 9.0                                          # 반지름 190mm x 2도 ≈ 7mm 위에서 놓음
    assert world.sim.violations == [] and not world.sim.cover_open


def test_clamped_ring_normal_path_stays_low(monkeypatch):
    """고리를 꽉 물었다고 해도(놀이 0) 정상 경로는 외력 한계보다 한참 아래 (끝점을 데모만큼 원 밖으로 당기는 몇 mm 뿐)."""
    w = _make_world(monkeypatch, 25.0, ring_slack=0.0)
    w.tasks.cover_open()
    w.tasks.cover_close()
    assert w.sim.violations == [] and w.sim.max_cover_force < cfg.COVER_F_LIMIT / 2


@pytest.mark.parametrize('segments', [3, 2])
def test_segmented_line_stays_near_hinge_circle(monkeypatch, segments):
    """경유 → 끝 직선을 나누면 고리를 꽉 물어도 외력이 낮다."""
    monkeypatch.setattr(cfg, 'COVER_MODE', 'hinge')
    monkeypatch.setattr(cfg, 'COVER_LINE_SEGMENTS', segments)
    w = _make_world(monkeypatch, 25.0, ring_slack=0.0)
    w.tasks.cover_open()
    w.tasks.cover_close()
    assert w.sim.violations == [] and w.sim.max_cover_force < cfg.COVER_F_LIMIT * 0.75


def test_single_straight_line_like_demo(monkeypatch):
    """데모처럼 한 번에 직선(COVER_LINE_SEGMENTS=1): 가운데에서 경첩 원보다 약 32mm 안쪽으로 파고든다.
    고리에 놀이(가짜 기본 25mm)가 있으면 지나가고, 꽉 물었으면 외력 감시가 멈춘다 (파손 전)."""
    from bed_inspect.bed_robot import BedError
    monkeypatch.setattr(cfg, 'COVER_MODE', 'hinge')
    monkeypatch.setattr(cfg, 'COVER_LINE_SEGMENTS', 1)
    w = _make_world(monkeypatch, 25.0)
    w.tasks.cover_open()
    assert w.sim.violations == [] and w.sim.cover_open
    w2 = _make_world(monkeypatch, 25.0, ring_slack=0.0)
    with pytest.raises(BedError, match='외력'):
        w2.tasks.cover_open()
    assert w2.sim.held == 'cover' and not any('경첩 파손' in v for v in w2.sim.violations)


def test_force_guard_stops_wrong_arc(monkeypatch):
    """(고리를 꽉 문 최악의 경우) 경유점이 60mm 틀어진 원호: 뚜껑을 억지로 끌게 된다 → 외력 한계에서 멈추고, 그리퍼는 잡은 채로 둔다."""
    from bed_inspect.bed_robot import BedError
    slow_world = _make_world(monkeypatch, 25.0, ring_slack=0.0)
    bad = dict(slow_world.tasks.cover)
    bad['via'] = bad['via'][:2] + [bad['via'][2] + 60.0] + bad['via'][3:]          # 원보다 60mm 위로 틀어진 끝
    slow_world.tasks.cover = bad
    with pytest.raises(BedError, match='외력'):
        slow_world.tasks.cover_open()
    s = slow_world.sim
    assert ('halt',) in slow_world.dr.calls
    assert s.held == 'cover'                                         # 놓지 않았다 (놓으면 뚜껑이 떨어짐)
    assert not s.compliance                                          # 순응 제어는 꺼졌다
    assert not any('경첩 파손' in v for v in s.violations)            # 부서지기 전에 멈췄다
    assert s.max_cover_force < sim_break(slow_world)


def sim_break(w):
    return w.mod.HINGE_BREAK_N


def test_without_guards_wrong_arc_breaks_hinge(monkeypatch):
    """(고리를 꽉 문 경우) 대조 실험: 순응 제어와 힘 감시를 끄면 같은 틀린 원호에서 가짜 경첩이 부서진다 (보호 장치가 실제로 일하는 증거)."""
    monkeypatch.setattr(cfg, 'COVER_COMPLIANCE', False)
    monkeypatch.setattr(cfg, 'COVER_F_LIMIT', 1e9)
    monkeypatch.setattr(cfg, 'COVER_END_TOL', 1e9)
    slow_world = _make_world(monkeypatch, 25.0, ring_slack=0.0)
    bad = dict(slow_world.tasks.cover)
    bad['via'] = [bad['via'][0], bad['via'][1] + 30.0] + bad['via'][2:]
    slow_world.tasks.cover = bad
    slow_world.tasks.cover_open()
    assert any('경첩 파손' in v for v in slow_world.sim.violations)


def test_stop_button_during_arc(slow_world):
    """원호 도중 정지 버튼 → 정지, 그리퍼는 잡은 채, 순응 제어 꺼짐."""
    from bed_inspect.bed_robot import BedStop
    w = slow_world
    orig = w.dr.amovec

    def amovec_then_stop(*a, **k):
        ret = orig(*a, **k)
        threading.Timer(0.3, w.stop.set).start()
        return ret
    w.dr.amovec = amovec_then_stop
    with pytest.raises(BedStop):
        w.tasks.cover_open()
    assert w.sim.held == 'cover' and not w.sim.compliance
    assert 0.0 < w.sim.cover_strain()[1] < 90.0                     # 반쯤 열린 채 로봇이 들고 있다


def test_start_point_check(world, monkeypatch):
    """원호 직전 위치가 시작점과 다르면 원호를 그리지 않고 멈춘다."""
    from bed_inspect.bed_robot import BedError
    real = world.dr.get_current_posx
    monkeypatch.setattr(world.dr, 'get_current_posx',
                        lambda ref=0: ([v + (5.0 if i == 2 else 0.0) for i, v in enumerate(real()[0])], 2))
    with pytest.raises(BedError, match='벗어나'):
        world.tasks.cover_open()
    assert not any(c[0] in ('movec', 'amovec') for c in world.dr.calls)


def test_loose_grip_on_line_service_mode(monkeypatch):
    """(이전 hinge 방식) 폭 지정 그리퍼('service')일 때: 원호는 꽉(10mm), 경유→끝 직선은 살며시(20mm). 닫을 때는 반대로."""
    monkeypatch.setattr(cfg, 'COVER_MODE', 'hinge')
    w = _make_world(monkeypatch, 1e3)
    monkeypatch.setattr(cfg, 'GRIPPER_MODE', 'service')
    seq = []

    def grip(key):
        seq.append(key)
        w.dr.calls.append(('grip', key))
        w.sim.set_width(cfg.GRIP_WIDTH[key])
    w.tasks.r.grip = grip
    w.tasks.cover_open()
    assert seq == ['cover_open', 'cover_grip', 'cover_loose', 'cover_open']
    calls = w.dr.calls
    i_loose = calls.index(('grip', 'cover_loose'))
    assert calls[i_loose - 2][0] == 'amovec' and calls[i_loose + 2][0] == 'amovel'   # 원호 뒤, 직선 앞
    seq.clear()
    w.tasks.cover_close()
    assert seq == ['cover_open', 'cover_loose', 'cover_grip', 'cover_open']
    assert w.sim.violations == [] and not w.sim.cover_open



def test_delta_mode_sequence(monkeypatch):
    """(delta 방식, 기본) 열기: 홈 → 고리 위 → 수직 하강 → 꽉 잡기 → [순응] 원호 80도 → 살며시 → [순응] 직선 끝점(100도)
    → 놓기 → 툴 -Z 로 빠지기 → 위로 → 뚜껑 넘어 시작점 위 → 홈.  닫기: 홈 → 시작점 위 → 받침 쪽 위 → 받침 고리 앞 → 들어가기
    → 꽉 잡기 → [순응] 원호 한 번(닫히기 2도 전) → 놓기 → 수직 상승 → 홈.  열기와 닫기는 따로 호출."""
    monkeypatch.setattr(cfg, 'COVER_MODE', 'delta')
    world = _make_world(monkeypatch, 1e3)
    p, dr = world.tasks.cover, world.dr
    assert cfg.COVER_SINGLE_ARC and p['line'] == [] and p['via'] == p['end']
    assert p['end_deg'] + p['grip_angle'] == pytest.approx(cfg.COVER_END_DIR_DEG, abs=1e-3)   # 경첩 기준 115도
    assert p['end_out'] == pytest.approx(0.0, abs=1e-3)
    assert p['radius'] == cfg.COVER_ARC_RADIUS and p['grip'][:3] == pytest.approx(cfg.COVER_GRIP_XYZ)
    world.tasks.cover_open()
    moves = _moves(dr)
    kinds = [c[0] for c in moves]
    assert kinds == ['movej', 'movel', 'movel', 'amovec', 'movel', 'movel', 'movel', 'movej']   # moveC 한 번, 직선 없음
    assert moves[3][1:] == (p['mid'], p['end']) and moves[4][1] == p['end_app']
    assert [m[1] for m in moves[5:-1]][-2:] == [p['exit_up'], p['exit_over']]
    assert world.sim.violations == [] and world.sim.cover_open and world.sim.held is None
    dr.calls.clear()
    world.tasks.cover_close()
    moves = _moves(dr)
    assert [c[0] for c in moves] == ['movej', 'movel', 'movel', 'movel', 'movel', 'amovec', 'movel', 'movej']
    assert [m[1] for m in moves[1:5]] == [p['exit_over'], p['rest_up'], p['rest_app'], p['rest']]
    assert moves[5][1:] == (p['close_mid'], p['close_end']) and moves[6][1] == p['above']
    assert world.sim.violations == [] and not world.sim.cover_open


def test_delta_mode_clamped_ring_low_force(monkeypatch):
    """(delta 방식) 고리를 꽉 물어도(놀이 0) 끝점이 원 위라 외력이 낮다 (데모 끝 차이를 그대로 쓰면 원 밖으로 당겨 멈춤)."""
    from bed_inspect.bed_robot import BedError
    monkeypatch.setattr(cfg, 'COVER_MODE', 'delta')
    w = _make_world(monkeypatch, 25.0, ring_slack=0.0)
    w.tasks.cover_open()
    w.tasks.cover_close()
    assert w.sim.violations == [] and w.sim.max_cover_force < cfg.COVER_F_LIMIT / 2
    for k, v in (('COVER_SINGLE_ARC', False), ('COVER_END_DIR_DEG', None), ('COVER_END_DEG', None)):
        monkeypatch.setattr(cfg, k, v)                               # 데모 끝 차이 그대로 → 원 밖으로 당김
    w2 = _make_world(monkeypatch, 25.0, ring_slack=0.0)
    with pytest.raises(BedError, match='외력'):
        w2.tasks.cover_open()



def test_taught_mode_two_arcs(monkeypatch):
    """(교시 방식, 기본) 열기: 원호 1 (시작 → lift 교시점, 경첩 원) → 원호 2 (lift → 끝 교시점, 원보다 조금 안쪽) → 놓기.
    닫기: 끝점에서 잡고 원호 2 거꾸로 → 원호 1 거꾸로 (닫히기 2도 전). 가짜 경첩에서 위반 없음."""
    monkeypatch.setattr(cfg, 'COVER_MODE', 'taught')
    world = _make_world(monkeypatch, 1e3)
    p, dr = world.tasks.cover, world.dr
    assert p['via'] == pytest.approx(cfg.COVER_LIFT_POSX) and p['end'] == pytest.approx(cfg.COVER_END_POSX)
    assert 60 < p['via_deg'] < 85 and 95 < p['end_deg'] < 115
    world.tasks.cover_open()
    moves = _moves(dr)
    arcs = [m for m in moves if m[0] == 'amovec']
    assert [a[1:] for a in arcs] == [(p['mid'], p['via']), (p['arc2_mid'], p['end'])]
    assert world.sim.violations == [] and world.sim.cover_open and world.sim.held is None
    dr.calls.clear()
    world.tasks.cover_close()
    arcs = [m for m in _moves(dr) if m[0] == 'amovec']
    assert [a[1:] for a in arcs] == [(p['arc2_mid'], p['via']), (p['close_mid'], p['close_end'])]
    assert world.sim.violations == [] and not world.sim.cover_open



def test_one_arc_mode(world):
    """(교시 2점 원호 하나, 기본) 열기: 시작 → 끝 moveC 한 번 (반지름 COVER_TAUGHT_RADIUS) → 놓기 → 위로 빠지기.
    닫기: 끝에서 잡고 같은 원으로 moveC 한 번. 가짜 경첩에서 위반 없음."""
    import math
    assert cfg.COVER_MODE == 'one_arc'
    p, dr = world.tasks.cover, world.dr
    assert p['grip'][:3] == pytest.approx(cfg.COVER_ONE_START_POSX[:3]) and p['end'][:3] == pytest.approx(cfg.COVER_ONE_END_POSX[:3])
    if cfg.COVER_KEEP_VERTICAL:                       # 원호 내내 툴 Z 수직 (자세 하나)
        assert p['grip'][3:] == p['mid'][3:] == p['end'][3:] and p['grip'][4] == -180.0
    _, yc, zc = p['hinge']
    for k in ('grip', 'mid', 'end'):
        assert math.hypot(p[k][1] - yc, p[k][2] - zc) == pytest.approx(cfg.COVER_TAUGHT_RADIUS, abs=0.01)
    world.tasks.cover_open()
    arcs = [m for m in _moves(dr) if m[0] == 'amovec']
    assert [a[1:] for a in arcs] == [(p['mid'], p['end'])]
    assert world.sim.violations == [] and world.sim.cover_open and world.sim.held is None
    dr.calls.clear()
    world.tasks.cover_close()
    arcs = [m for m in _moves(dr) if m[0] == 'amovec']
    assert [a[1:] for a in arcs] == [(p['close_mid'], p['close_end'])]
    assert world.sim.violations == [] and not world.sim.cover_open
