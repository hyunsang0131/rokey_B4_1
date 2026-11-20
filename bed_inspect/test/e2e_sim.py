# 전체 공정 검증: 가짜 로봇(bed_sim) + 공정 노드(bed_process)를 실제로 띄우고,
# 화면이 보내는 것과 같은 명령을 보내서 끝까지 돌려 본다.
#
#   python3 test/e2e_sim.py            # 전체 시나리오
#   python3 test/e2e_sim.py replace    # 이름에 replace 가 들어간 시나리오만
#
# 확인하는 것: 최종 상태, 불량 판정, 실제 블록 위치(가짜 세계) = 재고 기록, 위반 0건

import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from bed_inspect import bed_config as cfg
from bed_inspect import bed_math

LATCH = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
LOG_DIR = os.environ.get('E2E_LOG_DIR', '/tmp/bed_e2e')
SPEED = os.environ.get('E2E_SPEED', '40')


class Harness:
    def __init__(self, name, sim_args=(), env=None):
        os.makedirs(LOG_DIR, exist_ok=True)
        self.name = name
        self.sim_log = open(os.path.join(LOG_DIR, f'{name}_sim.log'), 'w')
        self.proc_log = open(os.path.join(LOG_DIR, f'{name}_process.log'), 'w')
        self.sim = subprocess.Popen(['ros2', 'run', 'bed_inspect', 'bed_sim', '--speed', SPEED, '--seed', '1',
                                     *sim_args], stdout=self.sim_log, stderr=subprocess.STDOUT,
                                    start_new_session=True, env=dict(os.environ, **(env or {})))
        time.sleep(2.0)
        self.proc = subprocess.Popen(['ros2', 'run', 'bed_inspect', 'bed_process', cfg.SIM_TEACH_FILE],
                                     stdout=self.proc_log, stderr=subprocess.STDOUT, start_new_session=True,
                                     env=dict(os.environ, **(env or {})))
        self.node = rclpy.create_node('e2e_' + name)
        self.status, self.world, self.logs, self.history = None, None, [], []
        self.node.create_subscription(String, '/bed/status', self._on_status, LATCH)
        self.node.create_subscription(String, '/bed_sim/world', self._on_world, 10)
        self.node.create_subscription(String, '/bed/log', lambda m: self.logs.append(m.data), 50)
        self.pub = self.node.create_publisher(String, '/bed/cmd', 10)
        self.human = self.node.create_publisher(String, '/bed_sim/human', 10)
        self.wait(lambda: self.status and self.status['state'] in ('idle', 'error'), 40, '공정 노드 준비')
        assert self.status['state'] == 'idle', self.status['message']
        self.wait(lambda: self.pub.get_subscription_count() > 0, 10, '명령 구독')

    def _on_status(self, m):
        self.status = json.loads(m.data)
        self.history.append((self.status['state'], self.status['step']))

    def _on_world(self, m):
        self.world = json.loads(m.data)

    def spin(self, sec=0.05):
        rclpy.spin_once(self.node, timeout_sec=sec)

    def wait(self, cond, timeout, what):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin()
            if cond():
                return
            if self.proc.poll() is not None or self.sim.poll() is not None:
                raise AssertionError(f'{what}: 프로세스가 죽었습니다 (로그: {LOG_DIR}/{self.name}_*.log)')
        raise AssertionError(f'{what}: {timeout}초 안에 끝나지 않음. 상태={self.status and self.status["state"]} '
                             f'단계={self.status and self.status["step"]} 메시지={self.status and self.status["message"]}')

    def send(self, **cmd):
        self.pub.publish(String(data=json.dumps(cmd)))

    def run(self, timeout=240, **cmd):
        """명령을 보내고 끝날 때까지(실행 중이 아닐 때까지) 기다린다. 끝난 상태를 돌려준다."""
        stamp = self.status['stamp']
        self.send(**cmd)
        self.wait(lambda: self.status['stamp'] > stamp and self.status['state'] == 'running', 10, '시작')
        self.wait(lambda: self.status['state'] not in ('running',), timeout, f'{cmd} 실행')
        return self.status['state']

    def settle(self):
        """가짜 세계의 최신 상태를 받는다."""
        t = time.monotonic()
        self.world = None
        self.wait(lambda: self.world is not None and time.monotonic() - t > 0.5, 5, '세계 상태')
        return self.world

    def check_consistent(self):
        """기록된 재고 = 실제(가짜 세계) 블록 위치, 위반 없음, 손에 든 것 없음."""
        w = self.settle()
        inv = self.status['inventory']
        assert w['violations'] == [], w['violations']
        assert w['bed'] == inv['bed'], (w['bed'], inv['bed'])
        assert w['rack'] == inv['rack'], (w['rack'], inv['rack'])
        assert w['held'] is None
        assert w['cover_open'] == self.status['world']['cover_open']
        return w

    def grades(self, key):
        return [b['grade'] for b in self.status[key]['blocks']]

    def close(self):
        with open(os.path.join(LOG_DIR, f'{self.name}_status.json'), 'w') as f:
            json.dump(self.status, f, ensure_ascii=False, indent=1)
        for p in (self.proc, self.sim):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in (self.proc, self.sim):
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
        self.node.destroy_node()
        self.sim_log.close()
        self.proc_log.close()


# ── 시나리오 ──────────────────────────────────────────────────────────
def sc_measure_ok(h, tol=0.03, ztol=0.3):
    """불량 없음: 측정만 → 전부 정상, 기울기 0, 강성이 실제값(2.0)과 3% 이내."""
    assert h.run(cmd='start', mode='measure') == 'done', h.status['message']
    assert h.grades('first') == ['ok'] * 6
    for b in h.status['first']['blocks']:
        assert abs(b['k'] - 2.0) / 2.0 < tol, b
        assert abs(b['z_surf'] - (120.0 - 0.5)) < ztol, b       # 표면 120mm, 천 여유 0.5mm
    t = h.status['first']['tilt']
    assert abs(t['col_deg']) < 0.2 and abs(t['row_deg']) < 0.2, t
    w = h.check_consistent()
    assert not w['cover_open'] and w['max_force'] < cfg.F_LIMIT
    assert math.dist(w['pose'][:3], [350.0, 0.0, 350.0]) < 0.5          # 홈으로 돌아옴


def sc_replace(h):
    """2번, 5번 불량: 교체 모드 → 불량 2개를 버리고 예비로 바꾼 뒤 재측정 정상."""
    assert h.run(cmd='start', mode='replace') == 'done', h.status['message']
    assert h.grades('first') == ['ok', 'fail', 'ok', 'ok', 'fail', 'ok']
    assert h.grades('second') == ['ok'] * 6
    assert h.status['plan']['faulty'] == [2, 5] and h.status['plan']['done'] == 4
    w = h.check_consistent()
    assert w['discarded'] == [30.0, 30.0] and w['discarded_bad'] == [True, True]   # 버린 것이 정말 불량 블록
    assert w['bed_bad'] == [False] * 6
    assert not w['cover_open']
    steps = [s for _, s in h.history if s]
    assert sorted(set(steps)) == list(range(1, 10)), sorted(set(steps))            # 9단계를 전부 거침


def sc_replace_no_spare(h):
    """예비 블록이 1개뿐인데 불량이 2개: 1개만 바꾸고 경고."""
    assert h.run(cmd='start', mode='replace') == 'warning', h.status['message']
    assert h.status['plan']['skipped'] == [5]
    assert h.grades('second') == ['ok', 'ok', 'ok', 'ok', 'fail', 'ok']
    h.check_consistent()


def sc_replace_none(h):
    """불량 없음: 교체 모드는 3단계에서 끝나고 커버를 열지 않는다."""
    assert h.run(cmd='start', mode='replace') == 'done'
    assert max(s for _, s in h.history) == cfg.STEP_CHECK
    h.check_consistent()


def sc_stop_and_resume(h):
    """측정 중 정지 → 다시 시작해도 천을 두 번 감지 않고 정상 완료."""
    stamp = h.status['stamp']
    h.send(cmd='start', mode='replace')
    h.wait(lambda: h.status['stamp'] > stamp and h.status['step'] == cfg.STEP_MEASURE and h.status['active_slot'] == 3, 60, '3번 측정')
    time.sleep(0.3)
    h.send(cmd='stop')
    h.wait(lambda: h.status['state'] == 'stopped', 20, '정지')
    w = h.settle()
    assert w['pose'][2] > 170.0, w['pose']                 # 누른 채 멈추지 않고 올라와 있음
    assert h.run(cmd='start', mode='replace') == 'done', h.status['message']
    w = h.check_consistent()
    assert w['discarded_bad'] == [True]


def refused(h, **cmd):
    """명령을 보내면 '확인 필요'라서 거부되는지 확인."""
    stamp = h.status['stamp']
    h.send(**cmd)
    h.wait(lambda: h.status['stamp'] > stamp and h.status['state'] == 'error', 10, '거부')
    assert '재고 수정' in h.status['message'], h.status['message']


def save_real_state(h):
    """사람이 실제 상태를 보고 재고 수정에 그대로 입력하는 것을 흉내."""
    h.human.publish(String(data='tidy'))
    time.sleep(0.5)
    w = h.settle()
    assert w['held'] is None and None not in w['bed']
    stamp = h.status['stamp']
    h.send(cmd='set_inventory', bed=w['bed'], rack=w['rack'], cover_open=w['cover_open'])
    h.wait(lambda: h.status['stamp'] > stamp and h.status['state'] == 'idle', 10, '재고 저장')
    assert not h.status['world']['uncertain']


def sc_stop_during_swap(h):
    """블록을 옮기는 중 정지 → 재고 확인 전에는 시작을 거부 → 재고 저장 후 (열린 커버부터 닫고) 정상 완료."""
    stamp = h.status['stamp']
    h.send(cmd='start', mode='replace')
    h.wait(lambda: h.status['stamp'] > stamp and h.status['step'] == cfg.STEP_PUT, 120, '블록 넣기 단계')
    time.sleep(0.4)
    h.send(cmd='stop')
    h.wait(lambda: h.status['state'] == 'stopped', 20, '정지')
    assert h.status['world']['uncertain'] and h.status['world']['cover_open']
    refused(h, cmd='start', mode='measure')
    save_real_state(h)
    assert h.run(cmd='start', mode='replace') == 'done', h.status['message']
    w = h.check_consistent()
    assert not w['cover_open'] and w['discarded_bad'] == [True] and w['bed_bad'] == [False] * 6


def sc_stop_during_swap_no_cover(h):
    """커버 없음: 블록을 옮기는 중 정지 → 재고 확인 전에는 시작을 거부 → 재고 저장 후 정상 완료."""
    stamp = h.status['stamp']
    h.send(cmd='start', mode='replace')
    h.wait(lambda: h.status['stamp'] > stamp and h.status['step'] == cfg.STEP_PUT, 120, '블록 넣기 단계')
    time.sleep(0.4)
    h.send(cmd='stop')
    h.wait(lambda: h.status['state'] == 'stopped', 20, '정지')
    assert h.status['world']['uncertain'] and not h.status['world']['cover_open']
    refused(h, cmd='start', mode='measure')
    save_real_state(h)
    assert h.run(cmd='start', mode='replace') == 'done', h.status['message']
    w = h.check_consistent()
    assert w['discarded_bad'] == [True] and w['bed_bad'] == [False] * 6


def sc_stop_during_cover(h):
    """커버를 여는 중 정지 → 거부 → 재고 저장 후 정상 완료."""
    stamp = h.status['stamp']
    h.send(cmd='start', mode='replace')
    h.wait(lambda: h.status['stamp'] > stamp and h.status['step'] == cfg.STEP_OPEN, 120, '커버 열기 단계')
    time.sleep(0.3)
    h.send(cmd='stop')
    h.wait(lambda: h.status['state'] == 'stopped', 20, '정지')
    assert h.status['world']['uncertain']
    refused(h, cmd='start', mode='replace')
    refused(h, cmd='calibrate')
    refused(h, cmd='home')                                   # 커버를 물고 있을 수 있으므로 홈 이동도 막는다
    save_real_state(h)
    assert h.run(cmd='start', mode='replace') == 'done', h.status['message']
    w = h.check_consistent()
    assert not w['cover_open'] and w['discarded_bad'] == [True]


def sc_press_test(h):
    """press_test 프로그램: bed_2 를 두 번 누르고 저장. 강성이 2.0 근처로 나오는지."""
    assert h.run(cmd='start', mode='measure') == 'done'
    names = [p['name'] for p in json.load(open(cfg.SIM_TEACH_FILE))]
    n = names.index('bed_2') + 1
    out = subprocess.run(['ros2', 'run', 'bed_inspect', 'press_test', cfg.SIM_TEACH_FILE],
                         input=f'{n}\n{n}\nr\nq\n', capture_output=True, text=True, timeout=120).stdout
    open(os.path.join(LOG_DIR, 'press_test_out.txt'), 'w').write(out)
    ks = [float(line.split()[1]) for line in out.splitlines() if line.strip().startswith('bed_2 ') and 'N/mm' in line]
    assert len(ks) >= 2 and all(abs(k - 2.0) < 0.06 for k in ks), out[-1500:]
    assert '2회 평균' in out and '저장:' in out, out[-1500:]
    w = h.settle()
    assert w['violations'] == [] and w['pose'][2] > 170.0


def sc_confirm(h):
    """교체 전 확인: 취소하면 아무것도 안 바꾸고, 진행하면 교체한다."""
    stamp = h.status['stamp']
    h.send(cmd='start', mode='replace', confirm=True)
    h.wait(lambda: h.status['stamp'] > stamp and h.status['state'] == 'confirm', 120, '확인 대기')
    assert h.status['plan']['moves'] == ['침대 3번 (30mm) → 버림', '보관대 1번 (30mm) → 침대 3번']
    h.send(cmd='cancel')
    h.wait(lambda: h.status['state'] == 'stopped', 30, '취소')
    w = h.check_consistent()
    assert w['discarded'] == [] and not w['cover_open']
    stamp = h.status['stamp']
    h.send(cmd='start', mode='replace', confirm=True)
    h.wait(lambda: h.status['stamp'] > stamp and h.status['state'] == 'confirm', 120, '확인 대기')
    h.send(cmd='confirm')
    h.wait(lambda: h.status['state'] in ('done', 'warning', 'error', 'stopped'), 240, '교체')
    assert h.status['state'] == 'done', h.status['message']
    assert h.check_consistent()['discarded_bad'] == [True]


def sc_calibrate(h):
    """기준값 저장 → 이후 판정은 기준값으로."""
    assert h.run(cmd='calibrate') == 'done', h.status['message']
    assert abs(h.status['calib']['30.0'] - 2.0) < 0.06
    assert h.run(cmd='start', mode='measure') == 'done'
    assert all(b['ref_src'] == 'calib' for b in h.status['first']['blocks'])
    h.check_consistent()


def sc_all_soft_needs_calib(h):
    """6개가 전부 똑같이 물러졌을 때: 중앙값 비교로는 못 잡는다는 한계를 확인(문서에 적은 내용)."""
    assert h.run(cmd='start', mode='measure') == 'done'
    assert h.grades('first') == ['ok'] * 6
    assert all(abs(b['k'] - 1.0) < 0.05 for b in h.status['first']['blocks'])


COVER = {'BED_COVER': '1'}                               # 커버가 있는 구성으로 시험 (bed_sim, bed_process 둘 다)
TALL = f'{cfg.TALL_HEIGHT:g}'
TALL_RACK = ('--rack', f'{TALL},30,30,30,{TALL},-', '--rack-kind', 'S,H,S,H,S,-')   # 보관대: T, H, S, H, T, 빈자리
REPAIR_RACK = ('--rack', '30,30,30,30,30,-', '--rack-kind', 'S,H,S,H,S,-')
HARD, SOFT = 2.0 * 2.0, 2.0 * 0.5                         # bed_sim: K_REF x KIND


def kinds(w):
    """가짜 세계의 침대 6칸 종류: H(40A) / S(10A) / T(키 큰 10A) / -(빈칸)."""
    out = []
    for h, k in zip(w['bed'], w['bed_k']):
        if h is None:
            out.append('-')
        elif h == cfg.TALL_HEIGHT:
            out.append('T')
        else:
            out.append('H' if abs(k - HARD) < 1e-6 else 'S' if abs(k - SOFT) < 1e-6 else '?')
    return out


def sc_repair(h, profile=None, empties=(3,), mismatch=(4,), want='HHHHHH'):
    """자동 보수: 빈칸과 맞지 않는 칸을 찾고, 보관대 5개를 눌러 보고 골라 넣은 뒤 재측정.
    빼낸 모듈은 버리지 않고 보관대로 되돌린다. want: 끝난 뒤 침대 6칸의 종류."""
    cmd = {'target': 'user', 'profile': profile} if profile else {'target': 'auto'}
    assert h.run(timeout=400, cmd='start', mode='repair', **cmd) == 'done', h.status['message']
    plan = h.status['plan']
    assert plan['empties'] == list(empties) and plan['mismatch'] == list(mismatch), plan
    assert len(plan['rack']) == 5 and plan['done'] == len(plan['moves']), plan
    assert h.grades('second') == ['ok'] * 6, h.status['second']
    w = h.check_consistent()
    assert ''.join(kinds(w)) == want, (kinds(w), want)
    assert w['discarded'] == [] and not w['cover_open']
    assert sum(k is not None for k in w['rack_k']) == 5 - len(empties)       # 빼낸 모듈은 보관대로
    assert any('환산 비율 제안' in line for line in h.logs), h.logs[-10:]
    assert sum('살짝 닿기:' in line for line in h.logs) == 5, h.logs              # 보관대 5개 모두 살짝 닿기
    for r in plan['rack']:
        assert (r['k'] is None) == bool(r.get('tall')), r                         # 높은 큐브만 누르지 않음
    steps = {s for _, s in h.history if s}
    assert steps == set(range(1, 13)), steps
    assert not any('롤러' in line and '건너뜀' not in line for line in h.logs if '단계' in line)


def sc_repair_rack_not_empty(h):
    """예외 처리: 기록상 비어 있는 보관대 6번에 실제로는 큐브가 있음 → 놓기 전에 살짝 닿아서 알아채고 멈춘다."""
    stamp = h.status['stamp']
    h.send(cmd='set_inventory', bed=[30.0] * 6, rack=[30.0] * 5 + [None])     # 사람이 6번을 빈자리로 잘못 적음
    h.wait(lambda: h.status['stamp'] > stamp and h.status['state'] == 'idle', 10, '재고 저장')
    assert h.run(timeout=400, cmd='start', mode='repair', target='auto') == 'error', h.status['message']
    assert '6번이 기록과 달리 비어 있지 않습니다' in h.status['message'], h.status['message']
    w = h.settle()
    assert w['violations'] == [] and w['held'] is None, w                      # 부딪히지 않았고 아무것도 들고 있지 않음
    assert w['rack'][5] is not None and w['discarded'] == []
    assert math.dist(w['pose'][:3], [350.0, 0.0, 350.0]) < 0.5                 # 홈에서 멈춤
    assert not h.status['world']['uncertain']                                  # 옮기는 도중이 아니므로 확인 필요 표시 없음


def sc_repair_none(h):
    """고칠 칸이 없으면 4단계에서 끝나고 커버를 열지 않는다."""
    assert h.run(timeout=300, cmd='start', mode='repair', target='auto') == 'done', h.status['message']
    assert max(s for _, s in h.history) == 4
    assert h.status['plan']['moves'] == []
    h.check_consistent()


def sc_repair_short(h):
    """부드러운 칸을 4개 원하는데 보관대에 부드러운 큐브가 3개뿐 → 아무것도 옮기지 않고 오류."""
    prof = ['soft', 'soft', 'hard', 'soft', 'hard', 'soft']
    assert h.run(timeout=300, cmd='start', mode='repair', target='user', profile=prof) == 'error'
    assert '큐브' in h.status['message'], h.status['message']
    w = h.check_consistent()
    assert not w['cover_open'] and w['discarded'] == []


def sc_set_inventory(h):
    """재고 수정: 잘못된 값은 거부, 맞는 값은 저장."""
    h.send(cmd='set_inventory', bed=[30, 30, 30, 30, 30, 31], rack=cfg.RACK_INIT)
    h.wait(lambda: '실패' in h.status['message'], 5, '거부')
    assert h.status['inventory']['bed'] == [30.0] * 6


SCENARIOS = [
    ('measure_ok', (), sc_measure_ok),
    ('measure_neg_sign_noisy', ('--sign', '-1', '--noise', '0.5'), lambda h: sc_measure_ok(h, tol=0.08, ztol=0.8)),
    ('replace', ('--fault', '2,5', '--rack', '30,30,25,35,35,-'), sc_replace),
    ('replace_io_gripper', ('--fault', '2,5', '--rack', '30,30,25,35,35,-'), sc_replace, {'BED_GRIPPER_MODE': 'io'}),
    ('replace_no_spare', ('--fault', '2,5', '--rack', '30,25,25,35,35,-'), sc_replace_no_spare),
    ('replace_none', (), sc_replace_none),
    ('stop_and_resume', ('--fault', '4'), sc_stop_and_resume),
    ('stop_during_swap', ('--fault', '1'), sc_stop_during_swap, COVER),
    ('stop_during_cover', ('--fault', '6'), sc_stop_during_cover, COVER),
    ('replace_with_cover', ('--fault', '2,5', '--rack', '30,30,25,35,35,-'), sc_replace, COVER),
    ('stop_during_swap_no_cover', ('--fault', '1'), sc_stop_during_swap_no_cover),
    ('press_test', (), sc_press_test),
    ('confirm', ('--fault', '3'), sc_confirm),
    ('calibrate', (), sc_calibrate),
    ('all_soft', ('--fault', '1,2,3,4,5,6'), sc_all_soft_needs_calib),
    ('set_inventory', (), sc_set_inventory),
    ('repair_auto', ('--bed', '30,30,-,30,30,30', '--bed-kind', 'H,H,-,S,H,H', *REPAIR_RACK), sc_repair),
    ('repair_user', ('--bed', '30,30,-,30,30,30', '--bed-kind', 'H,H,-,H,H,H', *REPAIR_RACK),
     lambda h: sc_repair(h, ['hard', 'hard', 'soft', 'soft', 'hard', 'hard'], want='HHSSHH')),
    ('repair_user_mixed', ('--bed-kind', 'H,S,H,S,S,H', *REPAIR_RACK),
     lambda h: sc_repair(h, ['hard', 'soft', 'hard', 'soft', 'hard', 'hard'], empties=(), mismatch=(5,),
                         want='HSHSHH')),
    ('repair_none', ('--bed-kind', 'H,H,H,H,H,H', *REPAIR_RACK), sc_repair_none),
    ('repair_rack_not_empty', ('--bed-kind', 'H,H,H,S,H,H', '--rack', '30,30,30,30,30,30',
                               '--rack-kind', 'S,H,S,H,S,H'), sc_repair_rack_not_empty),
    # 실제 준비물 구성: 40A 6개, 10A 2개, 키 큰 10A 2개 (침대 5 + 보관대 5)
    ('repair_auto_tall', ('--bed', f'30,30,-,{TALL},30,30', '--bed-kind', 'H,H,-,S,H,H', *TALL_RACK),
     sc_repair),                                                     # 4번 키 큰 큐브 → 주변과 맞지 않음
    ('repair_custom_tall', ('--bed', '30,30,-,30,30,30', '--bed-kind', 'H,H,-,S,H,H', *TALL_RACK),
     lambda h: sc_repair(h, ['tall', 'tall', 'soft', 'soft', 'hard', 'hard'], empties=(3,), mismatch=(1, 2),
                         want='TTSSHH')),
    ('repair_user_short', ('--bed-kind', 'H,H,H,H,H,H', *REPAIR_RACK), sc_repair_short),
]


def main():
    only = sys.argv[1:]
    rclpy.init()
    failed = []
    for name, args, fn, *rest in SCENARIOS:
        if only and not any(o in name for o in only):
            continue
        t0 = time.monotonic()
        h = None
        try:
            h = Harness(name, args, rest[0] if rest else None)
            fn(h)
            print(f'PASS  {name}  ({time.monotonic() - t0:.0f}s)', flush=True)
        except AssertionError as e:
            failed.append(name)
            print(f'FAIL  {name}: {e}', flush=True)
            if h:
                print('   마지막 로그:', *h.logs[-6:], sep='\n     ', flush=True)
        finally:
            if h:
                h.close()
            time.sleep(1.0)
    rclpy.shutdown()
    print('\n결과:', '전부 통과' if not failed else f'실패 {failed}')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
