# 모의 미션 노드: 로봇 없이 HMI 를 시험하기 위한 가짜 미션
#
#   ros2 run cushion_hmi mock_mission
#   ros2 run cushion_hmi mock_mission --ros-args -p defects:="[2,5]" -p spares:=2 -p bad_spares:=1
#
# 파라미터
#   defects     처음부터 무른 모듈이 들어 있는 슬롯 번호 (결함 모의)      기본 [4]
#   spares      선반에 있는 정상 교체용 모듈 수                          기본 2
#   bad_spares  교체용 모듈 중 앞에서부터 몇 개가 불량인지 (재시도 시험)  기본 0
#
# 플로우차트와 같은 순서로 상태를 바꾸며 /cushion/status, /cushion/force, /cushion/log 를 발행한다.
# 천 조이기/풀기, 커버 열기/닫기처럼 방식이 미정인 단계는 "작업자 확인" 단계로 두었다.
# 실제 미션 노드를 만들 때 이 파일의 구조(제너레이터 상태 머신, 발행 형식)를 그대로 따라가면 된다.

import random
import statistics

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from cushion_hmi import protocol as P

TICK = 0.02            # 타이머 주기 [s]. 제너레이터를 한 번 진행시키는 간격

# 모의 실리콘 특성 (실제 값 아님)
NORMAL_K = 6.0         # 정상 모듈 강성 [N/mm]
SOFT_K = 2.5           # 무른 모듈 강성 [N/mm]
NOMINAL_Z0 = 120.0     # 정상 모듈 접촉 높이 [mm]
SOFT_SAG = 2.0         # 무른 모듈 처짐 [mm]
D_MAX = 8.0            # 최대 압입 깊이 [mm]
F_MAX = 30.0           # 최대 압입 힘 [N]
FIT_FROM = 1.0         # 이 깊이부터 직선 근사 (접촉 초기 구간 제외) [mm]

# 판정 기준 (초안): k < K_RATIO * k_ref  또는  z0 < z0_ref - Z0_DROP
K_RATIO = 0.8
Z0_DROP = 1.0
MAX_RETRY = 2


def fit_slope(zs, fs):
    """최소제곱 직선 기울기."""
    n = len(zs)
    if n < 2:
        return 0.0
    mz, mf = sum(zs) / n, sum(fs) / n
    den = sum((z - mz) ** 2 for z in zs)
    return sum((z - mz) * (f - mf) for z, f in zip(zs, fs)) / den if den else 0.0


class MockMission(Node):
    def __init__(self):
        super().__init__('cushion_mock_mission')
        self.declare_parameter('defects', [4])
        self.declare_parameter('spares', 2)
        self.declare_parameter('bad_spares', 0)

        self.pub_status = self.create_publisher(String, P.TOPIC_STATUS, 10)
        self.pub_force = self.create_publisher(String, P.TOPIC_FORCE, 100)
        self.pub_log = self.create_publisher(String, P.TOPIC_LOG, 50)
        self.create_subscription(String, P.TOPIC_CMD, self.on_cmd, 10)
        self.create_timer(TICK, self.tick)
        self.create_timer(0.5, self.publish_status)

        self.baseline = None   # calibrate 모드 결과 {id: (k, z0)}
        self.reset_all()
        self.log(f'모의 미션 노드 시작. 무른 모듈 슬롯: {sorted(self.soft)}, 선반 재고: {self.spares}')

    # ------------------------------------------------------------------ 상태
    def reset_all(self):
        self.job = None
        self.running = False
        self.state = 'IDLE'
        self.mode = 'full'
        self.current = None
        self.wait_op = False
        self.prompt = ''
        self.go = False
        self.alarm = ''
        self.retry = 0
        self.ref = {}
        self.soft = set(self.get_parameter('defects').value)
        self.spares = int(self.get_parameter('spares').value)
        self.bad_spares = int(self.get_parameter('bad_spares').value)
        self.modules = {i: self.blank(i) for i in P.module_ids()}

    @staticmethod
    def blank(mid):
        return {'id': mid, 'status': 'unknown', 'k': None, 'z0': None}

    def publish_status(self):
        d = {
            'state': self.state, 'mode': self.mode, 'running': self.running,
            'current_module': self.current,
            'modules': [self.modules[i] for i in P.module_ids()],
            'wait_operator': self.wait_op, 'prompt': self.prompt,
            'alarm': self.alarm, 'retry': self.retry, 'spares': self.spares,
        }
        self.pub_status.publish(String(data=P.encode(d)))

    def log(self, text):
        self.get_logger().info(text)
        self.pub_log.publish(String(data=text))

    def set_state(self, s):
        self.state = s
        self.log(f'상태 → {s} ({P.STATE_LABELS.get(s, s)})')
        self.publish_status()

    # ------------------------------------------------------------------ 명령
    def on_cmd(self, msg):
        d = P.decode(msg.data)
        if d is None:
            return
        cmd = d.get('cmd')
        if cmd == 'start':
            if self.running:
                self.log('이미 실행 중입니다')
                return
            mode = d.get('mode', 'full')
            if mode not in P.MODE_KEYS:
                self.log(f'알 수 없는 모드: {mode}')
                return
            self.mode = mode
            self.modules = {i: self.blank(i) for i in P.module_ids()}
            self.alarm = ''
            self.retry = 0
            self.job = self.mission()
            self.running = True
            self.log(f'시작 (모드: {mode})')
        elif cmd == 'stop':
            if self.running:
                self.job = None
                self.running = False
                self.wait_op = False
                self.prompt = ''
                self.current = None
                self.alarm = '작업자 정지 요청으로 중단했습니다'
                self.set_state('IDLE')
        elif cmd == 'continue':
            if self.wait_op:
                self.go = True
        elif cmd == 'reset':
            if self.running:
                self.log('실행 중에는 초기화할 수 없습니다')
            else:
                self.reset_all()
                self.log('초기화: 모듈 상태, 선반 재고를 처음으로 되돌렸습니다')
        self.publish_status()

    def tick(self):
        if self.job is None:
            return
        try:
            next(self.job)
        except StopIteration:
            self.job = None
            self.running = False
            self.current = None
            self.publish_status()
        except Exception as e:  # noqa: B902  실제 노드에서는 여기서 안전 복구(SafetyGuard)
            self.job = None
            self.running = False
            self.alarm = f'예외 발생: {e}'
            self.set_state('FAULT')

    # ------------------------------------------------------------------ 동작 단위
    def sleep(self, sec):
        for _ in range(max(1, int(sec / TICK))):
            yield

    def ask(self, prompt):
        """작업자 확인 단계. HMI 의 [작업 완료 · 계속] 을 기다린다."""
        self.wait_op, self.prompt, self.go = True, prompt, False
        self.log('작업자 확인 대기: ' + prompt)
        self.publish_status()
        while not self.go:
            yield
        self.wait_op, self.prompt = False, ''
        self.publish_status()

    def measure(self, mid):
        """압입 측정 1회 (모의). 실제로는 MeasureSkill 이 로봇으로 수행."""
        soft = mid in self.soft
        k_true = (SOFT_K if soft else NORMAL_K) * random.uniform(0.97, 1.03)
        z0 = NOMINAL_Z0 - (SOFT_SAG if soft else 0.0) + random.gauss(0, 0.15)

        m = self.modules[mid]
        m.update(status='measuring', k=None, z0=None)
        self.current = mid
        self.publish_status()
        yield from self.sleep(0.4)   # 상공 이동, 저속 하강, 접촉 검출

        zs, fs = [], []
        depth, first = 0.0, True
        while depth <= D_MAX + 1e-9:
            f = k_true * depth * (1 + 0.04 * depth) + random.gauss(0, 0.08)
            self.pub_force.publish(String(data=P.encode(
                {'module': mid, 'z': round(depth, 3), 'fz': round(f, 3), 'reset': first})))
            first = False
            if depth >= FIT_FROM:
                zs.append(depth)
                fs.append(f)
            if f >= F_MAX:
                break
            depth += 0.1
            yield

        k = fit_slope(zs, fs)
        m.update(status='measured', k=round(k, 2), z0=round(z0, 2))
        self.publish_status()
        self.log(f'M{mid} 측정: k = {k:.2f} N/mm, z0 = {z0:.2f} mm')
        yield from self.sleep(0.3)   # 후퇴

    def make_ref(self):
        """판정 기준값. calibrate 결과가 있으면 위치별 값, 없으면 전체 중앙값."""
        if self.baseline:
            return dict(self.baseline)
        ks = [m['k'] for m in self.modules.values() if m['k'] is not None]
        zs = [m['z0'] for m in self.modules.values() if m['z0'] is not None]
        kr, zr = statistics.median(ks), statistics.median(zs)
        return {i: (kr, zr) for i in P.module_ids()}

    def judge(self, ids):
        bad = []
        for mid in ids:
            m = self.modules[mid]
            kr, zr = self.ref[mid]
            if m['k'] < K_RATIO * kr or m['z0'] < zr - Z0_DROP:
                m['status'] = 'defect'
                bad.append(mid)
            else:
                m['status'] = 'ok'
        self.publish_status()
        return bad

    # ------------------------------------------------------------------ 미션 (플로우차트)
    def mission(self):
        ids = P.module_ids()
        self.set_state('INIT')
        yield from self.sleep(0.8)

        self.set_state('MEASURE')
        for mid in ids:
            yield from self.measure(mid)
        self.current = None

        if self.mode == 'calibrate':
            self.baseline = {i: (self.modules[i]['k'], self.modules[i]['z0']) for i in ids}
            for i in ids:
                self.modules[i]['status'] = 'ok'
            self.log('기준값 저장 (이후 판정은 위치별 기준값 사용)')
            yield from self.report()
            return

        self.set_state('JUDGE')
        self.ref = self.make_ref()
        defects = self.judge(ids)
        yield from self.sleep(0.5)
        if not defects:
            self.log('불량 모듈 없음')
            yield from self.report()
            return
        self.log(f'불량 모듈: {", ".join(f"M{i}" for i in defects)}')
        if self.mode == 'diagnose':
            yield from self.report()
            return

        while True:
            self.set_state('COVER_OPEN')
            yield from self.ask('커버를 열어 주세요')

            self.set_state('REPLACE')
            replaced = []
            for mid in defects:
                if self.spares <= 0:
                    self.alarm = '선반에 정상 모듈이 없습니다. 재고를 채워 주세요'
                    self.log(self.alarm)
                    break
                self.current = mid
                self.publish_status()
                self.log(f'M{mid} 제거 → 폐기 위치')
                yield from self.sleep(1.0)
                self.spares -= 1
                self.log(f'선반에서 교체용 모듈 픽업 (남은 재고 {self.spares})')
                yield from self.sleep(0.8)
                self.log(f'M{mid} 자리에 순응 제어 삽입')
                yield from self.sleep(0.8)
                if self.bad_spares > 0:
                    self.bad_spares -= 1      # 교체용도 불량인 경우 (재시도 시험용)
                else:
                    self.soft.discard(mid)
                self.modules[mid].update(status='replaced', k=None, z0=None)
                replaced.append(mid)
                self.publish_status()
            self.current = None

            self.set_state('COVER_CLOSE')
            yield from self.ask('커버를 닫아 주세요')
            if not replaced:
                break

            self.set_state('REMEASURE')
            targets = sorted(set(replaced).union(*(P.neighbors(i) for i in replaced)))
            self.log(f'재측정 대상 (교체 + 인접): {", ".join(f"M{i}" for i in targets)}')
            for mid in targets:
                yield from self.measure(mid)
            self.current = None
            defects = self.judge(targets)
            if not defects:
                self.log('재측정 결과 모두 정상')
                break
            self.retry += 1
            if self.retry >= MAX_RETRY:
                self.alarm = (f'{MAX_RETRY}회 재시도 후에도 불량: '
                              + ', '.join(f'M{i}' for i in defects) + '. 작업자 확인 필요')
                self.log(self.alarm)
                break
            self.log(f'재측정 불량 {", ".join(f"M{i}" for i in defects)} → 재시도 {self.retry}')

        yield from self.report()

    def report(self):
        self.set_state('REPORT')
        for i in P.module_ids():
            m = self.modules[i]
            self.log(f'  M{i}: {P.MODULE_STATUS[m["status"]]:<5}  k={m["k"]}  z0={m["z0"]}')
        yield from self.sleep(0.5)
        self.set_state('DONE')


def main(args=None):
    rclpy.init(args=args)
    node = MockMission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # launch 에서 Ctrl+C 하면 SIGINT 가 두 번 올 수 있어 정리 중 예외도 무시
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
