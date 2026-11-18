# measure_teach_force 측정값을 cushion_hmi 화면으로 보내는 연결부 (표시 전용)
#
# - 교시점 순서 1~6 을 모듈 M1~M6 에 대응시킨다 (배치: 1 2 / 3 4 / 5 6, 위 = 선반, 왼쪽 = 로봇).
#   교시점이 6개보다 많으면 7번째부터는 HMI 에 표시하지 않는다 (측정·CSV 는 그대로).
# - 샘플마다 /cushion/force 로 {module, t, fz} 를 보내서 HMI 그래프 가로축이 시간이 된다.
# - /cushion/status 는 백그라운드 스레드가 0.5초마다 보낸다.
#   (메인 스레드가 input() 에서 기다리는 동안에도 HMI 가 "연결됨"을 유지하도록)
# - HMI 명령(/cushion/cmd)은 받지 않는다. status 에 control=false 를 실어 HMI 버튼을 끈다.
# - cushion_hmi 패키지가 없으면 조용히 꺼지고, 측정 스크립트는 원래대로 동작한다.

import threading
import time

from std_msgs.msg import String

try:
    from cushion_hmi import protocol as P
except ImportError:
    P = None

SOURCE = 'measure_teach_force'


class HmiLink:
    def __init__(self, node, points):
        self.enabled = P is not None
        self.log_ = node.get_logger()
        if not self.enabled:
            self.log_.warn('cushion_hmi 가 없어 HMI 표시를 끕니다 (측정은 그대로 진행)')
            return

        ids = P.module_ids()
        if len(points) != len(ids):
            self.log_.warn(f'교시점이 {len(points)}개입니다. HMI 는 앞에서부터 {len(ids)}개까지만 '
                           f'M1~M{len(ids)} 로 표시합니다.')
        self.n = min(len(points), len(ids))
        self.modules = {
            mid: {'id': mid, 'status': 'unknown', 'name': points[mid - 1]['name'],
                  'fz': None, 'fz_std': None}
            for mid in ids if mid <= self.n
        }
        for mid in ids:
            if mid > self.n:
                self.modules[mid] = {'id': mid, 'status': 'unknown', 'name': '(교시점 없음)',
                                     'fz': None, 'fz_std': None}

        self.state = 'IDLE'
        self.current = None
        self.alarm = ''
        self.t0 = 0.0
        self.first = True
        self.lock = threading.Lock()

        self.pub_status = node.create_publisher(String, P.TOPIC_STATUS, 10)
        self.pub_force = node.create_publisher(String, P.TOPIC_FORCE, 100)
        self.pub_log = node.create_publisher(String, P.TOPIC_LOG, 50)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._heartbeat, daemon=True)
        self._thread.start()
        self.log(f'측정 스크립트 연결. 교시점 {self.n}개 → M1~M{self.n}')

    # ------------------------------------------------------------------ 내부
    def _heartbeat(self):
        while not self._stop.wait(0.5):
            self.publish_status()

    def _mid(self, idx):
        """교시점 인덱스(0부터) -> 모듈 번호. HMI 에 표시하지 않는 점이면 None."""
        return idx + 1 if self.enabled and idx < self.n else None

    def publish_status(self):
        if not self.enabled:
            return
        with self.lock:
            d = {
                'state': self.state, 'mode': 'teach_force', 'running': self.state == 'MEASURE',
                'current_module': self.current,
                'modules': [dict(self.modules[i]) for i in P.module_ids()],
                'wait_operator': False, 'prompt': '', 'alarm': self.alarm,
                'source': SOURCE, 'control': False,
            }
        self.pub_status.publish(String(data=P.encode(d)))

    # ------------------------------------------------------------------ 측정 스크립트에서 호출
    def begin(self, idx):
        """교시점 idx 측정 시작 (이동 전)."""
        mid = self._mid(idx)
        if mid is None:
            return
        with self.lock:
            self.state, self.current, self.alarm = 'MEASURE', mid, ''
            self.modules[mid].update(status='measuring', fz=None, fz_std=None)
            self.first = True
        self.publish_status()

    def start_sampling(self, idx):
        """도착·대기 후 힘 읽기를 시작하는 시점. 그래프 시간 0."""
        self.t0 = time.monotonic()

    def sample(self, idx, fz):
        mid = self._mid(idx)
        if mid is None:
            return
        d = {'module': mid, 't': round(time.monotonic() - self.t0, 3), 'fz': round(fz, 3),
             'reset': self.first}
        self.first = False
        self.pub_force.publish(String(data=P.encode(d)))

    def result(self, idx, fz_mean, fz_std):
        mid = self._mid(idx)
        if mid is None:
            return
        with self.lock:
            self.modules[mid].update(status='measured', fz=round(fz_mean, 2),
                                     fz_std=round(fz_std, 2))
        self.publish_status()

    def failed(self, idx, msg):
        """측정 실패·로봇 이상. idx 가 None 이면 특정 교시점과 관계없는 이상."""
        if not self.enabled:
            return
        mid = None if idx is None else self._mid(idx)
        with self.lock:
            if mid is not None:
                self.modules[mid]['status'] = 'unknown'
            self.state, self.current, self.alarm = 'FAULT', None, msg
        self.publish_status()

    def idle(self):
        if not self.enabled:
            return
        with self.lock:
            if self.state != 'FAULT':
                self.state = 'IDLE'
            self.current = None
        self.publish_status()

    def log(self, text):
        if self.enabled:
            self.pub_log.publish(String(data=text))

    def close(self):
        if not self.enabled:
            return
        self._stop.set()
        self._thread.join(timeout=1.0)
        with self.lock:
            self.state, self.current = 'DONE', None
        self.publish_status()
