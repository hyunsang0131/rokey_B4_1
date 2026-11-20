# cushion_hmi(팀원 화면, 웹 주문) ↔ bed_process(9단계 공정) 연결 노드.
#
#   ros2 run bed_inspect cushion_bridge                         # 주문은 받아서 화면에 보여 주기만 (시작은 bed_cmd order 또는 화면)
#   ros2 run bed_inspect cushion_bridge --ros-args -p auto_start:=true    # 주문이 오면 바로 시작
#
# 하는 일
#   /bed/status  → /cushion/status   공정 상태를 cushion HMI 형식으로 (0.5초마다, HMI 연결 표시용)
#   /bed/log     → /cushion/log      로그 그대로
#   /cushion/cmd → /bed/cmd          HMI 버튼을 공정 명령으로
#       calibrate → calibrate,  diagnose → measure,  full → replace,  repair → repair(자동),
#       order → repair(사용자 지정, 마지막 주문),  stop → stop,  continue → confirm(교체 전 확인 대기일 때)
#   /cushion/order_config (order_server 가 웹 주문을 보냄)
#       → 슬롯 6개(부드러움/단단함)를 profile(soft/hard)로 바꿔 저장
#       → auto_start 면 공정이 쉬고 있을 때 바로 {"mode": "repair", "target": "user", "profile": [...]} 시작
#         (측정 → 주문과 다른 칸·빈칸 찾기 → 보관대에서 맞는 큐브로 교체 → 재측정 = 9단계 그대로)
#         공정이 도는 중이면 끝난 뒤에 시작한다.
#
# 칸 번호: 웹 Slot n = HMI 모듈 Mn = 공정 n번 칸 (셋 다 3행 x 2열, 1 2 / 3 4 / 5 6 순서). 다르면 slot_map 으로 바꾼다.
#   예: -p slot_map:="[2,1,4,3,6,5]"  → Slot 1 이 공정 2번 칸

import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

try:
    from cushion_hmi import protocol as P
    TOPIC_CMD, TOPIC_STATUS, TOPIC_LOG = P.TOPIC_CMD, P.TOPIC_STATUS, P.TOPIC_LOG
    TOPIC_FORCE = P.TOPIC_FORCE
    TOPIC_ORDER = getattr(P, 'TOPIC_ORDER_CONFIG', '/cushion/order_config')
except ImportError:                                 # cushion_hmi 가 없어도 같은 이름으로 동작
    TOPIC_CMD, TOPIC_STATUS, TOPIC_LOG = '/cushion/cmd', '/cushion/status', '/cushion/log'
    TOPIC_FORCE = '/cushion/force'
    TOPIC_ORDER = '/cushion/order_config'

LAST_ORDER_TOPIC = '/bed/last_order'           # cushion_bridge 가 남겨 두는 마지막 웹 주문 (latched)
STATUS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# 공정 단계 번호 → cushion HMI 상태 이름
STEP_STATE = {0: 'IDLE', 1: 'INIT', 2: 'MEASURE', 3: 'JUDGE', 4: 'COVER_OPEN', 5: 'REPLACE', 6: 'REPLACE',
              7: 'COVER_CLOSE', 8: 'REMEASURE', 9: 'REPORT'}
# 공정 판정 → HMI 모듈 상태
GRADE_STATUS = {'ok': 'ok', 'warn': 'warn', 'fail': 'defect', 'mismatch': 'mismatch', 'empty': 'empty',
                'nodata': 'unknown'}
# 웹 주문 값 → 공정 profile
ORDER_PROFILE = {'부드러움': 'soft', '단단함': 'hard', 'soft': 'soft', 'hard': 'hard', 'tall': 'tall'}
IDLE_STATES = ('idle', 'done', 'warning', 'stopped')


def order_to_profile(order, slot_map):
    """웹 주문 {"slots": {"Slot 1": "부드러움", ...}} → 공정 칸 순서의 profile ['soft', 'hard', ...]."""
    slots = order.get('slots')
    if not isinstance(slots, dict):
        raise ValueError('slots 가 없습니다')
    n = len(slot_map)
    profile = [None] * n
    for i in range(n):
        v = slots.get(f'Slot {i + 1}')
        if v not in ORDER_PROFILE:
            raise ValueError(f'Slot {i + 1} 값이 이상합니다: {v!r}')
        profile[slot_map[i] - 1] = ORDER_PROFILE[v]
    if None in profile:
        raise ValueError(f'slot_map 이 이상합니다: {slot_map}')
    return profile


def cmd_for(mode, profile):
    """HMI 모드 → 공정 명령 (dict). 못 바꾸면 ValueError."""
    if mode == 'calibrate':
        return {'cmd': 'calibrate'}
    if mode == 'diagnose':
        return {'cmd': 'start', 'mode': 'measure'}
    if mode == 'full':
        return {'cmd': 'start', 'mode': 'replace'}
    if mode == 'repair':
        return {'cmd': 'start', 'mode': 'repair', 'target': 'auto'}
    if mode == 'order':
        if not profile:
            raise ValueError('받은 주문이 없습니다. 웹에서 주문(ADD TO CART)을 먼저 보내세요.')
        return {'cmd': 'start', 'mode': 'repair', 'target': 'user', 'profile': list(profile)}
    raise ValueError(f'알 수 없는 모드: {mode}')


def hmi_mode(bed):
    """공정 status 의 mode → HMI 모드 이름."""
    m = bed.get('mode')
    if m == 'measure':
        return 'diagnose'
    if m == 'replace':
        return 'full'
    if m == 'repair':
        plan = bed.get('plan') or {}
        return 'order' if plan.get('target_mode') == 'user' else 'repair'
    return m


def to_cushion_status(bed, slot_map, order=None, pending=False):
    """bed_process 의 /bed/status → cushion HMI 의 /cushion/status."""
    st = bed.get('state') or 'idle'
    step = int(bed.get('step') or 0)
    running = st in ('running', 'confirm', 'starting')
    if st == 'confirm':
        state = 'CONFIRM'
    elif running:
        state = STEP_STATE.get(step, 'INIT')
    elif st in ('done', 'warning'):
        state = 'DONE'
    elif st == 'error':
        state = 'FAULT'
    else:
        state = 'IDLE'
    # 재측정을 시작했으면 재측정 결과, 아니면 1차 결과
    second = bed.get('second') or {}
    use = second if (second.get('blocks') and (step >= 8 or second.get('done'))) else (bed.get('first') or {})
    blocks = {b.get('slot'): b for b in (use.get('blocks') or [])}
    active = bed.get('active_slot')
    modules = []
    for i in range(1, len(slot_map) + 1):          # HMI 모듈 i = 공정 slot_map[i-1] 번 칸
        b = blocks.get(slot_map[i - 1]) or {}
        g = b.get('grade')
        status = GRADE_STATUS.get(g) if g else ('measured' if b.get('k') is not None else 'unknown')
        if running and active == slot_map[i - 1] and step in (2, 8):
            status = 'measuring'
        modules.append({'id': i, 'status': status, 'k': b.get('k'),
                        'z0': b.get('z_surf') if b.get('z_surf') is not None else b.get('z0')})
    current = None
    if active is not None and active in slot_map:
        current = slot_map.index(active) + 1
    rack = (bed.get('inventory') or {}).get('rack') or []
    msg = bed.get('message') or ''
    out = {
        'state': state, 'mode': hmi_mode(bed), 'running': running, 'current_module': current,
        'modules': modules, 'wait_operator': st == 'confirm',
        'prompt': msg if st == 'confirm' else '',
        'alarm': msg if st in ('error', 'warning') else '',
        'retry': 0, 'spares': sum(1 for h in rack if h is not None),
        'source': 'bed_process', 'control': True,
        'step': step, 'step_name': bed.get('step_name') or '', 'detail': bed.get('detail') or '',
    }
    if order:
        out['order'] = {'order_id': order.get('order_id'), 'pending': pending}
    return out


class CushionBridge:
    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()
        v = node.declare_parameter('auto_start', False).value
        self.auto_start = v.strip().lower() in ('1', 'true', 'yes', 'on') if isinstance(v, str) else bool(v)
        self.slot_map = [int(v) for v in node.declare_parameter('slot_map', [1, 2, 3, 4, 5, 6]).value]
        if sorted(self.slot_map) != list(range(1, len(self.slot_map) + 1)):
            raise SystemExit(f'slot_map 이 1~{len(self.slot_map)} 를 한 번씩 써야 합니다: {self.slot_map}')
        self.lock = threading.Lock()
        self.bed = None                 # 마지막 /bed/status
        self.order = None               # 마지막 주문 (dict)
        self.profile = None             # 그 주문의 profile
        self.pending = False            # 공정이 끝나면 시작할 주문이 있음
        self.pub_bed = node.create_publisher(String, '/bed/cmd', 10)
        self.pub_status = node.create_publisher(String, TOPIC_STATUS, 10)
        self.pub_log = node.create_publisher(String, TOPIC_LOG, 50)
        # 마지막으로 받은 주문을 남겨 둔다 (나중에 켠 bed_cmd order 가 바로 쓸 수 있게)
        self.pub_last = node.create_publisher(String, LAST_ORDER_TOPIC, STATUS_QOS)
        node.create_subscription(String, '/bed/status', self.on_bed_status, STATUS_QOS)
        node.create_subscription(String, '/bed/log', self.on_bed_log, 50)
        node.create_subscription(String, '/bed/curve', self.on_bed_curve, 20)
        self.pub_force = node.create_publisher(String, TOPIC_FORCE, 200)
        node.create_subscription(String, TOPIC_CMD, self.on_hmi_cmd, 10)
        node.create_subscription(String, TOPIC_ORDER, self.on_order, 10)
        node.create_timer(0.5, self.publish_status)
        self.say(f'cushion_bridge 시작 (주문 자동 시작 {"켬" if self.auto_start else "끔"}, 칸 대응 {self.slot_map})')

    # ── 보내기 ──
    def say(self, text):
        self.log.info(text)
        self.pub_log.publish(String(data='[연결] ' + text))

    def send_bed(self, cmd):
        self.pub_bed.publish(String(data=json.dumps(cmd, ensure_ascii=False)))

    def publish_status(self):
        with self.lock:
            bed = self.bed
            order, pending = self.order, self.pending
        # 공정 노드가 없으면 보내지 않는다 → HMI 가 "응답 없음"으로 표시
        if bed is None or self.node.count_publishers('/bed/status') == 0:
            return
        self.pub_status.publish(String(data=json.dumps(
            to_cushion_status(bed, self.slot_map, order, pending), ensure_ascii=False)))

    # ── 받기 ──
    def on_bed_status(self, msg):
        try:
            bed = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.bed = bed
            start = self.pending and bed.get('state') in IDLE_STATES
            if start:
                self.pending = False
        if start:
            self.start_order()
        self.publish_status()

    def on_bed_curve(self, msg):
        """공정이 한 칸을 다 누르면 그 힘-변위 곡선을 HMI 그래프로 보낸다 (가로축 = 접촉점부터 눌린 깊이)."""
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        if d.get('kind') != 'bed' or d.get('slot') not in self.slot_map:
            return
        mid = self.slot_map.index(d['slot']) + 1
        curve = d.get('curve') or []
        z_ref = d.get('z0') if d.get('z0') is not None else (curve[0][0] if curve else 0.0)
        pts = [(round(z_ref - z, 3), f) for z, f in curve if z_ref - z >= -0.5]   # 접촉 전(공중) 점은 뺀다
        for n, (depth, f) in enumerate(pts):
            self.pub_force.publish(String(data=json.dumps(
                {'module': mid, 'z': depth, 'fz': f, 'reset': n == 0})))

    def on_bed_log(self, msg):
        self.pub_log.publish(String(data=msg.data))

    def on_hmi_cmd(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        c = d.get('cmd')
        if c == 'start':
            try:
                cmd = cmd_for(d.get('mode', 'full'), self.profile)
            except ValueError as e:
                self.say(f'시작 못 함: {e}')
                return
            self.say(f'HMI 시작 → 공정 {cmd}')
            self.send_bed(cmd)
        elif c == 'stop':
            with self.lock:
                self.pending = False
            self.send_bed({'cmd': 'stop'})
        elif c == 'continue':
            if (self.bed or {}).get('state') == 'confirm':
                self.send_bed({'cmd': 'confirm'})
        elif c == 'reset':
            self.say('초기화: 공정에는 명령을 보내지 않습니다 (재고는 bed_hmi 의 재고 수정에서 바꿉니다).')

    def on_order(self, msg):
        try:
            order = json.loads(msg.data)
            profile = order_to_profile(order, self.slot_map)
        except (ValueError, AttributeError) as e:
            self.say(f'주문을 읽지 못했습니다: {e}')
            return
        with self.lock:
            self.order, self.profile = order, profile
            state = (self.bed or {}).get('state')
        self.pub_last.publish(String(data=json.dumps(order, ensure_ascii=False)))
        text = ', '.join(f'{i + 1}번 {"부드럽게" if p == "soft" else "단단하게"}' for i, p in enumerate(profile))
        self.say(f'주문 {order.get("order_id")} 받음: {text}')
        if not self.auto_start:
            self.say('자동 시작이 꺼져 있습니다. HMI 에서 "주문대로 구성"을 골라 시작하세요.')
            return
        if self.bed is None or self.node.count_publishers('/bed/status') == 0:
            with self.lock:
                self.pending = True
            self.say('공정 노드(bed_process)가 아직 없습니다. 켜지면 이 주문으로 시작합니다.')
        elif state in IDLE_STATES:
            self.start_order()
        elif state == 'error':
            self.say('공정이 오류 상태라 자동으로 시작하지 않습니다. 원인을 해결한 뒤 "주문대로 구성"으로 시작하세요.')
        else:
            with self.lock:
                self.pending = True
            self.say('공정이 진행 중입니다. 끝나면 이 주문으로 시작합니다.')

    def start_order(self):
        cmd = cmd_for('order', self.profile)
        self.say(f'주문 {(self.order or {}).get("order_id")} 시작: 측정 → 주문대로 교체 → 재측정')
        self.send_bed(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Node('cushion_bridge')
    CushionBridge(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
