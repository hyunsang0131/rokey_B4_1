# 공정 명령을 터미널에서 하나씩 실행하는 프로그램. bed_process 가 떠 있어야 한다 (cushion_system.launch.py).
#
#   ros2 run bed_inspect bed_cmd inspect                 # ① 첫 공정 전체: 측정 → 판정 → 커버 열기 → 교체 → 커버 닫기 → 재측정
#                                                        #    (기본: 자동 보수 = 빈칸 채우기 + 주변과 맞지 않는 모듈 교체)
#   ros2 run bed_inspect bed_cmd inspect --replace       #    기준값 대비 불량 교체 모드로 (기준값 측정이 먼저 필요)
#   ros2 run bed_inspect bed_cmd order                   # ② 주문대로 구성: 웹 주문을 기다렸다가(이미 받은 게 있으면 그것) 9단계
#   ros2 run bed_inspect bed_cmd order 주문.json         #    웹 대신 파일 (bed_ordering.html 이 내려받는 JSON)
#   ros2 run bed_inspect bed_cmd order --wait            #    이미 받은 주문은 무시하고 새 주문을 기다림
#   ros2 run bed_inspect bed_cmd measure | calibrate | home | stop
#   ros2 run bed_inspect bed_cmd wait_robot              # 로봇(bringup) 서비스가 뜰 때까지 기다림 (launch 에서 씀)
#
# 시작한 뒤에는 단계가 바뀔 때마다 화면에 찍고, 끝나면(완료/경고/오류/정지) 결과를 찍고 끝난다.
# Ctrl+C 를 누르면 공정에 정지를 보낸다.

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String

from bed_inspect.cushion_bridge import LAST_ORDER_TOPIC, PROFILE_KO, TOPIC_ORDER, order_to_profile

LATCH = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
END_STATES = ('done', 'warning', 'error', 'stopped')


class Runner:
    def __init__(self, node):
        self.node = node
        self.status = None
        self.order = None
        self.pub = node.create_publisher(String, '/bed/cmd', 10)
        node.create_subscription(String, '/bed/status', self._on_status, LATCH)
        node.create_subscription(String, '/bed/log', lambda m: print('  ' + m.data), 50)
        node.create_subscription(String, TOPIC_ORDER, self._on_order, 10)          # 새 웹 주문 (어떤 QoS 서버와도 맞음)
        node.create_subscription(String, LAST_ORDER_TOPIC, self._on_order, LATCH)  # 이미 받은 마지막 주문

    def _on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            pass

    def _on_order(self, msg):
        try:
            self.order = json.loads(msg.data)
        except ValueError:
            pass

    def spin_until(self, cond, timeout=None):
        end = None if timeout is None else time.monotonic() + timeout
        while rclpy.ok() and not cond():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if end is not None and time.monotonic() > end:
                return False
        return True

    def send(self, cmd):
        self.pub.publish(String(data=json.dumps(cmd, ensure_ascii=False)))

    def run(self, cmd, ready_timeout=15.0):
        """명령을 보내고 끝날 때까지 기다린다. 결과 상태를 돌려준다."""
        ok = lambda: (self.status is not None and self.node.count_subscribers('/bed/cmd') > 0     # noqa: E731
                      and self.status.get('state') != 'starting')
        self.spin_until(lambda: False, 1.0)            # 다른 노드들을 알아챌 시간
        n = self.node.count_subscribers('/bed/cmd')
        if n > 1:
            print(f'공정 노드(bed_process)가 {n}개 떠 있습니다. 예전에 켠 터미널(launch)을 전부 Ctrl+C 로 끄고 하나만 켜세요.')
            return 'error'
        if not self.spin_until(ok, ready_timeout):
            print('공정 노드(bed_process)가 없습니다. 먼저 ros2 launch bed_inspect cushion_system.launch.py ... 를 켜세요.')
            return 'error'
        if self.status.get('state') in ('running', 'confirm', 'starting'):
            print(f'공정이 진행 중입니다 ({self.status.get("step_name")}). 끝날 때까지 기다립니다...')
            if not self.spin_until(lambda: self.status.get('state') not in ('running', 'confirm', 'starting'),
                                   ready_timeout):
                print('공정이 끝나지 않아 시작하지 않습니다.')
                return 'error'
        stamp = self.status.get('stamp', 0)
        print(f'시작: {cmd}')
        self.send(cmd)
        # 새 상태가 'running' 으로 바뀌는 것을 본 뒤, 끝 상태가 될 때까지
        if not self.spin_until(lambda: self.status.get('stamp', 0) > stamp and self.status.get('state') not in ('idle',), 15):
            print('공정이 명령을 받지 않았습니다.')
            return 'error'
        last = None
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.2)
            s = self.status
            key = (s.get('step'), s.get('state'))
            if key != last:
                last = key
                if s.get('state') == 'confirm':
                    print(f'>> 확인 대기: {s.get("message")}  (화면에서 진행/취소)')
            if s.get('state') in END_STATES and s.get('stamp', 0) > stamp:
                print(f'\n결과: {s.get("state")}  {s.get("message") or ""}')
                return s.get('state')
        return 'stopped'


def load_order_file(path):
    with open(path) as f:
        return json.load(f)


def main(args=None):
    ap = argparse.ArgumentParser(prog='bed_cmd')
    ap.add_argument('what', choices=['inspect', 'order', 'measure', 'calibrate', 'home', 'stop', 'wait_robot'])
    ap.add_argument('--ready-timeout', type=float, default=120.0, help='bed_process 가 뜰 때까지 기다리는 시간 [s]')
    ap.add_argument('file', nargs='?', help='order: 주문 JSON 파일 (없으면 웹 주문)')
    ap.add_argument('--replace', action='store_true', help='inspect: 기준값 대비 불량 교체 모드')
    ap.add_argument('--wait', action='store_true', help='order: 이미 받은 주문은 무시하고 새 주문을 기다림')
    ap.add_argument('--confirm', action='store_true', help='교체 전에 계획을 화면에서 확인')
    ap.add_argument('--slot-map', default='1,2,3,4,5,6', help='웹 Slot n → 공정 칸 번호 (기본 1:1)')
    opt = ap.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init(args=args)
    node = Node('bed_cmd')
    r = Runner(node)
    code = 0
    try:
        if opt.what == 'wait_robot':
            srv = '/dsr01/dsr_controller2/system/get_robot_state'
            print(f'로봇 서비스 기다리는 중: {srv}')
            end = time.monotonic() + opt.ready_timeout
            while rclpy.ok() and time.monotonic() < end:
                names = [n for n, _ in node.get_service_names_and_types()]
                if srv in names:
                    print('로봇 준비됨')
                    time.sleep(3.0)                     # 컨트롤러가 STANDBY 로 자리 잡을 시간
                    return
                time.sleep(1.0)
            print('로봇 서비스가 뜨지 않았습니다. bringup 로그를 확인하세요 (가상 모드는 한 번 더 켜면 되는 경우가 많음).')
            code = 1
            return
        if opt.what == 'stop':
            r.spin_until(lambda: node.count_subscribers('/bed/cmd') > 0, 5)
            r.send({'cmd': 'stop'})
            r.spin_until(lambda: False, 0.5)
            print('정지를 보냈습니다.')
            return
        if opt.what == 'inspect':
            cmd = {'cmd': 'start', 'mode': 'replace'} if opt.replace else \
                  {'cmd': 'start', 'mode': 'repair', 'target': 'auto'}
        elif opt.what == 'measure':
            cmd = {'cmd': 'start', 'mode': 'measure'}
        elif opt.what in ('calibrate', 'home'):
            cmd = {'cmd': opt.what}
        else:                                           # order
            slot_map = [int(v) for v in opt.slot_map.split(',')]
            if opt.file:
                order = load_order_file(opt.file)
            else:
                r.spin_until(lambda: r.order is not None, 1.5)      # 이미 받은 주문 (주문 서버가 마지막 것을 갖고 있음)
                if opt.wait or r.order is None:
                    r.order = None
                    print('웹에서 주문하세요 (bed_ordering.html 에서 ADD TO CART). 기다리는 중... (Ctrl+C 취소)')
                    r.spin_until(lambda: r.order is not None)
                order = r.order
            profile = order_to_profile(order, slot_map)
            print(f'주문 {order.get("order_id")}: ' +
                  ', '.join(f'{i + 1}번 {PROFILE_KO.get(p, p)}' for i, p in enumerate(profile)))
            cmd = {'cmd': 'start', 'mode': 'repair', 'target': 'user', 'profile': profile}
        if opt.confirm and cmd.get('cmd') == 'start':
            cmd['confirm'] = True
        state = r.run(cmd, opt.ready_timeout)
        code = 0 if state in ('done', 'warning') else 1
    except KeyboardInterrupt:
        print('\n중단: 공정에 정지를 보냅니다.')
        r.send({'cmd': 'stop'})
        r.spin_until(lambda: False, 0.5)
        code = 1
    except (OSError, ValueError) as e:
        print(f'오류: {e}')
        code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(code)


if __name__ == '__main__':
    main()
