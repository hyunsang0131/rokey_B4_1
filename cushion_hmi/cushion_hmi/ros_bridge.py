# ROS 2 <-> Qt 연결
#
# Qt 이벤트 루프(app.exec_)와 rclpy.spin 은 둘 다 무한 루프라 한 스레드에서 같이 못 돈다.
# 그래서 rclpy 는 QThread 에서 spin_once 를 반복하고, 받은 메시지는 Qt 시그널로 넘긴다.
# 시그널은 스레드를 건너면 자동으로 큐에 쌓였다가 GUI 스레드에서 처리되므로 안전하다.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

from PyQt5.QtCore import QThread, pyqtSignal

from cushion_hmi import protocol as P


class RosBridge(QThread):
    status_received = pyqtSignal(dict)
    force_received = pyqtSignal(dict)
    log_received = pyqtSignal(str)
    order_config_received = pyqtSignal(dict)
    cmd_result = pyqtSignal(dict)
    search_live = pyqtSignal(dict)      # 큐브 탐지 실시간 (/cube_search/live, pjt_test1 cube_search LivePub)       # 명령 서비스 응답 {'cmd', 'accepted', 'message', 'state'}

    def __init__(self, node_name='cushion_hmi'):
        super().__init__()
        self._running = True
        self.node = Node(node_name)
        self.pub_cmd = self.node.create_publisher(String, P.TOPIC_CMD, 10)
        self.pub_order_config = self.node.create_publisher(
            String, P.TOPIC_ORDER_CONFIG, 10)
        self.node.create_subscription(String, P.TOPIC_STATUS, self._on_status, 10)
        self.node.create_subscription(String, P.TOPIC_FORCE, self._on_force, 100)
        self.node.create_subscription(String, P.TOPIC_LOG, self._on_log, 50)
        self.node.create_subscription(
            String, P.TOPIC_ORDER_CONFIG, self._on_order_config, 10)
        self.node.create_subscription(String, '/cube_search/live', self._on_search_live, 200)
        # 명령은 서비스로 보내 응답을 받는다 (HMI → cushion_bridge → bed_process). 2026-11-20
        try:
            from bed_interfaces.srv import Command
            self._Command = Command
            self.cmd_cli = self.node.create_client(Command, P.SRV_COMMAND)
        except ImportError:
            self._Command, self.cmd_cli = None, None
        self._pending = []              # (future, cmd, 보낸 시각)

    # --- ROS 콜백 (이 QThread 안에서 실행) ---
    def _on_status(self, msg):
        d = P.decode(msg.data)
        if d is not None:
            self.status_received.emit(d)

    def _on_force(self, msg):
        d = P.decode(msg.data)
        if d is not None:
            self.force_received.emit(d)

    def _on_log(self, msg):
        self.log_received.emit(msg.data)

    def _on_search_live(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(d, dict):
            self.search_live.emit(d)

    def _on_order_config(self, msg):
        d = P.decode(msg.data)
        if d is not None:
            self.order_config_received.emit(d)

    def run(self):
        import time
        while self._running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            now = time.monotonic()
            for item in list(self._pending):
                fut, cmd, t0 = item
                if fut.done():
                    self._pending.remove(item)
                    r = fut.result()
                    self.cmd_result.emit({'cmd': cmd, 'accepted': bool(r and r.accepted),
                                          'message': r.message if r else '응답 없음', 'state': r.state if r else ''})
                elif now - t0 > P.CMD_TIMEOUT:
                    self._pending.remove(item)
                    fut.cancel()
                    self.cmd_result.emit({'cmd': cmd, 'accepted': False, 'state': '',
                                          'message': f'{P.CMD_TIMEOUT:g}초 안에 응답이 없습니다 (공정/연결 노드 확인)'})

    # --- GUI 스레드에서 호출 ---
    def send_cmd(self, cmd, **kwargs):
        """명령을 서비스(/cushion/command)로 보낸다. 응답은 cmd_result 시그널로. 서비스가 없으면 토픽으로(응답 없음)."""
        import time
        d = {'cmd': cmd}
        d.update(kwargs)
        if self.cmd_cli is not None and self.cmd_cli.service_is_ready():
            req = self._Command.Request()
            req.command = P.encode(d)
            self._pending.append((self.cmd_cli.call_async(req), cmd, time.monotonic()))
            return True
        self.pub_cmd.publish(String(data=P.encode(d)))
        self.cmd_result.emit({'cmd': cmd, 'accepted': None, 'state': '',
                              'message': '명령 서비스가 없어 토픽으로 보냈습니다 (받았는지 확인 못 함)'})
        return False

    def estop_direct(self):
        """공정·연결 노드가 응답하지 않을 때: HMI 가 로봇 컨트롤러에 직접 빠른 정지 + 서보 끄기 (서비스)."""
        from dsr_msgs2.srv import MoveStop, ServoOff
        if not hasattr(self, '_estop_clis'):
            self._estop_clis = (self.node.create_client(MoveStop, P.ROBOT_SRV_PREFIX + 'motion/move_stop'),
                                self.node.create_client(ServoOff, P.ROBOT_SRV_PREFIX + 'system/servo_off'))
        stop_cli, off_cli = self._estop_clis
        sent = []
        if stop_cli.wait_for_service(timeout_sec=0.5):
            req = MoveStop.Request(); req.stop_mode = 0
            stop_cli.call_async(req); sent.append('빠른 정지')
        if off_cli.wait_for_service(timeout_sec=0.5):
            req = ServoOff.Request(); req.stop_type = 1
            off_cli.call_async(req); sent.append('서보 끄기')
        return sent

    def shutdown(self):
        self._running = False
        self.wait(1000)
        self.node.destroy_node()
