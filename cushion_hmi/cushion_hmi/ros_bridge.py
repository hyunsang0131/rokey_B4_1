# ROS 2 <-> Qt 연결
#
# Qt 이벤트 루프(app.exec_)와 rclpy.spin 은 둘 다 무한 루프라 한 스레드에서 같이 못 돈다.
# 그래서 rclpy 는 QThread 에서 spin_once 를 반복하고, 받은 메시지는 Qt 시그널로 넘긴다.
# 시그널은 스레드를 건너면 자동으로 큐에 쌓였다가 GUI 스레드에서 처리되므로 안전하다.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtCore import QThread, pyqtSignal

from cushion_hmi import protocol as P


class RosBridge(QThread):
    status_received = pyqtSignal(dict)
    force_received = pyqtSignal(dict)
    log_received = pyqtSignal(str)
    order_config_received = pyqtSignal(dict)

    def __init__(self, node_name='cushion_hmi'):
        super().__init__()
        self._running = True
        self.node = Node(node_name)
        self.pub_cmd = self.node.create_publisher(String, P.TOPIC_CMD, 10)
        self.node.create_subscription(String, P.TOPIC_STATUS, self._on_status, 10)
        self.node.create_subscription(String, P.TOPIC_FORCE, self._on_force, 100)
        self.node.create_subscription(String, P.TOPIC_LOG, self._on_log, 50)
        self.node.create_subscription(
            String, P.TOPIC_ORDER_CONFIG, self._on_order_config, 10)

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

    def _on_order_config(self, msg):
        d = P.decode(msg.data)
        if d is not None:
            self.order_config_received.emit(d)

    def run(self):
        while self._running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # --- GUI 스레드에서 호출 ---
    def send_cmd(self, cmd, **kwargs):
        d = {'cmd': cmd}
        d.update(kwargs)
        self.pub_cmd.publish(String(data=P.encode(d)))

    def shutdown(self):
        self._running = False
        self.wait(1000)
        self.node.destroy_node()
