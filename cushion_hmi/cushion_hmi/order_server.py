"""Forward browser mattress orders to the HMI through a local HTTP endpoint.

    ros2 run cushion_hmi order_server                                  # 0.0.0.0:5000 (다른 PC 에서도 접속)
    ros2 run cushion_hmi order_server --ros-args -p host:=127.0.0.1 -p port:=5000   # 이 PC 에서만

브라우저에서 http://<이 PC IP>:5000/ 을 열면 주문 페이지(bed_ordering.html)가 바로 뜨고,
그 페이지는 같은 주소의 /order 로 주문을 보낸다 (IP 를 HTML 에 적어 넣을 필요 없음).
다른 곳에 올린 HTML 을 쓸 때는 주소 뒤에 ?hmi=http://<이 PC IP>:5000/order 를 붙인다.
"""

import json
import os
import socket
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Thread

from cushion_hmi import protocol as P

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String

HOST = os.environ.get('CUSHION_ORDER_HOST', '0.0.0.0')     # 기본: 모든 네트워크에서 받음 (IP 가 달라도 됨)
PORT = int(os.environ.get('CUSHION_ORDER_PORT', '5000'))


def order_page_path():
    """설치된 bed_ordering.html 경로 (없으면 소스 폴더)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('cushion_hmi'), 'bed_ordering.html')
        if os.path.exists(p):
            return p
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'bed_ordering.html')


def local_ips():
    """이 PC 의 IP 들 (안내용)."""
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('10.255.255.255', 1))         # 실제로 보내지는 않는다 (경로만 정함)
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass
    return sorted(ip for ip in ips if not ip.startswith('127.')) or ['127.0.0.1']
MAX_BODY_SIZE = 16 * 1024
SLOT_NAMES = [f'Slot {i}' for i in range(1, 7)]


def validate_order_config(data):
    """Validate the JSON shape emitted by bed_ordering.html."""
    if not isinstance(data, dict):
        raise ValueError('요청 본문은 JSON 객체여야 합니다')
    order_id = data.get('order_id')
    slots = data.get('slots')
    if not isinstance(order_id, str) or not order_id or len(order_id) > 64:
        raise ValueError('order_id가 없거나 형식이 올바르지 않습니다')
    if not isinstance(slots, dict) or set(slots) != set(SLOT_NAMES):
        raise ValueError('slots에는 Slot 1부터 Slot 6까지 필요합니다')
    if any(
            not isinstance(value, str) or value not in P.STIFFNESS_K
            for value in slots.values()):
        raise ValueError('각 슬롯 값은 부드러움 또는 단단함이어야 합니다')
    created_at = data.get('created_at')
    if created_at is not None and (
            not isinstance(created_at, str) or len(created_at) > 64):
        raise ValueError('created_at 형식이 올바르지 않습니다')
    return {
        'order_id': order_id,
        'created_at': created_at,
        'slots': {name: slots[name] for name in SLOT_NAMES},
    }


def make_handler(node, publisher):
    """Create an HTTP request handler connected to the ROS publisher."""

    class OrderHandler(BaseHTTPRequestHandler):

        def _send(self, status, body):
            payload = b'' if status == 204 else json.dumps(
                body, ensure_ascii=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Methods', 'POST, OPTIONS')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type')
            self.end_headers()
            if payload:
                self.wfile.write(payload)

        def do_GET(self):
            if self.path.split('?')[0] not in ('/', '/bed_ordering.html'):
                self._send(404, {'error': 'not found'})
                return
            try:
                with open(order_page_path(), 'rb') as f:
                    page = f.read()
            except OSError as exc:
                self._send(500, {'error': f'주문 페이지를 찾을 수 없습니다: {exc}'})
                return
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(page)))
            self.end_headers()
            self.wfile.write(page)

        def do_OPTIONS(self):
            if self.path != '/order':
                self._send(404, {'error': 'not found'})
                return
            self._send(204, {})

        def do_POST(self):
            if self.path != '/order':
                self._send(404, {'error': 'not found'})
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if length <= 0 or length > MAX_BODY_SIZE:
                    raise ValueError('요청 본문 크기가 올바르지 않습니다')
                raw = self.rfile.read(length)
                config = validate_order_config(json.loads(raw.decode('utf-8')))
            except (UnicodeDecodeError, json.JSONDecodeError,
                    ValueError) as exc:
                node.get_logger().warning(f'주문 요청 거부: {exc}')
                self._send(400, {'error': str(exc)})
                return

            publisher.publish(String(data=P.encode(config)))
            node.get_logger().info(f'브라우저 주문 수신: {config["order_id"]}')
            self._send(200, {'ok': True, 'order_id': config['order_id']})

        def log_message(self, fmt, *args):
            node.get_logger().info('HTTP ' + (fmt % args))

    return OrderHandler


def main(args=None):
    """Run the local HTTP server and publish validated orders to ROS 2."""
    rclpy.init(args=args)
    node = Node('cushion_order_server')
    host = node.declare_parameter('host', HOST).value
    port = int(node.declare_parameter('port', PORT).value)
    # 마지막 주문을 남겨 둔다 (나중에 켠 bed_cmd order 도 받을 수 있게)
    latch = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(String, P.TOPIC_ORDER_CONFIG, latch)
    server = ThreadingHTTPServer((host, port), make_handler(node, publisher))
    server_thread = Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    node.get_logger().info(f'주문 서버 실행: {host}:{port}  (주문 받는 곳 /order)')
    shown = local_ips() if host in ('0.0.0.0', '') else [host]
    for ip in shown:
        node.get_logger().info(f'  주문 페이지: http://{ip}:{port}/   (다른 PC 브라우저에서 이 주소를 연다)')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server_thread.join()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
