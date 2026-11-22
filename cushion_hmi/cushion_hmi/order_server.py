"""Forward browser mattress orders to the HMI through a local HTTP endpoint."""

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Thread

from cushion_hmi import protocol as P

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

HOST = '127.0.0.1'
PORT = 5000
MAX_BODY_SIZE = 16 * 1024
SLOT_NAMES = [f'Slot {i}' for i in range(1, 7)]


def validate_order_config(data):
    """Validate and normalize the JSON order emitted by bed_ordering.html."""
    if not isinstance(data, dict):
        raise ValueError('요청 본문은 JSON 객체여야 합니다')
    order_id = data.get('order_id')
    slots = data.get('slots')
    if not isinstance(order_id, str) or not order_id or len(order_id) > 64:
        raise ValueError('order_id가 없거나 형식이 올바르지 않습니다')
    slots = P.normalize_order_slots(slots)
    created_at = data.get('created_at')
    if created_at is not None and (
            not isinstance(created_at, str) or len(created_at) > 64):
        raise ValueError('created_at 형식이 올바르지 않습니다')
    bed_model = data.get('bed_model', '모델 정보 없음')
    size = data.get('size', '')
    if not isinstance(bed_model, str) or len(bed_model) > 64:
        raise ValueError('bed_model 형식이 올바르지 않습니다')
    if not isinstance(size, str) or len(size) > 32:
        raise ValueError('size 형식이 올바르지 않습니다')
    return {
        'order_id': order_id,
        'created_at': created_at,
        'bed_model': bed_model,
        'size': size,
        'slots': slots,
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


def create_order_server(node, publisher):
    """Create the local HTTP server that publishes browser orders to ROS."""
    return ThreadingHTTPServer(
        (HOST, PORT), make_handler(node, publisher))


def main(args=None):
    """Run the local HTTP server and publish validated orders to ROS 2."""
    rclpy.init(args=args)
    node = Node('cushion_order_server')
    publisher = node.create_publisher(String, P.TOPIC_ORDER_CONFIG, 10)
    server = create_order_server(node, publisher)
    server_thread = Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    node.get_logger().info(f'주문 서버 실행: http://{HOST}:{PORT}/order')
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
