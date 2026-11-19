# RG2 그리퍼 열기 / 닫기 / 너비 지정
#
#   ros2 run pjt_test1 gripper open                  # 열기
#   ros2 run pjt_test1 gripper close                 # 닫기
#   ros2 run pjt_test1 gripper grip                  # DO1·DO2 둘 다 ON → WebLogic 너비 잡기 (62 mm)
#   ros2 run pjt_test1 gripper                       # 대화형 (o / c / g / q)
#   ros2 run pjt_test1 gripper width 40 --modbus     # OnRobot 서비스로 너비 40 mm (아래 주의)
#
# 제어 방식
#   기본: 컨트롤러 디지털 출력 DO1/DO2 (강의 grip_test 방식)
#     - 닫기: DO1=ON, DO2=OFF / 열기: DO1=OFF, DO2=ON / 너비 잡기: DO1=ON, DO2=ON
#       (WebLogic: IN1=1,IN2=0 → 10 mm 5 N / IN1=0,IN2=1 → 70 mm 10 N / IN1=1,IN2=1 → 62 mm 20 N)
#     - 실물에서 동작 확인 (2026-11-15, 닫기 후 Compute Box 너비 0.7 mm)
#     - 열기/닫기만 가능, 너비 지정 불가
#   --modbus: /onrobot/sendCommand 서비스 (onrobot_rg_msgs/SetCommand)
#     - 실물: 브링업의 OnRobotRGControllerServer 가 Compute Box(192.168.1.1)로 Modbus 전송
#       → 이 셀에서는 명령이 전송돼도 그리퍼가 움직이지 않았음 (그리퍼가 DO 제어로 설정된 것으로 추정)
#     - 에뮬레이터: gripper_virtual_node 가 RViz 그리퍼만 움직임 (숫자는 rad 로 해석)
#     - 명령 문자열: 'o' 열기, 'c' 닫기, 숫자 = 너비(1/10 mm, RG2 최대 1100)
#   --do: 예전 옵션 이름. 지금은 기본이라 붙여도 같다.
#
# 주의: 그리퍼 사이에 손가락을 넣지 마세요. 닫기 힘은 드라이버 기본값(RG2 최대 40 N)이다.

import sys
import time

import rclpy
from rclpy.utilities import remove_ros_args
import DR_init

ROBOT_ID = 'dsr01'
ROBOT_MODEL = 'm0609'
SRV_NAME = '/onrobot/sendCommand'  # "/" 로 시작 → 절대 이름 (namespace 안 붙음)
RG2_MAX_WIDTH_MM = 110.0
MOVE_WAIT = 1.0  # 명령 후 동작 완료까지 대기 [s]

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

USAGE = '사용법: gripper [open | close | grip] | gripper width <mm> --modbus'


class Gripper:
    """OnRobot 서비스 또는 컨트롤러 DO 로 그리퍼 제어."""

    def __init__(self, node, use_do):
        self.node = node
        self.log = node.get_logger()
        self.use_do = use_do
        self.cli = None

        if use_do:
            # 클래스 안에서 DR_init.__dsr__node 라고 쓰면 파이썬 이름 맹글링으로
            # DR_init._Gripper__dsr__node 에 들어가 버린다 → setattr 로 정확한 이름에 넣는다.
            setattr(DR_init, '__dsr__node', node)
            import DSR_ROBOT2
            self.dr = DSR_ROBOT2
            while not DSR_ROBOT2._ros2_set_digital_output.wait_for_service(timeout_sec=1.0):
                self.log.info('Waiting for service: set_digital_output')
            self.log.info('컨트롤러 DO 로 제어')
        else:
            from onrobot_rg_msgs.srv import SetCommand
            self.SetCommand = SetCommand
            self.cli = node.create_client(SetCommand, SRV_NAME)
            if not self.cli.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(
                    f'{SRV_NAME} 서비스가 없습니다. 브링업(m0609_rg2_bringup)과 '
                    'ROS_DOMAIN_ID 를 확인하거나 --modbus 없이(DO) 실행하세요.')
            self.log.info(f'{SRV_NAME} 로 제어')

    def _send(self, cmd):
        req = self.SetCommand.Request()
        req.command = cmd
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        res = future.result()
        if res is None or not res.success:
            self.log.error(f"그리퍼 명령 '{cmd}' 실패")
            return False
        return True

    def _do(self, close, both=False):
        d = self.dr
        want = {1: 1 if close else 0, 2: 1 if (both or not close) else 0}
        ok = True
        # 항상 (0, 0) 을 먼저 거친다. WebLogic 은 입력이 '바뀌는 순간'에 동작을 실행하므로,
        # 이미 같은 값이 켜져 있으면(예: 닫기 → 닫기, WebLogic 설정을 바꾼 뒤) 다시 실행되지 않는다.
        # (0, 0) 은 WebLogic 에 동작이 없는 조합이라 거쳐도 안전하다.
        for idx in (1, 2):
            ok = d.set_digital_output(idx, 0) == 0 and ok
        time.sleep(0.1)
        for idx, val in want.items():
            if val:
                ok = d.set_digital_output(idx, val) == 0 and ok
        if not ok:
            self.log.error('set_digital_output 실패')
        return ok

    def open(self):
        ok = self._do(False) if self.use_do else self._send('o')
        if ok:
            time.sleep(MOVE_WAIT)
            self.log.info('그리퍼 열기')
        return ok

    def close(self):
        ok = self._do(True) if self.use_do else self._send('c')
        if ok:
            time.sleep(MOVE_WAIT)
            self.log.info('그리퍼 닫기')
        return ok

    def grip(self):
        if not self.use_do:
            self.log.error('grip 은 DO 방식 전용입니다 (--modbus 빼고 실행).')
            return False
        ok = self._do(True, both=True)
        if ok:
            time.sleep(MOVE_WAIT)
            self.log.info('그리퍼 너비 잡기 (DO1·DO2 ON, WebLogic 설정 너비)')
        return ok

    def width(self, mm):
        if self.use_do:
            self.log.error('DO 방식(기본)은 너비 지정이 안 됩니다. --modbus 를 붙이세요.')
            return False
        if not 0.0 <= mm <= RG2_MAX_WIDTH_MM:
            self.log.error(f'너비는 0 ~ {RG2_MAX_WIDTH_MM:.0f} mm 사이로 입력하세요.')
            return False
        ok = self._send(str(int(round(mm * 10))))  # 드라이버 단위: 1/10 mm
        if ok:
            time.sleep(MOVE_WAIT)
            self.log.info(f'그리퍼 너비 {mm:.1f} mm')
        return ok


def interactive(g):
    print('\n=== 그리퍼 ===')
    if g.use_do:
        print('o: 열기 | c: 닫기 | g: 너비 잡기 | q: 종료')
    else:
        print('o: 열기 | c: 닫기 | 숫자: 너비(mm, 0~110) | q: 종료')
    while True:
        sel = input('> ').strip().lower()
        if sel == 'q':
            return
        if sel == 'o':
            g.open()
        elif sel == 'c':
            g.close()
        elif sel == 'g' and g.use_do:
            g.grip()
        else:
            try:
                g.width(float(sel))
            except ValueError:
                print('  o, c, 숫자, q 중에 입력하세요.')


def main(args=None):
    user_args = remove_ros_args(sys.argv)[1:]
    use_do = '--modbus' not in user_args  # 기본은 DO. --do 는 예전 옵션이라 무시
    words = [a for a in user_args if a not in ('--do', '--modbus')]

    rclpy.init(args=args)
    node = rclpy.create_node('pjt_gripper', namespace=ROBOT_ID)
    try:
        g = Gripper(node, use_do)
        if not words:
            interactive(g)
        elif words[0] == 'open':
            g.open()
        elif words[0] == 'close':
            g.close()
        elif words[0] == 'grip':
            g.grip()
        elif words[0] == 'width' and len(words) == 2:
            try:
                g.width(float(words[1]))
            except ValueError:
                print(USAGE)
        else:
            print(USAGE)
    except RuntimeError as e:
        node.get_logger().error(str(e))
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
