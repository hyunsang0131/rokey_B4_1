# robot_setting 공통 모듈
#
# 모든 스크립트가 같은 방식으로 로봇에 연결하도록 묶어 둔 것.
#   1) DR_init 에 로봇 ID/모델 설정
#   2) 노드 생성 후 DSR_ROBOT2 import (순서 중요: 노드가 먼저)
#   3) 사용할 서비스가 연결될 때까지 대기 (시간 제한 있음)
#
# DSR_ROBOT2 에 함수가 없는 서비스(servo_off, set_robot_control)는
# 여기서 직접 클라이언트를 만든다.

import sys
import time

import rclpy
from rclpy.utilities import remove_ros_args
import DR_init

ROBOT_ID = 'dsr01'
ROBOT_MODEL = 'm0609'
SRV_PREFIX = 'dsr_controller2/'  # DSR_ROBOT2 와 같은 접두사 (노드 namespace=dsr01 아래)

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# GetRobotState.srv 주석 기준
STATE_NAMES = {
    0: 'INITIALIZING', 1: 'STANDBY', 2: 'MOVING', 3: 'SAFE_OFF (서보 꺼짐)',
    4: 'TEACHING', 5: 'SAFE_STOP', 6: 'EMERGENCY_STOP', 7: 'HOMMING',
    8: 'RECOVERY', 9: 'SAFE_STOP2', 10: 'SAFE_OFF2', 15: 'NOT_READY',
}
STATE_STANDBY = 1
STATE_MOVING = 2
STATE_SAFE_OFF = 3
STATE_SAFE_STOP = 5
STATE_HOMMING = 7
# 동작 중 이 상태가 되면 더 기다려도 소용없음 → 바로 중단하고 알람 출력
FAULT_STATES = (3, 5, 6, 9, 10)  # SAFE_OFF, SAFE_STOP, EMERGENCY_STOP, SAFE_STOP2, SAFE_OFF2

# LOG_GROUP (DRFC.h)
ALARM_GROUPS = {1: 'SYSTEM', 2: 'MOTION', 3: 'TP', 4: 'INVERTER', 5: 'SAFETY_CONTROLLER'}
ALARM_LEVELS = {1: 'INFO', 2: 'WARN', 3: 'ERROR'}

# GetRobotMode.srv 주석 기준
MODE_NAMES = {0: 'MANUAL (수동)', 1: 'AUTONOMOUS (자동)', 2: 'MEASURE'}

# SetRobotControl.srv 주석 기준
CONTROL_RESET_SAFE_STOP = 2  # SAFE_STOP -> STANDBY
CONTROL_RESET_SAFE_OFF = 3   # SAFE_OFF  -> STANDBY (서보 켜기)

# ServoOff.srv 주석 기준
STOP_TYPE_QUICK = 1


class Robot:
    """로봇 연결과 자주 쓰는 호출을 묶은 클래스."""

    def __init__(self, node_name):
        rclpy.init(args=sys.argv)
        self.node = rclpy.create_node(node_name, namespace=ROBOT_ID)
        # 클래스 안에서 DR_init.__dsr__node 라고 쓰면 파이썬 이름 맹글링으로
        # DR_init._Robot__dsr__node 에 들어가 버린다 → setattr 로 정확한 이름에 넣는다.
        setattr(DR_init, '__dsr__node', self.node)
        self.log = self.node.get_logger()

        # 노드를 만든 뒤 import 해야 한다 (import 시점에 서비스 클라이언트를 만든다)
        import DSR_ROBOT2
        from dsr_msgs2.srv import ServoOff, SetRobotControl
        self.dr = DSR_ROBOT2

        self._srv_servo_off = self.node.create_client(
            ServoOff, SRV_PREFIX + 'system/servo_off')
        self._srv_robot_control = self.node.create_client(
            SetRobotControl, SRV_PREFIX + 'system/set_robot_control')
        self._ServoOff = ServoOff
        self._SetRobotControl = SetRobotControl

        # 사용자 인자 (ros 인자 제외). 예: ros2 run robot_setting home --mechanic
        self.args = remove_ros_args(sys.argv)[1:]

    # ------------------------------------------------------------------
    def wait_services(self, clients, timeout=10.0):
        """서비스 연결 대기. timeout 안에 안 되면 False (브링업이 안 떠 있는 경우)."""
        t0 = time.time()
        for cli in clients:
            while not cli.wait_for_service(timeout_sec=1.0):
                if time.time() - t0 > timeout:
                    self.log.error(
                        f'서비스 연결 실패: {cli.srv_name}\n'
                        '  → 브링업(m0609_rg2_bringup)이 실행 중인지 확인하세요.')
                    return False
                self.log.info(f'Waiting for service: {cli.srv_name}')
        return True

    def base_clients(self):
        """상태 조회에 필요한 기본 서비스 클라이언트 목록."""
        d = self.dr
        return [d._ros2_get_robot_state, d._ros2_get_robot_mode,
                d._ros2_get_last_alarm]

    # ------------------------------------------------------------------
    def state(self):
        return self.dr.get_robot_state()

    def state_name(self, s=None):
        s = self.state() if s is None else s
        return f'{s} {STATE_NAMES.get(s, "UNKNOWN")}'

    def mode(self):
        return self.dr.get_robot_mode()

    def wait_state(self, targets, timeout=30.0, period=0.2, abort_on_fault=False):
        """robot_state 가 targets 중 하나가 될 때까지 대기.

        성공 시 그 상태, 시간 초과 시 None.
        abort_on_fault=True 면 FAULT_STATES 가 되는 즉시 그 상태를 반환한다
        (호출한 쪽에서 `s in FAULT_STATES` 로 구분).
        """
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = self.state()
            if s in targets:
                return s
            if abort_on_fault and s in FAULT_STATES:
                return s
            time.sleep(period)
        return None

    def alarm_text(self):
        """마지막 알람을 읽어 사람이 읽을 수 있는 문자열로 반환."""
        a = self.dr.get_last_alarm()
        if a == -1 or a is None:
            return '알람 읽기 실패'
        # DRFC 의 OPERATION_* 상수에서 같은 번호의 이름을 찾는다.
        # 번호가 그룹마다 겹칠 수 있어서(예: 1020) 후보를 모두 보여준다.
        names = [n for n, v in vars(self.dr).items()
                 if n.startswith('OPERATION_') and v == a.index]
        params = ' '.join(p for p in a.param if p)
        return (f'[{ALARM_LEVELS.get(a.level, a.level)}] '
                f'group={a.group}({ALARM_GROUPS.get(a.group, "?")}) '
                f'index={a.index} {" / ".join(names) or "(이름 없음)"} {params}').rstrip()

    def ensure_autonomous(self):
        """모션 명령 전 AUTONOMOUS 모드로 전환. 실패 시 False."""
        if self.mode() == self.dr.ROBOT_MODE_AUTONOMOUS:
            return True
        self.log.info('MANUAL → AUTONOMOUS 모드로 전환')
        return self.dr.set_robot_mode(self.dr.ROBOT_MODE_AUTONOMOUS) == 0

    # ------------------------------------------------------------------
    def _call(self, client, req):
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        res = future.result()
        return res is not None and res.success

    def servo_off(self, stop_type=STOP_TYPE_QUICK):
        req = self._ServoOff.Request()
        req.stop_type = stop_type
        return self._call(self._srv_servo_off, req)

    def robot_control(self, control):
        req = self._SetRobotControl.Request()
        req.robot_control = control
        return self._call(self._srv_robot_control, req)

    # ------------------------------------------------------------------
    def shutdown(self):
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
