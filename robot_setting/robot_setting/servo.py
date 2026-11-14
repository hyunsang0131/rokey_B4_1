# 서보 켜기 / 끄기
#
#   ros2 run robot_setting servo_on
#   ros2 run robot_setting servo_off
#
# 서보 켜기는 현재 상태에 따라 다른 명령을 보낸다.
#   SAFE_OFF  (서보 꺼짐)       → CONTROL_RESET_SAFE_OFF
#   SAFE_STOP (보호 정지, 충돌 등) → CONTROL_RESET_SAFE_STOP
# 컨트롤러가 명령을 받으면 항상 success 를 돌려주므로,
# 실제로 STANDBY 가 됐는지는 상태를 다시 읽어서 확인한다.

from robot_setting.common import (
    CONTROL_RESET_SAFE_OFF, CONTROL_RESET_SAFE_STOP,
    STATE_SAFE_OFF, STATE_SAFE_STOP, STATE_STANDBY, Robot,
)


def _connect(name):
    robot = Robot(name)
    clients = robot.base_clients() + [
        robot._srv_servo_off, robot._srv_robot_control]
    if not robot.wait_services(clients):
        robot.shutdown()
        return None
    return robot


def servo_on():
    robot = _connect('robot_setting_servo_on')
    if robot is None:
        return
    try:
        s = robot.state()
        robot.log.info(f'현재 상태: {robot.state_name(s)}')

        if s == STATE_STANDBY:
            robot.log.info('이미 서보가 켜져 있습니다.')
            return
        if s == STATE_SAFE_OFF:
            robot.robot_control(CONTROL_RESET_SAFE_OFF)
        elif s == STATE_SAFE_STOP:
            robot.robot_control(CONTROL_RESET_SAFE_STOP)
        else:
            robot.log.error('이 상태에서는 서보를 켤 수 없습니다. '
                            '비상정지 해제/알람 확인 후 펜던트에서 처리하세요.')
            return

        if robot.wait_state([STATE_STANDBY], timeout=10.0) is not None:
            robot.log.info('서보 ON 완료 (STANDBY)')
        else:
            robot.log.error(f'서보 ON 실패. 현재 상태: {robot.state_name()}')
    finally:
        robot.shutdown()


def servo_off():
    robot = _connect('robot_setting_servo_off')
    if robot is None:
        return
    try:
        robot.log.info(f'현재 상태: {robot.state_name()}')
        robot.servo_off()
        if robot.wait_state([STATE_SAFE_OFF], timeout=10.0) is not None:
            robot.log.info('서보 OFF 완료 (SAFE_OFF)')
        else:
            robot.log.error(f'서보 OFF 확인 실패. 현재 상태: {robot.state_name()}')
    finally:
        robot.shutdown()
