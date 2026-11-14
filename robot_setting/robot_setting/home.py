# 홈으로 이동
#
#   ros2 run robot_setting home              # 작업 홈 [0, 0, 90, 0, 90, 0] (강의 예제와 동일)
#   ros2 run robot_setting home --user       # 펜던트에 등록한 사용자 홈 (move_home USER)
#   ros2 run robot_setting home --mechanic   # 기계 원점 [0,0,0,0,0,0] (팔이 위로 똑바로 섬)
#
# 실행 전 확인: 서보 ON(STANDBY), 주변에 부딪힐 물체 없음, 비상정지 손에 들기

from robot_setting.common import (
    FAULT_STATES, STATE_HOMMING, STATE_MOVING, STATE_STANDBY, Robot,
)

WORK_HOME = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
VEL, ACC = 30, 30  # 처음엔 낮게. [deg/s, deg/s^2]


def report_fault(robot, msg, s=None):
    """정지 원인(상태 + 마지막 알람)을 출력."""
    robot.log.error(f'{msg}. 상태: {robot.state_name(s)}')
    robot.log.error(f'마지막 알람: {robot.alarm_text()}')
    robot.log.error('→ 원인 확인 후 servo_on 으로 복구하세요.')


def main():
    robot = Robot('robot_setting_home')
    d = robot.dr
    clients = robot.base_clients() + [
        d._ros2_set_robot_mode, d._ros2_movej, d._ros2_move_home]
    if not robot.wait_services(clients):
        robot.shutdown()
        return

    try:
        s = robot.state()
        if s in (STATE_HOMMING, STATE_MOVING):
            robot.log.error(f'로봇이 아직 움직이는 중입니다 ({robot.state_name(s)}). '
                            '끝난 뒤 다시 실행하세요.')
            return
        if s != STATE_STANDBY:
            robot.log.error(f'STANDBY 가 아닙니다 ({robot.state_name(s)}). '
                            '먼저 servo_on 을 실행하세요.')
            return
        if not robot.ensure_autonomous():
            robot.log.error('AUTONOMOUS 모드 전환 실패')
            return

        if '--mechanic' in robot.args or '--user' in robot.args:
            target = (d.DR_HOME_TARGET_MECHANIC if '--mechanic' in robot.args
                      else d.DR_HOME_TARGET_USER)
            name = 'MECHANIC' if '--mechanic' in robot.args else 'USER'
            robot.log.info(f'move_home({name}) 실행')
            robot.log.warn('실물에서 move_home 은 SOS 위반(7056)으로 서보가 꺼진 적이 있음. '
                           '비상정지를 손에 들고 실행하세요.')
            if d.move_home(target) != 0:
                report_fault(robot, 'move_home 실패')
                return
            # move_home 은 동작 시작 전에 바로 반환된다. 호출 직후에는 아직 STANDBY 라서
            # 바로 완료 확인을 하면 "완료"로 잘못 판단 → HOMMING 으로 바뀌는 것부터 기다린다.
            s = robot.wait_state([STATE_HOMMING, STATE_MOVING], timeout=3.0,
                                 abort_on_fault=True)
            if s in FAULT_STATES:
                report_fault(robot, '홈 이동 시작 직후 정지', s)
                return
            if s is None:
                robot.log.warn('HOMMING 상태로 바뀌지 않음 (이미 홈 위치일 수 있음)')
        else:
            robot.log.info(f'작업 홈 {WORK_HOME} 로 이동 (vel={VEL})')
            if d.movej(d.posj(WORK_HOME), vel=VEL, acc=ACC) != 0:
                report_fault(robot, 'movej 실패')
                return

        # 동작이 끝날 때까지 상태로 확인. 중간에 서보 오프/보호 정지가 되면 바로 중단.
        s = robot.wait_state([STATE_STANDBY], timeout=60.0, abort_on_fault=True)
        if s == STATE_STANDBY:
            robot.log.info('홈 이동 완료')
        elif s in FAULT_STATES:
            report_fault(robot, '홈 이동 중 정지', s)
        else:
            robot.log.error(f'홈 이동 완료 확인 실패(시간 초과). 현재 상태: {robot.state_name()}')
    except KeyboardInterrupt:
        robot.log.warn('중단됨 (로봇이 아직 움직이면 펜던트에서 정지하세요)')
    finally:
        robot.shutdown()


if __name__ == '__main__':
    main()
