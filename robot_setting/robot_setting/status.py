# 로봇 상태 조회 (로봇을 움직이지 않음)
#
#   ros2 run robot_setting status          # 한 번 출력
#   ros2 run robot_setting status --watch  # 1초마다 계속 출력 (Ctrl+C 종료)

import time

from robot_setting.common import MODE_NAMES, Robot


def print_status(robot):
    d = robot.dr
    posj = d.get_current_posj()
    posx, _ = d.get_current_posx(ref=d.DR_BASE)
    force = d.get_tool_force(ref=d.DR_BASE)
    mode = robot.mode()

    print('-' * 60)
    print(f'상태   : {robot.state_name()}')
    print(f'모드   : {mode} {MODE_NAMES.get(mode, "UNKNOWN")}')
    print(f'툴     : {d.get_tool()}    TCP: {d.get_tcp()}')
    print('관절   : [' + ' '.join(f'{v:7.2f}' for v in posj) + '] deg')
    print('TCP    : [' + ' '.join(f'{v:7.2f}' for v in posx) + '] mm/deg (BASE)')
    if force != -1:
        print('툴 힘  : [' + ' '.join(f'{v:6.2f}' for v in force) + '] N/Nm (BASE)')

    alarm = d.get_last_alarm()
    if alarm != -1:
        params = ' '.join(p for p in alarm.param if p)
        print(f'최근 알람: level={alarm.level} group={alarm.group} '
              f'index={alarm.index} {params}')


def main():
    robot = Robot('robot_setting_status')
    d = robot.dr
    clients = robot.base_clients() + [
        d._ros2_get_current_posj, d._ros2_get_current_posx,
        d._ros2_get_tool_force, d._ros2_get_current_tool,
        d._ros2_get_current_tcp, d._ros2_get_last_alarm,
    ]
    if not robot.wait_services(clients):
        robot.shutdown()
        return

    try:
        if '--watch' in robot.args:
            while True:
                print_status(robot)
                time.sleep(1.0)
        else:
            print_status(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.shutdown()


if __name__ == '__main__':
    main()
