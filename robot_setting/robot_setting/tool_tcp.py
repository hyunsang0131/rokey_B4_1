# 툴(무게/무게중심) + TCP(툴 끝점 오프셋) 설정
#
# 툴과 TCP 는 실제로 함께 바꾸는 경우가 많고(그리퍼/압자 교체),
# 둘 다 "MANUAL 모드로 전환 → 설정 → 원래 모드 복귀" 흐름이 같아서 한 실행 파일로 묶었다.
#
#   ros2 run robot_setting tool_tcp show
#       현재 툴/TCP 이름 출력 (모드 안 바꿈)
#
#   ros2 run robot_setting tool_tcp set --tool "Tool Weight_1" --tcp GripperDA_v1
#       등록된 툴/TCP 중에서 사용할 것 선택 (둘 중 하나만 줘도 됨)
#
#   ros2 run robot_setting tool_tcp add-tool 이름 무게kg cx cy cz [--inertia 6개] [--use]
#       툴 등록. cog(무게중심)는 플랜지 기준 mm. --use 면 등록 후 바로 선택
#
#   ros2 run robot_setting tool_tcp add-tcp 이름 x y z a b c [--use]
#       TCP 등록. 플랜지 기준 위치(mm)와 방향(deg). --use 면 등록 후 바로 선택
#
#   ros2 run robot_setting tool_tcp del-tool 이름
#   ros2 run robot_setting tool_tcp del-tcp 이름
#
# 주의
#   - 툴 무게/무게중심이 실제와 다르면 get_tool_force 값과 직접 교시가 틀어진다.
#   - 두산 ROS 2 드라이버에는 "등록된 툴/TCP 목록"을 읽는 함수가 없다. 목록은 펜던트에서 확인.
#   - 에뮬레이터는 등록 안 된 이름도 성공으로 받아준다. 실행 후 출력되는
#     "현재 툴/TCP" 가 의도한 이름인지, 실물에서는 펜던트에서도 한 번 확인할 것.

import argparse

from robot_setting.common import (
    MODE_NAMES, STATE_HOMMING, STATE_MOVING, Robot,
)


def build_parser():
    p = argparse.ArgumentParser(prog='tool_tcp', description='툴/TCP 설정')
    sub = p.add_subparsers(dest='cmd', required=True)

    sub.add_parser('show', help='현재 툴/TCP 출력')

    s = sub.add_parser('set', help='등록된 툴/TCP 선택')
    s.add_argument('--tool')
    s.add_argument('--tcp')

    t = sub.add_parser('add-tool', help='툴 등록')
    t.add_argument('name')
    t.add_argument('weight', type=float, help='무게 [kg]')
    # metavar 에 튜플을 쓰면 인자 누락 시 argparse 가 오류 메시지를 만들다 죽는다 → 문자열로
    t.add_argument('cog', type=float, nargs=3, metavar='COG',
                   help='무게중심 cx cy cz, 플랜지 기준 [mm]')
    t.add_argument('--inertia', type=float, nargs=6, default=[0.0] * 6,
                   metavar='I', help='관성 6개 (모르면 생략 → 0)')
    t.add_argument('--use', action='store_true', help='등록 후 바로 선택')

    c = sub.add_parser('add-tcp', help='TCP 등록')
    c.add_argument('name')
    c.add_argument('pos', type=float, nargs=6, metavar='POS',
                   help='x y z a b c, 플랜지 기준 위치 [mm] + 방향 [deg]')
    c.add_argument('--use', action='store_true', help='등록 후 바로 선택')

    for name in ('del-tool', 'del-tcp'):
        x = sub.add_parser(name)
        x.add_argument('name')
    return p


def print_current(robot):
    d = robot.dr
    print(f'현재 툴: {d.get_tool() or "(없음)"}')
    print(f'현재 TCP: {d.get_tcp() or "(없음 → 플랜지 기준)"}')


def run(robot, args):
    """설정 명령 실행. 성공하면 True."""
    d = robot.dr
    ok = True

    def check(ret, what):
        nonlocal ok
        if ret == 0:
            robot.log.info(f'{what} 성공')
        else:
            robot.log.error(f'{what} 실패 (이름 오타/미등록/사용 중인 항목인지 확인)')
            ok = False
        return ret == 0

    if args.cmd == 'set':
        if args.tool is not None:
            check(d.set_tool(args.tool), f"set_tool('{args.tool}')")
        if args.tcp is not None:
            check(d.set_tcp(args.tcp), f"set_tcp('{args.tcp}')")

    elif args.cmd == 'add-tool':
        if check(d.add_tool(args.name, args.weight, args.cog, args.inertia),
                 f"add_tool('{args.name}', {args.weight} kg, cog={args.cog})") and args.use:
            check(d.set_tool(args.name), f"set_tool('{args.name}')")

    elif args.cmd == 'add-tcp':
        if check(d.add_tcp(args.name, args.pos),
                 f"add_tcp('{args.name}', {args.pos})") and args.use:
            check(d.set_tcp(args.name), f"set_tcp('{args.name}')")

    elif args.cmd == 'del-tool':
        check(d.del_tool(args.name), f"del_tool('{args.name}')")

    elif args.cmd == 'del-tcp':
        check(d.del_tcp(args.name), f"del_tcp('{args.name}')")

    return ok


def main():
    robot = Robot('robot_setting_tool_tcp')
    parser = build_parser()
    try:
        args = parser.parse_args(robot.args)
        # 로봇에 연결/모드 변경 전에 인자부터 검사
        if args.cmd == 'set' and args.tool is None and args.tcp is None:
            parser.error('set 에는 --tool 또는 --tcp 중 하나가 필요합니다.')
    except SystemExit:
        robot.shutdown()
        return

    d = robot.dr
    clients = robot.base_clients() + [
        d._ros2_set_robot_mode, d._ros2_get_current_tool, d._ros2_get_current_tcp,
        d._ros2_set_current_tool, d._ros2_set_current_tcp,
        d._ros2_config_create_tool, d._ros2_config_create_tcp,
        d._ros2_config_delete_tool, d._ros2_config_delete_tcp]
    if not robot.wait_services(clients):
        robot.shutdown()
        return

    orig_mode = robot.mode()
    try:
        if args.cmd == 'show':
            print_current(robot)
            return

        s = robot.state()
        if s in (STATE_MOVING, STATE_HOMMING):
            robot.log.error(f'로봇이 움직이는 중입니다 ({robot.state_name(s)}). 멈춘 뒤 실행하세요.')
            return

        # 툴/TCP 설정은 MANUAL 모드에서만 허용된다
        if orig_mode != d.ROBOT_MODE_MANUAL:
            robot.log.info(f'{MODE_NAMES.get(orig_mode)} → MANUAL 모드로 전환')
            if d.set_robot_mode(d.ROBOT_MODE_MANUAL) != 0:
                robot.log.error('MANUAL 모드 전환 실패')
                return

        run(robot, args)
        print_current(robot)

    except KeyboardInterrupt:
        pass
    finally:
        if robot.mode() != orig_mode:
            robot.log.info(f'원래 모드({MODE_NAMES.get(orig_mode)})로 복귀')
            d.set_robot_mode(orig_mode)
        robot.shutdown()


if __name__ == '__main__':
    main()
