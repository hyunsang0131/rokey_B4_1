# 커버(뚜껑) 열기/닫기만 따로 돌려 보는 프로그램. 공정 4단계(열기)와 7단계(닫기)와 똑같은 코드(bed_tasks)를 쓴다.
# RViz 가상 모드(에뮬레이터)에서 좌표와 자세를 눈으로 확인할 때, 그리고 실물에서 커버만 시험할 때 쓴다.
#
#   ros2 launch m0609_rg2_bringup bringup.launch.py                # 가상 모드 (RViz + 에뮬레이터)
#   ros2 run bed_inspect cover_demo open                           # 열기 (공정 4단계)
#   ros2 run bed_inspect cover_demo close                          # 닫기 (공정 7단계). 열기와 닫기 사이에 교체 동작이 있어서 따로 실행
#   ros2 run bed_inspect cover_demo points                         # 좌표만 출력 (로봇 안 움직임)
#   ros2 run bed_inspect cover_demo open --no-compliance           # 순응 제어 없이 (외력 감시는 그대로)
#   ros2 run bed_inspect cover_demo open --virtual                 # RViz 가상 모드 (에뮬레이터는 TCP 가 없어서 TCP 확인을 경고로만)
#   ros2 run bed_inspect cover_demo open --file ~/ws_cobot_pjt/teach_points/bed.json   # 교시 파일의 home 사용
#
# 시작점/자세는 bed_config 의 COVER_GRIP_XYZ / COVER_GRIP_ORI. 실물에서는 한 손을 비상정지에 두세요.

import argparse
import sys

import rclpy
from rclpy.utilities import remove_ros_args
import DR_init

from bed_inspect import bed_config as cfg
from bed_inspect import bed_store
from bed_inspect.bed_tasks import make_cover_path

DR_init.__dsr__id = cfg.ROBOT_ID
DR_init.__dsr__model = cfg.ROBOT_MODEL

DEFAULT_HOME = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]


def print_points(p):
    rows = [('above', p['above']), ('grip', p['grip']), ('mid', p['mid']), ('via', p['via'])]
    if 'arc2_mid' in p:
        rows += [('arc2_mid', p['arc2_mid']), ('end', p['end'])]
    rows += [(f'line {i}', q) for i, q in enumerate(p['line'], 1)]
    rows += [(k, p[k]) for k in ('end_app', 'exit_out', 'exit_up', 'exit_over', 'rest_up', 'rest_app', 'rest', 'close_mid',
                                 'close_end') if k in p]
    print(f'  경첩 y {p["hinge"][1]:.2f} z {p["hinge"][2]:.2f}, 반지름 {p["radius"]:.1f}mm, '
          f'경첩 기준 각도: 경유(lift) {p["via_deg"] + p["grip_angle"]:.1f}도, 끝 {p["end_deg"] + p["grip_angle"]:.1f}도 '
          f'(시작점 {p["grip_angle"]:.1f}도)'
          + (f', 끝점이 원 밖으로 {p["end_out"]:.1f}mm' if 'end_out' in p else ''))
    for name, q in rows:
        print(f'  {name:10s} ' + ' '.join(f'{v:9.2f}' for v in q))
    if 'lid_tip' in p:
        print(f'  받침에 걸친 뚜껑 끝(길이 {p.get("lid_len", cfg.COVER_LID_LEN):g}mm 로 계산): y {p["lid_tip"][1]:.1f} z {p["lid_tip"][2]:.1f}')
    for w in p.get('warnings', []):
        print(f'  ※ 주의: {w}')


def main(args=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('action', choices=['open', 'close', 'points'], help='open: 열기, close: 닫기, points: 좌표만')
    ap.add_argument('--file', help='교시 파일 (home 위치를 여기서 가져옴)')
    ap.add_argument('--home', type=float, nargs=6, default=DEFAULT_HOME, help='home 관절각 (교시 파일이 없을 때)')
    ap.add_argument('--no-compliance', action='store_true', help='순응 제어 없이 (외력 감시는 그대로)')
    ap.add_argument('--virtual', action='store_true', help='RViz 가상 모드: TCP 가 달라도 경고만 하고 진행')
    ap.add_argument('--yes', action='store_true', help='시작 전에 Enter 를 기다리지 않음')
    opt = ap.parse_args(remove_ros_args(sys.argv)[1:])

    if opt.no_compliance:
        cfg.COVER_COMPLIANCE = False
    try:
        path = make_cover_path()
    except ValueError as e:
        print(f'커버 좌표 설정 오류: {e}')
        return
    print_points(path)
    if opt.action == 'points':
        return

    home = {'name': cfg.P_HOME, 'posj': list(opt.home)}
    if opt.file:
        pts = bed_store.load_teach(opt.file)
        home = pts[cfg.P_HOME] if cfg.P_HOME in pts else next(p for p in pts.values() if p['name'] == cfg.P_HOME)

    rclpy.init(args=args)
    node = rclpy.create_node('bed_cover_demo', namespace=cfg.ROBOT_ID)
    DR_init.__dsr__node = node
    import DSR_ROBOT2 as dr
    from bed_inspect.bed_robot import BedError, BedRobot
    from bed_inspect.bed_tasks import BedTasks

    log = node.get_logger().info
    robot = BedRobot(dr, node, log)
    tasks = BedTasks(robot, {cfg.P_HOME: home}, log)
    try:
        robot.wait_services()
        tcp = robot.prepare()
        log(f'TCP: {tcp or "(없음)"}, 그리퍼 모드 {cfg.GRIPPER_MODE}, home {home["posj"]}, '
            f'순응 제어 {"켬" if cfg.COVER_COMPLIANCE else "끔"}, 외력 한계 {cfg.COVER_F_LIMIT:g}N')
        if cfg.COVER_TCP is not None and cfg.COVER_TCP != tcp:
            msg = f'현재 TCP("{tcp or "없음"}")가 커버 좌표를 잴 때("{cfg.COVER_TCP}")와 다릅니다.'
            if not opt.virtual:                         # 실물: 다른 TCP 면 엉뚱한 자리로 가므로 막는다
                raise BedError(msg + ' 펜던트에서 TCP 를 바꾼 뒤 다시 실행하세요. (RViz 가상 모드면 --virtual)')
            node.get_logger().warn(msg + ' --virtual 이라 계속합니다 (좌표는 지금 TCP 기준으로 움직임).')
        if not opt.yes:
            input('Enter 를 누르면 커버 동작을 시작합니다 (Ctrl+C 취소) > ')
        if opt.action == 'open':
            log('=== 커버 열기 ===')
            tasks.cover_open()
            log(f'열기 끝. 현재 {[round(v, 1) for v in robot.cur_posx()]}')
        else:
            log('=== 커버 닫기 ===')
            tasks.cover_close()
            log(f'닫기 끝. 현재 {[round(v, 1) for v in robot.cur_posx()]}')
    except BedError as e:
        node.get_logger().error(str(e))
    except (KeyboardInterrupt, EOFError):
        print()
        node.get_logger().warn('중단됨 (로봇이 아직 움직이면 펜던트에서 정지하세요)')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
