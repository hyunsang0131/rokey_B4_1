# 직접 교시로 저장한 위치 파일을 골라서, 저장된 위치로 이동
#
#   ros2 run pjt_test1 move_to_teach                 # 파일 목록에서 선택
#   ros2 run pjt_test1 move_to_teach 파일경로.json    # 파일 바로 지정
#   ros2 run pjt_test1 move_to_teach --movel         # posx 로 직선 이동 (기본은 posj 로 movej)
#
# 파일 위치: ~/ws_cobot_pjt/teach_points/  (robot_setting direct_teach 가 저장하는 곳)
#
# 위치 선택 화면 명령
#   번호 + Enter   그 위치로 이동
#   a              저장된 순서대로 전부 이동
#   l              목록 다시 보기
#   q              종료
#
# 기본은 movej(posj): 관절 각도로 이동해서 TCP 설정과 상관없이 교시한 자세 그대로 간다.
# --movel 은 movel(posx): 툴 끝이 직선으로 움직인다. 단 posx 는 교시할 때의 TCP 기준이라
#   지금 TCP 가 다르면 엉뚱한 위치로 간다 → TCP 가 다르면 거부한다.
#
# 실행 전 확인: 서보 ON(STANDBY), 경로에 부딪힐 물체 없음, 비상정지 손에 들기

import glob
import json
import os
import sys
import time

import rclpy
from rclpy.utilities import remove_ros_args
import DR_init

ROBOT_ID = 'dsr01'
ROBOT_MODEL = 'm0609'
VEL, ACC = 30, 30        # movej [deg/s, deg/s^2]. 처음엔 낮게
VEL_X, ACC_X = 100, 100  # movel [mm/s, mm/s^2]
TEACH_DIR = os.path.expanduser('~/ws_cobot_pjt/teach_points')

STATE_STANDBY = 1
STATE_NAMES = {0: 'INITIALIZING', 1: 'STANDBY', 2: 'MOVING', 3: 'SAFE_OFF',
               4: 'TEACHING', 5: 'SAFE_STOP', 6: 'EMERGENCY_STOP', 7: 'HOMMING',
               8: 'RECOVERY', 9: 'SAFE_STOP2', 10: 'SAFE_OFF2', 15: 'NOT_READY'}

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def fmt(v):
    return '[' + ', '.join(f'{x:.2f}' for x in v) + ']'


def choose_file(args):
    """인자로 받은 파일, 없으면 목록에서 고르기. 취소하면 None."""
    files = [a for a in args if a.endswith('.json')]
    if files:
        return os.path.expanduser(files[0])

    paths = sorted(glob.glob(os.path.join(TEACH_DIR, '*.json')), reverse=True)  # 최신 먼저
    if not paths:
        print(f'교시 파일이 없습니다: {TEACH_DIR}')
        print('먼저 ros2 run robot_setting direct_teach 로 위치를 저장하세요.')
        return None

    print(f'\n=== 교시 파일 ({TEACH_DIR}) ===')
    for i, p in enumerate(paths, 1):
        try:
            with open(p) as f:
                n = len(json.load(f))
        except (OSError, ValueError):
            n = '?'
        print(f'  {i}. {os.path.basename(p)}  ({n}개)')

    while True:
        sel = input('파일 번호 (q: 종료): ').strip()
        if sel == 'q':
            return None
        if sel.isdigit() and 1 <= int(sel) <= len(paths):
            return paths[int(sel) - 1]
        print('  번호를 다시 입력하세요.')


def print_points(points):
    for i, p in enumerate(points, 1):
        print(f'  {i}. {p["name"]:10s} posj={fmt(p["posj"])}')
        print(f'     {"":10s} posx={fmt(p["posx"])}  (TCP: {p.get("tcp") or "없음"})')


def main(args=None):
    user_args = remove_ros_args(sys.argv)[1:]
    use_movel = '--movel' in user_args

    path = choose_file(user_args)
    if path is None:
        return
    try:
        with open(path) as f:
            points = json.load(f)
    except (OSError, ValueError) as e:
        print(f'파일을 읽을 수 없습니다: {path} ({e})')
        return
    if not points:
        print('저장된 위치가 없는 파일입니다.')
        return

    rclpy.init(args=args)
    node = rclpy.create_node('pjt_move_to_teach', namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    log = node.get_logger()

    try:
        import DSR_ROBOT2
        from DSR_ROBOT2 import (
            DR_BASE, ROBOT_MODE_AUTONOMOUS,
            get_current_posj, get_current_posx, get_last_alarm, get_robot_mode, get_robot_state,
            get_tcp, movej, movel, set_robot_mode,
        )
        from DR_common2 import posj, posx
    except ImportError as e:
        log.error(f'Error importing DSR_ROBOT2 : {e}')
        rclpy.shutdown()
        return

    for cli in (DSR_ROBOT2._ros2_get_robot_state, DSR_ROBOT2._ros2_get_robot_mode,
                DSR_ROBOT2._ros2_set_robot_mode, DSR_ROBOT2._ros2_get_last_alarm,
                DSR_ROBOT2._ros2_get_current_posj, DSR_ROBOT2._ros2_get_current_posx,
                DSR_ROBOT2._ros2_get_current_tcp,
                DSR_ROBOT2._ros2_movej, DSR_ROBOT2._ros2_movel):
        while not cli.wait_for_service(timeout_sec=1.0):
            log.info(f'Waiting for service: {cli.srv_name}')

    def state_str():
        s = get_robot_state()
        return f'{s} {STATE_NAMES.get(s, "UNKNOWN")}'

    def report_fault(msg):
        log.error(f'{msg}. 상태: {state_str()}')
        a = get_last_alarm()
        if a != -1:
            log.error(f'마지막 알람: level={a.level} group={a.group} index={a.index}')
        log.error('→ 원인 확인 후 robot_setting servo_on 으로 복구하세요.')

    def move_to(p):
        """한 위치로 이동. 성공하면 True."""
        if get_robot_state() != STATE_STANDBY:
            report_fault('STANDBY 가 아니라 이동할 수 없습니다')
            return False
        if use_movel:
            log.info(f'[{p["name"]}] movel → posx={fmt(p["posx"])}')
            ret = movel(posx(p['posx']), vel=VEL_X, acc=ACC_X, ref=DR_BASE)
        else:
            log.info(f'[{p["name"]}] movej → posj={fmt(p["posj"])}')
            ret = movej(posj(p['posj']), vel=VEL, acc=ACC)
        if ret != 0:
            report_fault(f'[{p["name"]}] 이동 실패')
            return False
        # 도착 확인: movej 는 관절각, movel 은 툴 끝 위치로 비교
        if use_movel:
            cur, _ = get_current_posx(ref=DR_BASE)
            err = sum((c - t) ** 2 for c, t in zip(cur[:3], p['posx'][:3])) ** 0.5
            log.info(f'[{p["name"]}] 도착. 위치 오차 {err:.2f} mm')
        else:
            cur = get_current_posj()
            err = max(abs(c - t) for c, t in zip(cur, p['posj']))
            log.info(f'[{p["name"]}] 도착. 관절 오차 최대 {err:.2f} deg')
        return True

    try:
        if get_robot_state() != STATE_STANDBY:
            report_fault('STANDBY 가 아닙니다. 먼저 servo_on 을 실행하세요')
            return

        if use_movel:
            tcp = get_tcp()
            bad = [p['name'] for p in points if p.get('tcp', '') != tcp]
            if bad:
                log.error(f'현재 TCP({tcp or "없음"})와 교시할 때 TCP 가 다른 위치: {bad}')
                log.error('--movel 은 TCP 가 같아야 합니다. --movel 없이(movej) 실행하세요.')
                return

        if get_robot_mode() != ROBOT_MODE_AUTONOMOUS:
            log.info('MANUAL → AUTONOMOUS 모드로 전환 (프로그램 동작에 필요)')
            if set_robot_mode(ROBOT_MODE_AUTONOMOUS) != 0:
                log.error('AUTONOMOUS 모드 전환 실패')
                return

        print(f'\n=== {os.path.basename(path)} ({"movel/posx" if use_movel else "movej/posj"}) ===')
        print_points(points)
        print('번호: 이동 | a: 전부 순서대로 | l: 목록 | q: 종료')

        while True:
            sel = input('> ').strip()
            if sel == 'q':
                break
            if sel == 'l':
                print_points(points)
            elif sel == 'a':
                for p in points:
                    if not move_to(p):
                        break
                    time.sleep(0.3)
            elif sel.isdigit() and 1 <= int(sel) <= len(points):
                move_to(points[int(sel) - 1])
            else:
                print('  번호, a, l, q 중에 입력하세요.')

    except (KeyboardInterrupt, EOFError):
        print()
        log.warn('중단됨 (로봇이 아직 움직이면 펜던트에서 정지하세요)')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
