# 침대 칸의 큐브를 교체 트레이 같은 번호 칸으로 옮기기 (그리퍼 실제 동작)
#
#   ros2 run pjt_test1 bring_test                    # ~/ws_cobot_pjt/teach_points/all_coordinate.json
#   ros2 run pjt_test1 bring_test 파일경로.json
#   ros2 run pjt_test1 bring_test --close            # 잡을 때 DO1 만 (닫기, 10 mm 5 N). 기본은 너비 잡기(DO1+DO2)
#   ros2 run pjt_test1 bring_test --plan             # 로봇 없이 동작 순서만 출력
#
# 점 이름 (all_coordinate.json)
#   침대  pin_b{n}, pin_b{n}_down, pin_b{n}_up      (n = 1~6)
#   트레이 tray_b{n}, tray_b{n}_down, tray_b{n}_up
#
# 한 칸 옮기기 = 출발 칸 위 → 열기 → 내려가기 → 잡기 → 올라오기 → 도착 칸 위 → 내려가기 → 잠깐 멈춤 → 놓기 → 올라오기
#
# 명령
#   1~6     그 번호 칸: 침대 → 트레이
#   r1~r6   그 번호 칸: 트레이 → 침대
#   a       전체 실행:      침대 → 트레이, 1 → 6
#   r       역순 전체 실행: 트레이 → 침대, 6 → 1 (전체 실행을 되돌린다)
#   l       점 목록 | q 종료
#
# 모든 점은 posx 로 직선 이동(movel)한다 → 현재 TCP 가 파일의 tcp(GripperDA_v1)와 같아야 한다.
# 주의: 잡는 너비는 WebLogic 의 IN1=1,IN2=1 프로그램 값이다. 이 파일의 자세는 큐브 넓은 쪽(64 mm 변)을 잡는다.
#       WebLogic: 닫기 0 mm 5 N / 열기 66 mm 10 N / 잡기 60 mm 20 N
# 실행 전 확인: 서보 ON(STANDBY), 경로에 부딪힐 물체 없음, 비상정지 손에 들기

import json
import os
import sys
import time

import rclpy
from rclpy.utilities import remove_ros_args
import DR_init

from pjt_test1.gripper import Gripper

ROBOT_ID = 'dsr01'
ROBOT_MODEL = 'm0609'
VEL_X, ACC_X = 100, 100        # 칸 사이 이동 [mm/s, mm/s^2]
VEL_Z, ACC_Z = 30, 30          # 칸 안으로 내려가고 올라올 때
SETTLE = 0.3                   # 내려간 뒤 놓기 전 대기 [s] (매달린 큐브 흔들림 가라앉히기)
CELLS = range(1, 7)
DEFAULT_FILE = os.path.expanduser('~/ws_cobot_pjt/teach_points/all_coordinate.json')
STATE_STANDBY = 1

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

SIDE = {'bed': 'pin_b', 'tray': 'tray_b'}
SIDE_KO = {'bed': '침대', 'tray': '트레이'}


def fmt(v):
    return '[' + ', '.join(f'{x:.2f}' for x in v) + ']'


def transfer_steps(n, src, dst):
    """한 칸 옮기기 동작 목록. ('move', 점 이름, 느림?) / ('open',) / ('grip',) / ('sleep', s)"""
    s, d = SIDE[src] + str(n), SIDE[dst] + str(n)
    return [('move', s, False), ('open',), ('move', s + '_down', True), ('grip',),
            ('move', s + '_up', True),
            ('move', d, False), ('move', d + '_down', True), ('sleep', SETTLE), ('open',),
            ('move', d + '_up', True)]


def job(cells, src, dst):
    return [(n, src, dst) for n in cells]


def print_plan(jobs):
    for n, src, dst in jobs:
        print(f'--- {n}번: {SIDE_KO[src]} → {SIDE_KO[dst]}')
        for st in transfer_steps(n, src, dst):
            print('   ', ' '.join(str(x) for x in st))


def main(args=None):
    user_args = remove_ros_args(sys.argv)[1:]
    use_close = '--close' in user_args
    files = [a for a in user_args if a.endswith('.json')]
    path = os.path.expanduser(files[0]) if files else DEFAULT_FILE
    try:
        with open(path) as f:
            pts = {p['name']: p for p in json.load(f)}
    except (OSError, ValueError) as e:
        print(f'파일을 읽을 수 없습니다: {path} ({e})')
        return

    need = {st[1] for n in CELLS for st in transfer_steps(n, 'bed', 'tray') if st[0] == 'move'}
    missing = sorted(need - pts.keys())
    if missing:
        print(f'{os.path.basename(path)} 에 없는 점: {missing}')
        return
    tcps = {pts[n].get('tcp', '') for n in need}

    if '--plan' in user_args:
        print('[전체 실행]')
        print_plan(job(CELLS, 'bed', 'tray'))
        print('[역순 전체 실행]')
        print_plan(job(reversed(CELLS), 'tray', 'bed'))
        return

    rclpy.init(args=args)
    node = rclpy.create_node('pjt_bring_test', namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    log = node.get_logger()

    try:
        import DSR_ROBOT2
        from DSR_ROBOT2 import (DR_BASE, ROBOT_MODE_AUTONOMOUS, get_current_posx, get_last_alarm,
                                get_robot_mode, get_robot_state, get_tcp, movel, set_robot_mode)
        from DR_common2 import posx
    except ImportError as e:
        log.error(f'Error importing DSR_ROBOT2 : {e}')
        rclpy.shutdown()
        return

    for cli in (DSR_ROBOT2._ros2_get_robot_state, DSR_ROBOT2._ros2_get_robot_mode,
                DSR_ROBOT2._ros2_set_robot_mode, DSR_ROBOT2._ros2_get_last_alarm,
                DSR_ROBOT2._ros2_get_current_posx, DSR_ROBOT2._ros2_get_current_tcp,
                DSR_ROBOT2._ros2_movel):
        while not cli.wait_for_service(timeout_sec=1.0):
            log.info(f'Waiting for service: {cli.srv_name}')

    def report_fault(msg):
        log.error(f'{msg}. 상태: {get_robot_state()}')
        a = get_last_alarm()
        if a != -1:
            log.error(f'마지막 알람: level={a.level} group={a.group} index={a.index}')
        log.error('→ 원인 확인 후 robot_setting servo_on 으로 복구하세요.')

    def move(name, slow):
        if get_robot_state() != STATE_STANDBY:
            report_fault('STANDBY 가 아니라 이동할 수 없습니다')
            return False
        target = pts[name]['posx']
        v, a = (VEL_Z, ACC_Z) if slow else (VEL_X, ACC_X)
        log.info(f'[{name}] movel → {fmt(target)}')
        if movel(posx(target), vel=v, acc=a, ref=DR_BASE) != 0:
            report_fault(f'[{name}] 이동 실패')
            return False
        cur, _ = get_current_posx(ref=DR_BASE)
        err = sum((c - t) ** 2 for c, t in zip(cur[:3], target[:3])) ** 0.5
        if err > 2.0:
            log.warn(f'[{name}] 위치 오차 {err:.2f} mm')
        return True

    def run(jobs):
        for n, src, dst in jobs:
            log.info(f'=== {n}번 칸: {SIDE_KO[src]} → {SIDE_KO[dst]} ===')
            for st in transfer_steps(n, src, dst):
                if st[0] == 'move':
                    ok = move(st[1], st[2])
                elif st[0] == 'open':
                    ok = g.open()
                elif st[0] == 'grip':
                    ok = g.close() if use_close else g.grip()
                else:
                    time.sleep(st[1])
                    ok = True
                if not ok:
                    log.error(f'{n}번 칸에서 멈춤 (단계: {" ".join(map(str, st))})')
                    return False
        log.info('완료')
        return True

    try:
        g = Gripper(node, use_do=True)
        if get_robot_state() != STATE_STANDBY:
            report_fault('STANDBY 가 아닙니다. 먼저 servo_on 을 실행하세요')
            return
        tcp = get_tcp()
        if tcps != {tcp}:
            log.error(f'현재 TCP({tcp or "없음"})와 파일의 TCP{sorted(tcps)} 가 다릅니다. TCP 를 맞추세요.')
            return
        if get_robot_mode() != ROBOT_MODE_AUTONOMOUS:
            log.info('MANUAL → AUTONOMOUS 모드로 전환')
            if set_robot_mode(ROBOT_MODE_AUTONOMOUS) != 0:
                log.error('AUTONOMOUS 모드 전환 실패')
                return

        print(f'\n=== bring_test ({os.path.basename(path)}, 잡기: {"닫기 DO1" if use_close else "너비 잡기 DO1+DO2"}) ===')
        help_txt = ('1~6: 침대→트레이 | r1~r6: 트레이→침대 | a: 전체 실행 (침대→트레이, 1→6)\n'
                    'r: 역순 전체 실행 (트레이→침대, 6→1) | l: 점 목록 | q: 종료')
        print(help_txt)
        while True:
            sel = input('> ').strip().lower()
            if sel == 'q':
                break
            if sel == 'l':
                for name, p in pts.items():
                    print(f'  {name:14s} {fmt(p["posx"])}')
            elif sel == 'a':
                run(job(CELLS, 'bed', 'tray'))
            elif sel == 'r':
                run(job(reversed(CELLS), 'tray', 'bed'))
            elif sel.isdigit() and int(sel) in CELLS:
                run(job([int(sel)], 'bed', 'tray'))
            elif sel[:1] == 'r' and sel[1:].isdigit() and int(sel[1:]) in CELLS:
                run(job([int(sel[1:])], 'tray', 'bed'))
            else:
                print(help_txt)

    except RuntimeError as e:
        log.error(str(e))
    except (KeyboardInterrupt, EOFError):
        print()
        log.warn('중단됨 (로봇이 아직 움직이면 펜던트에서 정지하세요)')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
