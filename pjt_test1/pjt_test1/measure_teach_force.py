# 직접 교시로 저장한 위치마다 이동해서, TCP 에 작용하는 Z축 힘을 측정
#
#   ros2 run pjt_test1 measure_teach_force                # 파일 목록에서 선택
#   ros2 run pjt_test1 measure_teach_force 파일경로.json   # 파일 바로 지정
#
# 위치 선택 화면 명령
#   번호 + Enter   그 위치로 이동해서 측정
#   a              저장된 순서대로 전부 이동/측정
#   r              지금까지 결과 표 전체 보기 (차수별로 구분선)
#   a 나 번호 명령을 한 번 실행할 때마다 1차, 2차, ... 로 차수가 붙는다.
#   q              결과 표 출력, CSV 저장 후 종료
#
# 측정 방법
#   movej(posj) 로 이동 → SETTLE 초 대기(진동 가라앉기) → get_tool_force 를 SAMPLES 번 읽어 평균/표준편차
#   Fz(TCP) : TCP(툴) 좌표계 Z축 힘. 툴이 아래를 보면 툴 Z = 베이스 -Z 방향
#   Fz(BASE): 베이스 좌표계 Z축 힘 (참고용)
#
# 주의
#   - 툴 무게(set_tool)가 설정되어 있지 않으면 그리퍼 무게가 그대로 힘으로 잡힌다.
#     시작할 때 현재 툴 이름을 보여주고, 없으면 경고한다.
#   - 이 스크립트는 "그 위치에 멈춰 있을 때"의 힘만 잰다. 누르면서 재는 압입 측정은 별도.

import csv
import json
import os
import statistics
import sys
import time

import rclpy
from rclpy.utilities import remove_ros_args
import DR_init

from pjt_test1.move_to_teach import (
    ROBOT_ID, STATE_NAMES, STATE_STANDBY, choose_file, fmt, print_points,
)

VEL, ACC = 30, 30   # movej [deg/s, deg/s^2]
SETTLE = 0.5        # 도착 후 대기 [s]
SAMPLES = 20        # 힘 읽기 횟수
SAMPLE_DT = 0.05    # 읽기 간격 [s] (서비스 호출 시간이 더해져 실제로는 더 길다)
LOG_DIR = os.path.expanduser('~/ws_cobot_pjt/force_logs')


LINE = '  ' + '-' * 56


def print_results(results, only_round=None):
    """결과 표. 측정 차수(round)마다 구분선과 'N차' 제목을 붙인다."""
    rows = [r for r in results if only_round is None or r['round'] == only_round]
    if not rows:
        print('  (측정 결과 없음)')
        return
    cur = None
    for r in rows:
        if r['round'] != cur:
            cur = r['round']
            print(LINE)
            print(f'  [{cur}차]  {r["time"]}')
            print(f'  {"이름":14s} {"Fz(TCP) 평균":>13s} {"표준편차":>9s} {"Fz(BASE) 평균":>14s}')
        print(f'  {r["name"]:14s} {r["fz_tool"]:10.2f} N {r["fz_tool_std"]:7.2f} N'
              f' {r["fz_base"]:11.2f} N')
    print(LINE)


def main(args=None):
    path = choose_file(remove_ros_args(sys.argv)[1:])
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
    node = rclpy.create_node('pjt_measure_teach_force', namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    log = node.get_logger()

    try:
        import DSR_ROBOT2
        from DSR_ROBOT2 import (
            DR_BASE, DR_TOOL, ROBOT_MODE_AUTONOMOUS,
            get_last_alarm, get_robot_mode, get_robot_state, get_tcp, get_tool,
            get_tool_force, movej, set_robot_mode,
        )
        from DR_common2 import posj
    except ImportError as e:
        log.error(f'Error importing DSR_ROBOT2 : {e}')
        rclpy.shutdown()
        return

    for cli in (DSR_ROBOT2._ros2_get_robot_state, DSR_ROBOT2._ros2_get_robot_mode,
                DSR_ROBOT2._ros2_set_robot_mode, DSR_ROBOT2._ros2_get_last_alarm,
                DSR_ROBOT2._ros2_get_current_tool, DSR_ROBOT2._ros2_get_current_tcp,
                DSR_ROBOT2._ros2_get_tool_force, DSR_ROBOT2._ros2_movej):
        while not cli.wait_for_service(timeout_sec=1.0):
            log.info(f'Waiting for service: {cli.srv_name}')

    def report_fault(msg):
        s = get_robot_state()
        log.error(f'{msg}. 상태: {s} {STATE_NAMES.get(s, "UNKNOWN")}')
        a = get_last_alarm()
        if a != -1:
            log.error(f'마지막 알람: level={a.level} group={a.group} index={a.index}')
        log.error('→ 원인 확인 후 robot_setting servo_on 으로 복구하세요.')

    def measure(p):
        """한 위치로 이동해서 Fz 측정. 결과 dict, 실패하면 None."""
        if get_robot_state() != STATE_STANDBY:
            report_fault('STANDBY 가 아니라 이동할 수 없습니다')
            return None
        log.info(f'[{p["name"]}] movej → posj={fmt(p["posj"])}')
        if movej(posj(p['posj']), vel=VEL, acc=ACC) != 0:
            report_fault(f'[{p["name"]}] 이동 실패')
            return None

        time.sleep(SETTLE)
        fz_tool, fz_base = [], []
        for _ in range(SAMPLES):
            ft = get_tool_force(ref=DR_TOOL)
            fb = get_tool_force(ref=DR_BASE)
            if ft == -1 or fb == -1:
                log.warn('힘 읽기 실패, 건너뜀')
            else:
                fz_tool.append(ft[2])
                fz_base.append(fb[2])
            time.sleep(SAMPLE_DT)
        if len(fz_tool) < 2:
            log.error(f'[{p["name"]}] 힘을 충분히 읽지 못했습니다')
            return None

        r = {'name': p['name'],
             'fz_tool': statistics.mean(fz_tool),
             'fz_tool_std': statistics.stdev(fz_tool),
             'fz_base': statistics.mean(fz_base),
             'n': len(fz_tool),
             'posj': p['posj'],
             'time': time.strftime('%H:%M:%S')}
        log.info(f'[{p["name"]}] Fz(TCP) = {r["fz_tool"]:.2f} ± {r["fz_tool_std"]:.2f} N'
                 f'   Fz(BASE) = {r["fz_base"]:.2f} N   ({r["n"]}회 평균)')
        return r

    results = []
    round_no = 0  # 측정 차수: a 또는 번호 명령 한 번 = 1차
    try:
        if get_robot_state() != STATE_STANDBY:
            report_fault('STANDBY 가 아닙니다. 먼저 servo_on 을 실행하세요')
            return

        tool, tcp = get_tool(), get_tcp()
        log.info(f'현재 툴: {tool or "(없음)"}   TCP: {tcp or "(없음)"}')
        if not tool:
            log.warn('툴(무게)이 설정되어 있지 않습니다 → 그리퍼 무게가 힘에 그대로 포함됩니다. '
                     '(robot_setting tool_tcp set --tool ... 로 설정)')

        if get_robot_mode() != ROBOT_MODE_AUTONOMOUS:
            log.info('MANUAL → AUTONOMOUS 모드로 전환 (프로그램 동작에 필요)')
            if set_robot_mode(ROBOT_MODE_AUTONOMOUS) != 0:
                log.error('AUTONOMOUS 모드 전환 실패')
                return

        print(f'\n=== {os.path.basename(path)} : 위치별 Z축 힘 측정 ===')
        print_points(points)
        print('번호: 이동+측정 | a: 전부 | r: 결과 | q: 종료(저장)')

        while True:
            sel = input('> ').strip()
            if sel == 'q':
                break
            if sel == 'r':
                print_results(results)
            elif sel == 'a':
                round_no += 1
                print(LINE)
                print(f'  {round_no}차 측정 (전체 {len(points)}개)')
                for p in points:
                    r = measure(p)
                    if r is None:
                        break
                    r['round'] = round_no
                    results.append(r)
                print_results(results, only_round=round_no)  # 이번 차수만
            elif sel.isdigit() and 1 <= int(sel) <= len(points):
                round_no += 1
                print(LINE)
                print(f'  {round_no}차 측정 (1개)')
                r = measure(points[int(sel) - 1])
                if r is not None:
                    r['round'] = round_no
                    results.append(r)
                    print_results(results, only_round=round_no)
            else:
                print('  번호, a, r, q 중에 입력하세요.')

    except (KeyboardInterrupt, EOFError):
        print()
        log.warn('중단됨 (로봇이 아직 움직이면 펜던트에서 정지하세요)')
    finally:
        if results:
            print('\n=== 측정 결과 ===')
            print_results(results)
            os.makedirs(LOG_DIR, exist_ok=True)
            fname = os.path.join(LOG_DIR, time.strftime('teach_force_%Y%m%d_%H%M%S.csv'))
            with open(fname, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['round', 'name', 'fz_tool', 'fz_tool_std', 'fz_base', 'n', 'time',
                            'j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'teach_file'])
                for r in results:
                    w.writerow([r['round'], r['name'],
                                f'{r["fz_tool"]:.3f}', f'{r["fz_tool_std"]:.3f}',
                                f'{r["fz_base"]:.3f}', r['n'], r['time'], *r['posj'],
                                os.path.basename(path)])
            print(f'저장: {fname}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
