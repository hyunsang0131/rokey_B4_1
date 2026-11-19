# 블록 1개씩 누르기 테스트: 공정과 똑같은 방법으로 누르고, 강성과 힘-변위 곡선을 확인한다.
# 처음 로봇에 올릴 때, 그리고 F_TARGET 같은 측정 설정을 정할 때 쓴다.
#
#   ros2 run bed_inspect press_test                       # 교시 파일 목록에서 선택
#   ros2 run bed_inspect press_test 파일경로.json          # 파일 바로 지정
#   ros2 run bed_inspect press_test --approach 0          # 교시 높이에서 바로 누르기 시작
#
# 교시 위치: 블록 중심 위, 툴이 아래를 보는 자세.
#   기본은 설정의 PRESS_APPROACH_DZ 만큼 빠르게 내려온 뒤 누르기 시작한다 (공정과 같음).
#   천 바로 위(5~10mm)에서 교시했다면 --approach 0 을 붙인다.
#
# 화면 명령
#   번호 + Enter   그 위치에서 누르기 측정
#   a              전부 측정
#   r              결과 표
#   q              결과 표 출력, CSV 저장 후 종료
#
# 실행 전 확인: 서보 ON(STANDBY), 비상정지 손에 들기, 그리퍼가 닫혀도 되는 상태인지.

import argparse
import statistics
import sys
import time

import rclpy
from rclpy.utilities import remove_ros_args
import DR_init

from bed_inspect import bed_config as cfg
from bed_inspect import bed_store

DR_init.__dsr__id = cfg.ROBOT_ID
DR_init.__dsr__model = cfg.ROBOT_MODEL

LINE = '  ' + '-' * 70


def print_results(results):
    if not results:
        print('  (측정 결과 없음)')
        return
    print(LINE)
    print(f'  {"이름":12s} {"강성 k":>11s} {"표면 높이":>10s} {"눌린 깊이":>9s} {"최대 힘":>8s} {"R²":>6s}  비고')
    for r in results:
        k = f'{r["k"]:.3f} N/mm' if r['k'] is not None else '-'
        zs = f'{r["z_surf"]:.2f} mm' if r['z_surf'] is not None else '-'
        d = f'{r["disp"]:.1f} mm' if r['disp'] is not None else '-'
        fm = f'{r["f_max"]:.1f} N' if r['f_max'] is not None else '-'
        r2 = f'{r["r2"]:.3f}' if r['r2'] is not None else '-'
        print(f'  {r["name"]:12s} {k:>11s} {zs:>10s} {d:>9s} {fm:>8s} {r2:>6s}  {r["note"]}')
    print(LINE)
    # 같은 위치를 여러 번 쟀으면 반복 정밀도를 보여준다
    by_name = {}
    for r in results:
        if r['k'] is not None:
            by_name.setdefault(r['name'], []).append(r['k'])
    for name, ks in by_name.items():
        if len(ks) >= 2:
            m = statistics.mean(ks)
            print(f'  {name}: {len(ks)}회 평균 {m:.3f} N/mm, 흩어짐 ±{statistics.stdev(ks) / m * 100:.1f}%')


def main(args=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('file', nargs='?')
    ap.add_argument('--approach', type=float, default=cfg.PRESS_APPROACH_DZ,
                    help='교시 높이에서 빠르게 내려오는 양 [mm] (음수 = 아래)')
    opt = ap.parse_args(remove_ros_args(sys.argv)[1:])
    path = bed_store.choose_teach_file([opt.file] if opt.file else [])
    if path is None:
        return
    bed_store.use_sim_data_if_sim(path)     # 가짜 로봇용 교시 파일이면 기록을 bed_data_sim 에 따로 저장
    try:
        points = list(bed_store.load_teach(path).values())
    except (OSError, ValueError, KeyError) as e:
        print(f'파일을 읽을 수 없습니다: {path} ({e})')
        return

    rclpy.init(args=args)
    node = rclpy.create_node('bed_press_test', namespace=cfg.ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        import DSR_ROBOT2 as dr
    except ImportError as e:
        node.get_logger().error(f'DSR_ROBOT2 를 불러오지 못했습니다: {e}')
        rclpy.shutdown()
        return
    from bed_inspect.bed_robot import BedError, BedRobot
    from bed_inspect.bed_tasks import BedTasks

    log = node.get_logger().info
    robot = BedRobot(dr, node, log)
    tasks = BedTasks(robot, {p['name']: p for p in points}, log)
    results = []
    try:
        robot.wait_services()
        tcp = robot.prepare()
        log(f'TCP: {tcp or "(없음)"}, 목표 힘 {cfg.F_TARGET:g}N, 접근 {opt.approach:g}mm')
        print(f'\n=== {path} ===')
        for i, p in enumerate(points, 1):
            print(f'  {i}. {p["name"]}')
        print('번호: 누르기 측정 | a: 전부 | r: 결과 | q: 종료(저장)')
        while True:
            sel = input('> ').strip()
            if sel == 'q':
                break
            if sel == 'r':
                print_results(results)
                continue
            if sel == 'a':
                todo = points
            elif sel.isdigit() and 1 <= int(sel) <= len(points):
                todo = [points[int(sel) - 1]]
            else:
                print('  번호, a, r, q 중에 입력하세요.')
                continue
            new = []
            for p in todo:
                res = tasks.press_at(p, opt.approach)
                res.update(name=p['name'], slot=p['name'], height=0.0, time=time.strftime('%H:%M:%S'))
                new.append(res)
            results += new
            print_results(new)
    except BedError as e:
        node.get_logger().error(str(e))
    except (KeyboardInterrupt, EOFError):
        print()
        node.get_logger().warn('중단됨 (로봇이 아직 움직이면 펜던트에서 정지하세요)')
    finally:
        if results:
            print('\n=== 측정 결과 ===')
            print_results(results)
            print('저장:', bed_store.save_measure_log('press_test', results, None))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
