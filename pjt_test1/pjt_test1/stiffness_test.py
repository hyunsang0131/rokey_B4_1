# 침대 칸 강성 테스트: 정해진 칸 위로 가서, 공정(bed_inspect)과 똑같은 방법으로 누르고 강성을 보여준다.
#
#   ros2 run pjt_test1 stiffness_test                      # 교시 파일 목록에서 선택 (Enter = 기본 파일)
#   ros2 run pjt_test1 stiffness_test 파일경로.json         # 다른 교시 파일
#   ros2 run pjt_test1 stiffness_test --approach -30       # 교시 높이에서 빠르게 내려오는 양 [mm] 바꾸기
#   ros2 run pjt_test1 stiffness_test --target 15          # 목표 힘 [N] 바꾸기 (처음엔 15 권장, 12 미만은 거부)
#     단단한 큐브는 조금만 들어가서, 목표 힘이 낮으면 5N~목표 구간의 측정 점이 모자라 계산이 안 된다
#   ros2 run pjt_test1 stiffness_test --modbus             # 그리퍼를 /onrobot/sendCommand 로 (기본은 DO)
#
# 측정 방법 (bed_inspect 의 press_at 그대로)
#   1) 그리퍼를 닫고 칸 위(교시 높이)로 이동 → approach 만큼 빠르게 내려감
#   2) 0.3초 쉬고 힘 15번 평균 = 영점
#   3) 1mm 씩 내려가다 영점 대비 3N 넘으면 "닿음", 그 뒤 0.5mm 씩 목표 힘까지 누름
#   4) 5N ~ 목표 힘 구간을 직선으로 맞춰 기울기 = 강성 k [N/mm], 힘 0 지점 = 윗면 높이
#   5) 누른 채로 두지 않고 교시 높이로 올라옴
#
# 화면 명령
#   번호         그 칸 1번 측정             예) 3
#   번호*횟수    그 칸 여러 번 (반복 정밀도)  예) 3*5
#   a            전부 1번씩
#   n            지금 자리에서 힘 노이즈 측정 (아무것도 안 닿은 상태에서)
#   r            결과 표
#   q            결과 저장 후 종료
#
# 교시 파일: posj 가 있는 점은 movej, posx 만 있는 점(좌표로 계산한 점)은 movel 로 간다.
#   _down / _up 이 붙은 점은 무시한다 (위치 확인용 파일을 골라도 칸 위 점만 쓴다).
#
# 실행 전 확인: sodreal 연결, 서보 ON(STANDBY), 비상정지 손에 들기, 그리퍼가 닫혀도 되는 상태,
#              WebLogic 프로그램 RUN (DO 그리퍼일 때)

import argparse
import csv
import glob
import json
import os
import statistics
import sys
import time

import rclpy
from rclpy.utilities import remove_ros_args
import DR_init

from bed_inspect import bed_config as cfg

DR_init.__dsr__id = cfg.ROBOT_ID
DR_init.__dsr__model = cfg.ROBOT_MODEL

TEACH_DIR = os.path.expanduser('~/ws_cobot_pjt/teach_points')
DEFAULT_FILE = os.path.join(TEACH_DIR, 'teach_20261117_calc_bed6.json')
OUT_DIR = os.path.expanduser('~/ws_cobot_pjt/stiffness_test')
NOISE_SAMPLES = 50
LINE = '  ' + '-' * 78


def choose_file():
    """교시 폴더의 JSON 목록에서 고른다. Enter = 기본 파일, q = 취소(None)."""
    paths = sorted(glob.glob(os.path.join(TEACH_DIR, '*.json')), reverse=True)    # 이름 역순 = 최신 먼저
    if not paths:
        print(f'교시 파일이 없습니다: {TEACH_DIR}')
        return None
    default = DEFAULT_FILE if DEFAULT_FILE in paths else paths[0]
    print(f'\n=== 교시 파일 ({TEACH_DIR}) ===')
    for i, p in enumerate(paths, 1):
        try:
            pts = load_points(p)
            names = [q['name'] for q in pts]
            info = f'{len(pts)}칸: ' + ', '.join(names[:6]) + (' ...' if len(names) > 6 else '')
        except (OSError, ValueError, KeyError, TypeError):
            info = '읽을 수 없음'
        mark = '  ← 기본 (Enter)' if p == default else ''
        print(f'  {i:2d}. {os.path.basename(p)}  ({info}){mark}')
    while True:
        sel = input('파일 번호 (Enter: 기본, q: 종료): ').strip()
        if sel == 'q':
            return None
        if sel == '':
            return default
        if sel.isdigit() and 1 <= int(sel) <= len(paths):
            return paths[int(sel) - 1]
        print('  번호를 다시 입력하세요.')


def load_points(path):
    with open(path) as f:
        pts = json.load(f)
    return [p for p in pts if not p['name'].endswith(('_down', '_up'))]


def print_results(results):
    if not results:
        print('  (측정 결과 없음)')
        return
    ks = [r['k'] for r in results if r['k'] is not None]
    med = statistics.median(ks) if ks else None
    print(LINE)
    print(f'  {"칸":10s} {"강성 k":>12s} {"k/중앙값":>8s} {"윗면 높이":>10s} {"눌린 깊이":>9s} '
          f'{"최대 힘":>7s} {"R²":>6s}  비고')
    for r in results:
        k = f'{r["k"]:.3f} N/mm' if r['k'] is not None else '-'
        ratio = f'{r["k"] / med:.2f}' if r['k'] is not None and med else '-'
        zs = f'{r["z_surf"]:.2f} mm' if r['z_surf'] is not None else '-'
        d = f'{r["disp"]:.1f} mm' if r['disp'] is not None else '-'
        fm = f'{r["f_max"]:.1f} N' if r['f_max'] is not None else '-'
        r2 = f'{r["r2"]:.3f}' if r['r2'] is not None else '-'
        note = r['note'] or (r['error'] or '')
        print(f'  {r["name"]:10s} {k:>12s} {ratio:>8s} {zs:>10s} {d:>9s} {fm:>7s} {r2:>6s}  {note}')
    print(LINE)
    by_name = {}
    for r in results:
        if r['k'] is not None:
            by_name.setdefault(r['name'], []).append(r['k'])
    for name, v in by_name.items():
        if len(v) >= 2:
            m = statistics.mean(v)
            print(f'  {name}: {len(v)}회 평균 {m:.3f} N/mm, 흩어짐 ±{statistics.stdev(v) / m * 100:.1f}% '
                  f'(목표 ±5% 이내)')
    if len(by_name) >= 2:
        means = {n: statistics.mean(v) for n, v in by_name.items()}
        lo, hi = min(means, key=means.get), max(means, key=means.get)
        print(f'  가장 무른 칸 {lo} {means[lo]:.3f}, 가장 단단한 칸 {hi} {means[hi]:.3f} '
              f'→ 비율 {means[hi] / means[lo]:.2f} 배 (40A:10A 구분에 2배 이상 필요)')


def save(results, noise):
    os.makedirs(OUT_DIR, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    path = os.path.join(OUT_DIR, f'stiffness_{stamp}.csv')
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['time', 'name', 'k_N_per_mm', 'z_surf_mm', 'z0_mm', 'disp_mm', 'f_max_N', 'r2', 'error', 'note'])
        for r in results:
            w.writerow([r['time'], r['name'], r['k'], r['z_surf'], r['z0'], r['disp'], r['f_max'],
                        r['r2'], r['error'], r['note']])
    with open(os.path.join(OUT_DIR, f'stiffness_{stamp}_curves.json'), 'w') as f:
        json.dump({'noise': noise, 'target_N': cfg.F_TARGET,
                   'curves': [{'time': r['time'], 'name': r['name'], 'curve': r['curve']} for r in results]},
                  f, ensure_ascii=False, indent=1)
    return path


def main(args=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('file', nargs='?', help='교시 파일 (없으면 목록에서 선택)')
    ap.add_argument('--approach', type=float, default=cfg.PRESS_APPROACH_DZ,
                    help=f'교시 높이에서 빠르게 내려오는 양 [mm] (기본 {cfg.PRESS_APPROACH_DZ:g})')
    ap.add_argument('--target', type=float, default=cfg.F_TARGET,
                    help=f'목표 힘 [N] (기본 {cfg.F_TARGET:g})')
    ap.add_argument('--modbus', action='store_true', help='그리퍼를 /onrobot/sendCommand 로 제어')
    opt = ap.parse_args(remove_ros_args(sys.argv)[1:])

    path = os.path.expanduser(opt.file) if opt.file else choose_file()
    if path is None:
        return
    try:
        points = load_points(path)
    except (OSError, ValueError, KeyError) as e:
        print(f'교시 파일을 읽을 수 없습니다: {path} ({e})')
        return
    if not points:
        print('측정할 칸이 없습니다.')
        return

    # 공정과 같은 측정 코드를 쓰되, 이 테스트에 맞게 설정 몇 개만 바꾼다
    cfg.GRIPPER_MODE = 'service' if opt.modbus else 'io'      # 지금 셀은 WebLogic(DO) 로 동작 확인됨
    cfg.F_TARGET = opt.target
    if cfg.F_TARGET < 12 or cfg.F_TARGET > cfg.F_LIMIT - 5:
        print(f'목표 힘은 12 ~ {cfg.F_LIMIT - 5:g}N 사이로 주세요. 강성은 {cfg.F_FIT_MIN:g}N~목표 구간으로 계산하는데, '
              '목표가 낮으면 단단한 큐브에서 측정 점이 모자랍니다.')
        return

    rclpy.init(args=args)
    node = rclpy.create_node('pjt_stiffness_test', namespace=cfg.ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        import DSR_ROBOT2 as dr
    except ImportError as e:
        node.get_logger().error(f'DSR_ROBOT2 를 불러오지 못했습니다: {e}')
        rclpy.shutdown()
        return
    from bed_inspect.bed_robot import BedError, BedRobot
    from bed_inspect.bed_tasks import BedTasks

    class Robot(BedRobot):
        """posj 가 없는(좌표로 계산한) 점은 posx 로 직선 이동한다."""

        def movej_to(self, point, slow=False):
            if point.get('posj'):
                return super().movej_to(point, slow)
            self.require_standby(point['name'])
            ret = self.dr.movel(self.posx(list(point['posx'])), vel=cfg.VELX, acc=cfg.ACCX, ref=self.dr.DR_BASE)
            self._done(ret, f'[{point["name"]}] 직선 이동')

    log = node.get_logger().info
    robot = Robot(dr, node, log)
    tasks = BedTasks(robot, {p['name']: p for p in points}, log)
    results, noise = [], []

    def measure(p, n=1):
        new = []
        for i in range(n):
            if n > 1:
                log(f'[{p["name"]}] {i + 1}/{n}회')
            res = tasks.press_at(p, opt.approach)
            res.update(name=p['name'], time=time.strftime('%H:%M:%S'))
            new.append(res)
        return new

    try:
        robot.wait_services()
        tcp = robot.prepare()
        bad = [p['name'] for p in points if not p.get('posj') and p.get('tcp', '') != tcp]
        if bad:
            raise BedError(f'현재 TCP({tcp or "없음"})가 교시 파일과 다릅니다: {bad}. '
                           'TCP 를 맞추거나 파일의 tcp 값을 고치세요.')
        log(f'TCP {tcp}, 그리퍼 {cfg.GRIPPER_MODE}, 목표 힘 {cfg.F_TARGET:g}N, 접촉 {cfg.F_CONTACT:g}N, '
            f'접근 {opt.approach:g}mm')

        print(f'\n=== {os.path.basename(path)} ===')
        for i, p in enumerate(points, 1):
            print(f'  {i}. {p["name"]:10s} {"movej" if p.get("posj") else "movel"}  '
                  f'x={p["posx"][0]:.1f} y={p["posx"][1]:.1f} z={p["posx"][2]:.1f}')
        print('번호: 측정 | 번호*횟수: 반복 측정 | a: 전부 | n: 노이즈 | r: 결과 | q: 저장 후 종료')

        while True:
            sel = input('> ').strip().lower()
            if sel == 'q':
                break
            if sel == 'r':
                print_results(results)
                continue
            if sel == 'n':
                vals = robot.read_fz_list(NOISE_SAMPLES)
                m, s = statistics.mean(vals), statistics.stdev(vals)
                noise.append({'time': time.strftime('%H:%M:%S'), 'mean': m, 'std': s})
                print(f'  힘 노이즈 ({len(vals)}회): 평균 {m:.2f} N, 표준편차 σ {s:.3f} N, '
                      f'범위 {min(vals):.2f} ~ {max(vals):.2f} N')
                print(f'  → 접촉 기준 {cfg.F_CONTACT:g}N 은 σ 의 {cfg.F_CONTACT / s:.0f} 배 '
                      f'({"충분" if cfg.F_CONTACT >= 5 * s else "부족 - F_CONTACT 를 올려야 함"})')
                continue
            if sel == 'a':
                todo = [(p, 1) for p in points]
            else:
                num, _, cnt = sel.partition('*')
                if not (num.isdigit() and 1 <= int(num) <= len(points) and (cnt == '' or cnt.isdigit())):
                    print('  번호, 번호*횟수, a, n, r, q 중에 입력하세요.')
                    continue
                todo = [(points[int(num) - 1], int(cnt) if cnt else 1)]
            new = []
            for p, n in todo:
                new += measure(p, n)
            results += new
            print_results(new)
    except BedError as e:
        node.get_logger().error(str(e))
    except (KeyboardInterrupt, EOFError):
        print()
        node.get_logger().warn('중단됨 (로봇이 아직 움직이면 펜던트에서 정지하세요)')
    finally:
        if results:
            print('\n=== 전체 결과 ===')
            print_results(results)
            print('저장:', save(results, noise))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
