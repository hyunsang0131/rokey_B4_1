# 에뮬레이터(sodvir)에서 공정 동작을 눈으로 확인하기 위한 "가짜 교시 파일" 만들기
#
#   1) 에뮬레이터 실행:  sodvir   (bringup mode:=virtual, RViz 가 뜬다)
#   2) ros2 run bed_inspect emu_teach          → ~/ws_cobot_pjt/teach_points/bed_emu.json
#   3) 공정 실행 (README 의 "에뮬레이터로 보기" 참고)
#
# 침대 칸, 보관대 자리의 XY 는 bed_config 의 BED_XY, RACK_XY 를 쓴다.
# 높이, 툴 자세, 홈, 롤러, 커버, 버리는 곳은 아직 실물이 없어서 아래 "가정값"을 쓴다.
#   → 실제 로봇에서는 이 파일을 쓰지 말고 direct_teach 로 교시한 bed.json 을 쓴다.
#
# 에뮬레이터의 역기구학(ikin)으로 좌표 → 관절각을 구하고, 정기구학(fkin)으로 다시 계산해서
# 1mm 이상 어긋나는 점이 있으면 저장하지 않는다 (닿지 않는 위치).

import json
import math
import os
import sys
import time

import rclpy
from rclpy.utilities import remove_ros_args
import DR_init

from bed_inspect import bed_config as cfg

DR_init.__dsr__id = cfg.ROBOT_ID
DR_init.__dsr__model = cfg.ROBOT_MODEL

OUT_FILE = os.path.expanduser('~/ws_cobot_pjt/teach_points/bed_emu.json')

# ── 가정값 (mm, deg). 실물이 정해지면 direct_teach 로 교시한 값으로 바꾼다 ─────────
HOME_J = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]   # 홈 관절각
TEACH_Z = 180.0                             # 침대 칸 / 보관대 / 버리는 곳 교시 높이 (블록을 들고 지나가도 되는 높이)
DOWN = [0.0, 180.0, 0.0]                    # 툴이 아래를 보는 자세 (A, B, C)
SIDE = [0.0, 90.0, 0.0]                     # 툴이 +x 쪽을 보는 자세 (롤러 손잡이 잡기)
DISCARD_XY = (300.0, -300.0)                # 불량 블록 버리는 통


def bed_center():
    xs = [x for x, _ in cfg.BED_XY]
    ys = [y for _, y in cfg.BED_XY]
    return (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, max(xs), min(ys)


def target_poses():
    """교시 이름 → posx [x, y, z, a, b, c] (홈 제외)."""
    cx, cy, x_end, y_low = bed_center()
    p = {}
    for name, (x, y) in zip(cfg.P_BED, cfg.BED_XY):
        p[name] = [x, y, TEACH_Z] + DOWN
    for name, (x, y) in zip(cfg.P_RACK, cfg.RACK_XY):
        p[name] = [x, y, TEACH_Z] + DOWN
    p[cfg.P_DISCARD] = [DISCARD_XY[0], DISCARD_XY[1], TEACH_Z] + DOWN
    # 롤러 단계는 삭제됨. 커버는 교시하지 않고 bed_config 의 COVER_* 좌표로 움직인다
    return p


def main(args=None):
    out = OUT_FILE
    user_args = remove_ros_args(sys.argv)[1:]
    if user_args:
        out = os.path.expanduser(user_args[0])

    rclpy.init(args=args)
    node = rclpy.create_node('bed_emu_teach', namespace=cfg.ROBOT_ID)
    DR_init.__dsr__node = node
    log = node.get_logger()
    try:
        import DSR_ROBOT2 as dr
        from DR_common2 import posj, posx
    except ImportError as e:
        log.error(f'DSR_ROBOT2 를 불러오지 못했습니다: {e}')
        rclpy.shutdown()
        return 1

    code = 1
    try:
        for cli in (dr._ros2_ikin, dr._ros2_fkin, dr._ros2_get_solution_space, dr._ros2_get_current_tcp):
            n = 0
            while not cli.wait_for_service(timeout_sec=1.0):
                n += 1
                log.info(f'서비스 기다리는 중: {cli.srv_name} (sodvir 가 떠 있나요?)')
                if n >= 20:
                    log.error('에뮬레이터 서비스가 없습니다. 먼저 sodvir 를 실행하세요.')
                    return 1

        tcp = dr.get_tcp()
        tcp = tcp if isinstance(tcp, str) else ''
        sol = dr.get_solution_space(posj(HOME_J))
        home_x = dr.fkin(posj(HOME_J), ref=dr.DR_BASE)
        log.info(f'TCP: "{tcp}" / 홈 자세의 해 공간: {sol}')

        points = [{'name': cfg.P_HOME, 'posj': HOME_J, 'posx': [round(v, 3) for v in home_x],
                   'tcp': tcp, 'time': time.strftime('%H:%M:%S')}]
        bad = []
        for name, px in target_poses().items():
            j = dr.ikin(posx(px), sol, ref=dr.DR_BASE)
            if j is None or isinstance(j, (int, float)):        # 실패하면 -1 같은 숫자, 성공하면 배열
                bad.append(f'{name}: 역기구학 해 없음')
                continue
            j = [float(v) for v in j]
            back = dr.fkin(posj(j), ref=dr.DR_BASE)
            err = math.dist(list(back)[:3], px[:3])
            if err > 1.0:
                bad.append(f'{name}: 닿지 않음 (오차 {err:.1f} mm)')
                continue
            points.append({'name': name, 'posj': [round(v, 3) for v in j], 'posx': px,
                           'tcp': tcp, 'time': time.strftime('%H:%M:%S')})
            log.info(f'{name:16s} posx={[round(v, 1) for v in px]} → posj={[round(v, 1) for v in j]}')

        if bad:
            log.error('만들지 못한 점이 있어 저장하지 않습니다:\n  ' + '\n  '.join(bad))
            return 1
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, 'w') as f:
            json.dump(points, f, indent=2, ensure_ascii=False)
        log.info(f'저장: {out} ({len(points)}개)')
        code = 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
