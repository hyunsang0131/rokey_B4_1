# [v2 = 영상 후 버전: move_spiral 없음. 높이 올려 장력 찾고 그 반지름으로 크게 돌기] 나사로 실 고정 / 고정해제 (2026-11-21 형님 손그림 + 영상 IMG_9588).
# ros2 run bed_inspect string_v2 fix|unfix|points. 파일: string_v2_config / string_v2_plan / string_v2_demo.
#
#   ros2 run bed_inspect string_demo points --pick 300 -20 300          # 좌표만 출력 (로봇 안 움직임)
#   ros2 run bed_inspect string_v2 fix        # 고정: 구멍 나사 잡고 돌려 풀어 뽑기 → z 올려 장력 찾기 → 그 반지름으로 2바퀴 → 구멍에 순응 제어로 조이기
#   ros2 run bed_inspect string_demo unfix --place 300 -20 300          # 고정해제: 구멍의 나사 잡아 뽑기 → 반대로 2바퀴 풀기 → 놓기
#   옵션: --virtual (RViz 가상), --yes, --home j1..j6   (move_spiral 코드는 남아 있지만 v2 고정에서는 안 씀)
#
# 설정·확인 필요 값은 string_v2_config. 실물에서는 한 손을 비상정지에 두세요.

import argparse
import math
import sys
import time

import rclpy
from rclpy.utilities import remove_ros_args
import DR_init

from bed_inspect import bed_config as cfg
from bed_inspect import string_v2_config as sc
from bed_inspect import string_v2_plan as plan

DR_init.__dsr__id = cfg.ROBOT_ID
DR_init.__dsr__model = cfg.ROBOT_MODEL


def fmt(q):
    return ' '.join(f'{v:9.2f}' for v in q)


def print_points(action, p):
    print(f'  [v2] 모드 {action}, 감기 = 높이 {sc.WIND2_Z:g} 에서 장력 {sc.TENSION_N:g}N 찾고 {sc.CIRCLE_TURNS:g}바퀴 원 '
          f'(최대 반지름 {plan.max_circle_r():.0f}mm), TCP {sc.STR_TCP}, 탁자 z {sc.TABLE_Z}, 감는 나사 {sc.POST_XY}, '
          f'구멍 {sc.HOLE_XY}, 구멍 잡는 높이 {plan.hole_grip_z():.1f}')
    for k, v in p.items():
        if k == 'spiral':
            sp = v
            if sp['mode'] == 'inward':
                print(f'  spiral    안쪽 나선: 시작 반지름 {sc.SPIRAL_R_OUT} → {sp["end_r"]} 에서 정지 ({sp["stop_deg"]:.0f}도), '
                      f'목표(중심) {fmt(sp["target"][:3])}, 전체 {sp["rev"]:.1f}바퀴, z {sp["start"][2]:.1f} → {sp["end_z"]:.1f}')
            else:
                print(f'  spiral    바깥 나선: 중심 {fmt(sp["start"][:3])}, {sp["rev"]:.1f}바퀴, 반지름 0 → {sp["rmax"]}, '
                      f'z +{sp["lmax"]}')
        elif k == 'unwind':
            print(f'  unwind    풀기 원호 {v["angle"]:.0f}도 (위에서 {"시계" if sc.SPIRAL_CCW else "반시계"})')
            for n in ('start', 'via', 'end'):
                print(f'    {n:8s} {fmt(v[n])}')
        else:
            print(f'  {k:12s} {fmt(v)}')


class StringDemo:
    def __init__(self, robot, dr, node, log):
        self.r, self.dr, self.node, self.log = robot, dr, node, log
        self._spiral_cli = None
        self.virtual = False
        self._pushing = False

    # ── 기본 동작 ──
    def line(self, p, what, slow=False):
        v, a = (sc.SLOW_VEL, sc.SLOW_ACC) if slow else (sc.MOVE_VEL, sc.MOVE_ACC)
        self.r.movel_to(p, v, a, what)

    def tool_move(self, dz, rz, what, vel=None, acc=None):
        """툴 기준 상대 이동 (툴 Z 는 아래 방향): dz [mm], rz [deg] (툴 Z 둘레 회전)."""
        self.r.require_standby(what)
        self._mark(what)
        ret = self.dr.movel(self.r.posx([0.0, 0.0, float(dz), 0.0, 0.0, float(rz)]),
                            vel=list(vel or sc.TIGHTEN_VEL), acc=list(acc or sc.TIGHTEN_ACC),
                            ref=self.dr.DR_TOOL, mod=self.dr.DR_MV_MOD_REL)
        self.r._done(ret, what)

    # ── move_spiral ──
    def _spiral_call(self, req, timeout):
        from dsr_msgs2.srv import MoveSpiral
        if self._spiral_cli is None:
            self._spiral_cli = self.node.create_client(MoveSpiral, self.dr._srv_name_prefix + 'motion/move_spiral')
            if not self._spiral_cli.wait_for_service(timeout_sec=3.0):
                raise self.r_error('move_spiral 서비스가 없습니다.')
        fut = self._spiral_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result() if fut.done() else None
        return res is not None and res.success

    def _mark(self, what, target=None):
        """충돌 복구용 동작 기록 (BedRobot 에 _begin 이 있는 버전에서만)."""
        f = getattr(self.r, '_begin', None)
        if f is not None:
            f(what, target)

    def r_error(self, msg):
        from bed_inspect.bed_robot import BedError
        return BedError(msg)

    def spiral(self, sp):
        from dsr_msgs2.srv import MoveSpiral
        req = MoveSpiral.Request()
        req.vel = [float(v) for v in sc.SPIRAL_VEL]
        req.acc = [float(v) for v in sc.SPIRAL_ACC]
        req.time = 0.0
        req.task_axis = 2                       # Z
        req.ref = 0                             # DR_BASE
        req.mode = 0                            # 절대
        what = 'move_spiral 감기'
        self.r.require_standby(what)
        self._mark(what)
        if sp['mode'] == 'outward':
            # 기본형: 현재 위치가 중심, 반지름 0 → rmax, 축 방향 lmax. 방향은 컨트롤러 기본 (rot_dir 은 목표점 형에서만)
            req.revolution = float(sp['rev'])
            req.max_radius = float(sp['rmax'])
            req.max_length = float(sp['lmax'])
            req.target_pos = [0.0, 0.0, 0.0]
            req.sync_type = 0
            if not self._spiral_call(req, timeout=120.0):
                raise self.r_error(f'{what} 실패. {self.r.fault_text()}')
            self.r.require_standby(what)
            return
        # 안쪽형: 목표점(감는 나사)이 중심, 지금 위치(반지름 30)에서 안쪽으로. 0 까지 가기 전 2바퀴에서 멈춘다
        req.revolution = float(sp['rev'])
        req.max_radius = 0.0
        req.max_length = 0.0
        req.target_pos = [float(v) for v in sp['target']]
        req.spiral_dir = 1                      # INWARD
        req.rot_dir = sc.SPIRAL_ROT_DIR_CCW if sc.SPIRAL_CCW else 1 - sc.SPIRAL_ROT_DIR_CCW
        req.sync_type = 1                       # 비동기
        watch = plan.SpiralWatch(sc.POST_XY, sp['stop_deg'], ccw=sc.SPIRAL_CCW)
        if not self._spiral_call(req, timeout=5.0):
            raise self.r_error(f'{what} 시작 실패. {self.r.fault_text()}')
        t0, busy = time.monotonic(), False
        limit = 3.0 * (2 * math.pi * sc.SPIRAL_R_OUT * sp['rev']) / sc.SPIRAL_VEL[0] + 10.0
        try:
            while True:
                self.r.check_stop()
                xy = self.r.cur_xyz()[:2]
                res = watch.update(xy)
                if res == 'stop':
                    self.r.stop_motion()
                    break
                if res in ('too_close', 'wrong_dir'):
                    self.r.halt()
                    raise self.r_error(
                        f'{what}: ' + ('감는 나사에 너무 가까워져' if res == 'too_close' else
                                       f'반대 방향으로 돌아 (rot_dir 부호 확인: string_v2_config.SPIRAL_ROT_DIR_CCW)')
                        + f' 멈췄습니다. 반지름 {watch.r:.1f}mm, 돈 각도 {watch.swept:.0f}도')
                st = self.dr.check_motion()
                busy = busy or st == self.dr.DR_STATE_BUSY
                if st == self.dr.DR_STATE_IDLE and (busy or time.monotonic() - t0 > 1.0):
                    raise self.r_error(f'{what}: {watch.swept:.0f}도만 돌고 끝났습니다 (목표 {sp["stop_deg"]:.0f}도). '
                                       f'반지름 {watch.r:.1f}mm. {self.r.fault_text()}')
                if time.monotonic() - t0 > limit:
                    self.r.halt()
                    raise self.r_error(f'{what}: 시간 초과 ({limit:.0f}s)')
                time.sleep(0.02)
        except BaseException:
            if self.dr.check_motion() != self.dr.DR_STATE_IDLE:
                self.r.halt()
            raise
        self.r.require_standby(what)
        self.log(f'  감기 끝: {watch.swept:.0f}도, 반지름 {watch.r:.1f}mm, z {self.r.cur_xyz()[2]:.1f}')

    # ── 조이기 / 풀기 ──
    def j6_prepare(self, turn_deg, what):
        """(공중에서) 한 번에 turn_deg 돌려도 J6 범위 안에 들도록 J6 를 미리 돌려 둔다."""
        j = self.r.cur_posj()
        want = plan.j6_start(j[5], turn_deg)
        if abs(want - j[5]) < 0.5:
            return
        self.log(f'  {what}: J6 {j[5]:.1f} → {want:.1f} 미리 돌림 ({turn_deg:+.0f}도 돌릴 자리 확보)')
        self.r.require_standby(what)
        self._mark(what)
        self.r._done(self.dr.movej(self.r.posj(j[:5] + [want]), vel=30, acc=60), what)

    def turn(self, deg, what, advance=0):
        """툴 Z 둘레로 deg 회전 (ROT_CHUNK_DEG 이하로 나눠서).
        advance: +1 = 조이기(나사가 내려감), -1 = 풀기(나사가 올라옴), 0 = 그리퍼만 되돌리기.
        잡고 돌릴 때는 회전과 함께 나사 피치만큼 z 도 같이 움직인다 (나선 운동, 나사산과 같은 길).
        툴 Z 가 아래를 보므로 툴 +dz = 베이스 아래."""
        for d in plan.rot_chunks(deg):
            dz = advance * sc.SCREW_PITCH * abs(d) / 360.0
            self.tool_move(dz, d, what)

    def press_on(self, what, push=True):
        """순응 제어(x·y·z 부드럽게, 회전 단단) + (push 면) 아래로 TIGHTEN_PUSH_N 누르기."""
        self._pushing = push and sc.TIGHTEN_PUSH_N > 0
        if self.dr.task_compliance_ctrl(list(sc.TIGHTEN_COMPLIANCE)) != 0:
            raise self.r_error(f'{what}: 순응 제어를 켜지 못했습니다. {self.r.fault_text()}')
        self.r.sleep(sc.TIGHTEN_SETTLE)
        if self._pushing:
            if self.dr.set_desired_force([0.0, 0.0, -float(sc.TIGHTEN_PUSH_N), 0.0, 0.0, 0.0], [0, 0, 1, 0, 0, 0],
                                         mod=self.dr.DR_FC_MOD_REL) != 0:
                self.dr.release_compliance_ctrl()
                raise self.r_error(f'{what}: 힘 제어를 켜지 못했습니다. {self.r.fault_text()}')
            self.r.sleep(sc.TIGHTEN_SETTLE)

    def press_off(self):
        if self._pushing:
            self.dr.release_force()
        self.dr.release_compliance_ctrl()

    def ratchet(self, reps, sign, what, push=True, loosen=False):
        """잡은 상태에서 시작 (J6 는 j6_prepare 로 미리 돌려 둠). 한 번 =
        잡은 채로 TURN 만큼 돌리기 → 놓기 → 살짝 위 → 그리퍼만 되돌리기 → 나사 높이로 내려가 다시 잡기.
        조이기(loosen=False): 순응 제어(x·y 부드럽게 → 구멍에 제대로 자리잡게) + 아래로 누르며. 마지막엔 놓고 위로.
        풀기(loosen=True): z 만 부드럽게(나사가 올라오는 만큼 따라감), 누르지 않음. 마지막엔 잡은 채로 끝 (뽑기 위해)."""
        deg = sign * sc.TIGHTEN_TURN_DEG
        for i in range(reps):
            last = i == reps - 1
            self.log(f'  {what} {i + 1}/{reps}: {deg:+.0f}도' + (f' (아래로 {sc.TIGHTEN_PUSH_N:g}N)' if push and not loosen else ''))
            if loosen:
                self._pushing = False
                if self.dr.task_compliance_ctrl(list(sc.LOOSEN_COMPLIANCE)) != 0:
                    raise self.r_error(f'{what}: 순응 제어를 켜지 못했습니다. {self.r.fault_text()}')
                self.r.sleep(sc.TIGHTEN_SETTLE)
            else:
                self.press_on(what, push)
            try:
                self.turn(deg, f'{what} 회전', advance=-1 if loosen else 1)
                z_end = self.r.cur_z()
            finally:
                self.press_off()
            if last and loosen:
                break                           # 잡은 채로 → 부르는 쪽이 뽑는다
            self.r.grip(sc.OPEN_KEY)
            self.r.move_z(sc.TIGHTEN_LIFT, vel=sc.SLOW_VEL, acc=sc.SLOW_ACC)
            if last:
                break
            self.turn(-deg, f'{what} 그리퍼 되돌리기')
            # 나사가 내려간(풀 때는 올라온) 높이 그대로 가서 머리를 다시 잡는다 (돌린 만큼 피치로 이미 따라갔음)
            self.r.move_z(z_end - self.r.cur_z(), vel=sc.SLOW_VEL, acc=sc.SLOW_ACC)
            self.r.grip(sc.GRIP_KEY)

    def unscrew_out(self, app, grip_z, out_z, what):
        """구멍에 꽂힌 나사를 잡고 → 잡은 채로 돌려 풀고 → 수직으로 뽑는다 (11-21 형님: 안 돌리고 당기면 손가락에서 빠짐)."""
        self.r.grip(sc.OPEN_KEY)
        self.line(app, f'{what} 위')
        if sc.LOOSEN_REPS > 0:
            self.j6_prepare(-sc.TIGHTEN_SIGN * sc.TIGHTEN_TURN_DEG, '풀 준비')
        # 미리 돌린 J6 가 풀리지 않게 자세는 그대로 두고 수직으로만
        self.r.move_z(grip_z - self.r.cur_z(), vel=sc.SLOW_VEL, acc=sc.SLOW_ACC)
        self.r.grip(sc.GRIP_KEY)
        if sc.LOOSEN_REPS > 0:
            self.log(f'   잡은 채로 반대로 {sc.TIGHTEN_TURN_DEG:.0f}도씩 {sc.LOOSEN_REPS}번 풀기')
            self.ratchet(sc.LOOSEN_REPS, -sc.TIGHTEN_SIGN, '풀기', push=False, loosen=True)
        self.log('   위로 뽑기')
        self.r.move_z(out_z - self.r.cur_z(), vel=sc.SLOW_VEL, acc=sc.SLOW_ACC)

    # ── 전체 동작 ──
    def find_tension(self, end, what='장력 찾기'):
        """end 쪽으로 천천히 직선 이동하며 수평 외력 변화가 TENSION_N 을 넘으면(실이 팽팽) 멈춘다. 멈춘 xy 를 돌려준다."""
        self.r.require_standby(what)
        self._mark(what, end)
        f0 = self.r.read_force_vec(5)
        if self.dr.amovel(self.r.posx([float(v) for v in end]), vel=list(sc.TENSION_VEL), acc=[40.0, 40.0],
                          ref=self.dr.DR_BASE, mod=self.dr.DR_MV_MOD_ABS) != 0:
            raise self.r_error(f'{what} 시작 실패. {self.r.fault_text()}')
        t0, busy, df = time.monotonic(), False, 0.0
        try:
            while True:
                self.r.check_stop()
                f = self.r.read_force_vec(1)
                df = math.hypot(f[0] - f0[0], f[1] - f0[1])
                if df >= sc.TENSION_N:
                    self.r.stop_motion()
                    break
                st = self.dr.check_motion()
                busy = busy or st == self.dr.DR_STATE_BUSY
                if st == self.dr.DR_STATE_IDLE and (busy or time.monotonic() - t0 > 1.0):
                    raise self.r_error(f'{what}: {sc.TENSION_MAX:.0f}mm 를 가도 장력({sc.TENSION_N:g}N)을 못 느꼈습니다 '
                                       f'(최대 {df:.1f}N). 실이 걸렸는지 확인하세요.')
                time.sleep(0.02)
        except BaseException:
            if self.dr.check_motion() != self.dr.DR_STATE_IDLE:
                self.r.halt()
            raise
        xy = self.r.cur_xyz()[:2]
        self.log(f'  장력 {df:.1f}N 에서 멈춤: xy ({xy[0]:.1f}, {xy[1]:.1f}), 감는 나사에서 '
                 f'{math.hypot(xy[0] - sc.POST_XY[0], xy[1] - sc.POST_XY[1]):.0f}mm')
        return xy

    def circle(self, c, what):
        self.r.require_standby(what)
        self._mark(what, c['end'])
        ret = self.dr.movec(self.r.posx(c['via']), self.r.posx(c['end']), vel=list(sc.CIRCLE_VEL),
                            acc=list(sc.CIRCLE_ACC), ref=self.dr.DR_BASE, mod=self.dr.DR_MV_MOD_ABS, angle=c['angle'])
        self.r._done(ret, what)

    def fix(self, p):
        self.log('1) 실 끝 나사 잡고 돌려 풀어서 뽑기')
        self.unscrew_out(p['pick_app'], p['pick'][2], p['pick_up'][2], '실 끝 나사')
        self.log(f'2) z {sc.WIND2_Z:g} 까지 올리기')
        self.r.move_z(sc.WIND2_Z - self.r.cur_z())
        self.line(p['wind_up'], '자세 바로잡기', slow=True)
        self.log(f'3) 실이 팽팽해질 때까지 이동 (장력 {sc.TENSION_N:g}N)')
        if self.virtual:
            # 에뮬레이터는 실이 없어 장력을 못 느낌 → 감는 나사 너머 VIRTUAL_TENSION_R 자리를 장력 자리로 보고 진행
            d = plan.tension_dir()
            t = (sc.POST_XY[0] + d[0] * sc.VIRTUAL_TENSION_R, sc.POST_XY[1] + d[1] * sc.VIRTUAL_TENSION_R)
            self.log(f'  [가상] 장력 대신 감는 나사 너머 {sc.VIRTUAL_TENSION_R:g}mm 자리로')
            self.line(plan.pose(t, sc.WIND2_Z), '[가상] 장력 자리', slow=True)
        else:
            t = self.find_tension(p['tension_end'])
        try:
            c = plan.circle_through(t, sc.WIND2_Z, sc.CIRCLE_TURNS, ccw=sc.SPIRAL_CCW)
        except plan.StringPlanError as e:
            raise self.r_error(f'{e} → 장력 찾은 자리에서 멈췄습니다. 실을 짧게 하거나 STR_TENSION_MAX 를 줄이세요.')
        self.log(f'4) 그 반지름 {c["r"]:.0f}mm 로 감는 나사 둘레 {c["angle"]:.0f}도 (위에서 {"반시계" if sc.SPIRAL_CCW else "시계"})')
        self.circle(c, '크게 돌며 감기')
        self.log('5) 가운데 구멍 위로, 순응 제어로 구멍에 자리잡고 돌려 고정')
        self.line(p['hole_over'], '구멍 위')
        self.j6_prepare(sc.TIGHTEN_SIGN * sc.TIGHTEN_TURN_DEG, '돌릴 준비')
        # 미리 돌린 J6 가 풀리지 않게 자세는 그대로 두고 수직으로만 내려간다
        self.r.move_z(p['hole_touch'][2] - self.r.cur_z(), vel=sc.SLOW_VEL, acc=sc.SLOW_ACC)
        self.log(f'6) {sc.TIGHTEN_TURN_DEG:.0f}도씩 {sc.TIGHTEN_REPS}번 조이기')
        self.ratchet(sc.TIGHTEN_REPS, sc.TIGHTEN_SIGN, '조이기')
        self.line(p['hole_app'], '구멍 위로 빠지기')

    def unfix(self, p):
        self.log('1) 가운데 구멍의 나사 잡고 돌려 풀어서 뽑기')
        self.unscrew_out(p['hole_app'], p['hole_grip'][2], p['hole_pull'][2], '가운데 구멍 나사')
        arc = p['unwind']
        self.r.move_z(sc.WIND2_Z - self.r.cur_z())
        self.log(f'2) 감은 반대로 {arc["angle"]:.0f}도 풀기')
        self.line(arc['start'], '풀기 시작점')
        self.r.require_standby('풀기 원호')
        self._mark('풀기 원호', arc['end'])
        ret = self.dr.movec(self.r.posx(arc['via']), self.r.posx(arc['end']), vel=list(sc.UNWIND_VEL),
                            acc=list(sc.UNWIND_ACC), ref=self.dr.DR_BASE, mod=self.dr.DR_MV_MOD_ABS,
                            angle=arc['angle'])
        self.r._done(ret, '풀기 원호')
        self.log('3) 나사 내려놓기 (가운데 구멍에 대략 꽂아 둠)')
        self.line(p['place_over'], '놓는 자리 위')
        self.line(p['place'], '놓는 자리', slow=True)
        self.r.grip(sc.OPEN_KEY)
        self.line(p['place_app'], '놓는 자리 위로 빠지기')


def main(args=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('action', choices=['fix', 'unfix', 'points'])
    ap.add_argument('--pick', type=float, nargs=3, help='고정: 실 끝 나사를 잡는 TCP 위치 x y z')
    ap.add_argument('--place', type=float, nargs=3, help='고정해제: 풀고 나서 나사를 놓을 위치 x y z (없으면 --pick)')
    ap.add_argument('--spiral', choices=['inward', 'outward'], help='감기 방식 (기본 string_v2_config.SPIRAL_MODE)')
    ap.add_argument('--home', type=float, nargs=6, default=cfg.HOME_POSJ, help='home 관절각')
    ap.add_argument('--virtual', action='store_true', help='RViz 가상 모드: TCP 가 달라도 경고만')
    ap.add_argument('--yes', action='store_true', help='시작 전에 Enter 를 기다리지 않음')
    opt = ap.parse_args(remove_ros_args(sys.argv)[1:])
    if opt.spiral:
        sc.SPIRAL_MODE = opt.spiral
    pick = opt.pick or sc.PICK_XYZ
    place = opt.place or sc.PLACE_XYZ or pick

    try:
        fixp = plan.fix_points(pick) if pick is not None else None
        unfixp = plan.unfix_points(place) if place is not None else None
        if opt.action == 'fix' and fixp is None:
            plan.fix_points(None)
        if opt.action == 'unfix' and unfixp is None:
            plan.unfix_points(None)
    except plan.StringPlanError as e:
        print(f'설정 오류: {e}')
        return
    if fixp and opt.action in ('fix', 'points'):
        print('[고정]')
        print_points('fix', fixp)
    if unfixp and opt.action in ('unfix', 'points'):
        print('[고정해제]')
        print_points('unfix', unfixp)
    if opt.action == 'points':
        if fixp is None and unfixp is None:
            print('--pick 또는 --place 를 주면 좌표를 출력합니다.')
        return

    if opt.virtual:
        cfg.VIRTUAL = True                      # 에뮬레이터: TCP 자동 설정 건너뜀 (에뮬레이터엔 GripperDA_v1 없음)
    for k, w in sc.GRIP_WIDTHS.items():          # 나사 잡기 폭 (bed_config 에 없어도 동작)
        cfg.GRIP_WIDTH.setdefault(k, w)
    cfg.GRIPPER_MODE = sc.GRIPPER_MODE          # 나사 돌리기는 센 그리퍼 (bed 공정 설정과 따로)
    rclpy.init(args=args)
    node = rclpy.create_node('string_demo', namespace=cfg.ROBOT_ID)
    DR_init.__dsr__node = node
    import DSR_ROBOT2 as dr
    from bed_inspect.bed_robot import BedError, BedRobot

    log = node.get_logger().info
    robot = BedRobot(dr, node, log)
    demo = StringDemo(robot, dr, node, log)
    demo.virtual = opt.virtual
    home = {'name': cfg.P_HOME, 'posj': list(opt.home)}
    try:
        robot.wait_services()
        tcp = robot.prepare()
        log(f'TCP: {tcp or "(없음)"}, 그리퍼 모드 {cfg.GRIPPER_MODE}, 감기 {sc.SPIRAL_MODE}')
        if sc.STR_TCP != tcp:
            msg = f'현재 TCP("{tcp or "없음"}")가 좌표 기준 TCP("{sc.STR_TCP}")와 다릅니다.'
            if not opt.virtual:
                raise BedError(msg + ' 펜던트에서 TCP 를 바꾼 뒤 다시 실행하세요. (RViz 가상 모드면 --virtual)')
            node.get_logger().warn(msg + ' --virtual 이라 계속합니다.')
        if not opt.yes:
            input(f'Enter 를 누르면 {"고정" if opt.action == "fix" else "고정해제"} 동작을 시작합니다 (Ctrl+C 취소) > ')
        robot.movej_to(home)
        if opt.action == 'fix':
            log('=== 나사로 실 고정 ===')
            demo.fix(fixp)
        else:
            log('=== 실 고정해제 ===')
            demo.unfix(unfixp)
        robot.movej_to(home)
        log('끝.')
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
