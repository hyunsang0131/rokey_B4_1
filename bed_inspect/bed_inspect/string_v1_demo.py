# [v1 = 영상 보기 전 버전 (30/20, 180도), 구멍만 순응 제어로 (11-21 08:14)] 나사로 실 고정 / 고정해제.
#   감기: 반지름 30 → 20 안쪽 move_spiral 2바퀴, 조이기: 180도 x 4번
#   구멍: 3mm 위까지 내려간 뒤 순응 제어(x·y·z 부드럽게) + 아래로 5N 누르며 돌림 (08:14 형님 요청, v2 와 같은 방식)
#
#   ros2 run bed_inspect string_v1 points --pick 300 -20 300
#   ros2 run bed_inspect string_v1 fix --pick 300 -20 300
#   ros2 run bed_inspect string_v1 unfix --place 300 -20 300
#   옵션: --spiral inward|outward, --virtual, --yes, --home j1..j6
# 설정은 string_v1_config (환경변수 STR1_...). v2 는 팀원 담당 (ho3 커밋 654349d).

import argparse
import math
import sys
import time

import rclpy
from rclpy.utilities import remove_ros_args
import DR_init

from bed_inspect import bed_config as cfg
from bed_inspect import string_v1_config as sc
from bed_inspect import string_v1_plan as plan

DR_init.__dsr__id = cfg.ROBOT_ID
DR_init.__dsr__model = cfg.ROBOT_MODEL


def fmt(q):
    return ' '.join(f'{v:9.2f}' for v in q)


def print_points(action, p):
    print(f'  [v1] 모드 {action}, 감기 {sc.SPIRAL_MODE}, TCP {sc.STR_TCP}, 탁자 z {sc.TABLE_Z}, 감는 나사 {sc.POST_XY}, '
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
        elif k == 'unwind_halves':
            print(f'  unwind    풀기 반원 {len(v)}개 (반지름 키우며, 위에서 {"시계" if sc.SPIRAL_CCW else "반시계"})')
            for i, (via, end) in enumerate(v):
                print(f'    {i + 1:2d} via {fmt(via[:3])}  end {fmt(end[:3])}')
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
        self.hole_ref = None                # 풀 때 잰 실제 구멍 자리 (x, y, 나사 꽂혀 있던 잡는 높이 z)
        self._pushing = False

    def line(self, p, what, slow=False):
        v, a = (sc.SLOW_VEL, sc.SLOW_ACC) if slow else (sc.MOVE_VEL, sc.MOVE_ACC)
        self.r.movel_to(p, v, a, what)

    def tool_move(self, dz, rz, what, vel=None, acc=None):
        """툴 기준 상대 이동 (툴 Z 는 아래 방향): dz [mm], rz [deg]."""
        self.r.require_standby(what)
        self.r._begin(what)
        ret = self.dr.movel(self.r.posx([0.0, 0.0, float(dz), 0.0, 0.0, float(rz)]),
                            vel=list(vel or sc.TIGHTEN_VEL), acc=list(acc or sc.TIGHTEN_ACC),
                            ref=self.dr.DR_TOOL, mod=self.dr.DR_MV_MOD_REL)
        self.r._done(ret, what)

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

    def r_error(self, msg):
        from bed_inspect.bed_robot import BedError
        return BedError(msg)

    def spiral(self, sp):
        from dsr_msgs2.srv import MoveSpiral
        req = MoveSpiral.Request()
        req.vel = [float(v) for v in sc.SPIRAL_VEL]
        req.acc = [float(v) for v in sc.SPIRAL_ACC]
        req.time = 0.0
        req.task_axis = 2
        req.ref = 0
        req.mode = 0
        what = 'move_spiral 감기'
        self.r.require_standby(what)
        self.r._begin(what)
        if sp['mode'] == 'outward':
            req.revolution = float(sp['rev'])
            req.max_radius = float(sp['rmax'])
            req.max_length = float(sp['lmax'])
            req.target_pos = [0.0, 0.0, 0.0]
            req.sync_type = 0
            if not self._spiral_call(req, timeout=120.0):
                raise self.r_error(f'{what} 실패. {self.r.fault_text()}')
            self.r.require_standby(what)
            return
        if self.virtual:
            # 에뮬레이터는 목표점 중심 안쪽 move_spiral(_ex) 을 받으면 컨트롤러가 멈춘다 (11-21 확인, 실물은 정상)
            # → RViz 확인용으로 같은 나선 길(반지름 30 → 20, z +8)을 점으로 이어 movesx
            pts = sp['points'] if 'points' in sp else plan.spiral_points(sp)
            self.log(f'  [가상] move_spiral 대신 같은 나선 길을 점 {len(pts)}개로 이어 그림')
            ret = self.dr.movesx([self.r.posx(q) for q in pts], vel=list(sc.SPIRAL_VEL), acc=list(sc.SPIRAL_ACC),
                                 ref=self.dr.DR_BASE)
            self.r._done(ret, what)
            xyz = self.r.cur_xyz()
            self.log(f'  감기 끝(가상): 반지름 {math.hypot(xyz[0] - sc.POST_XY[0], xyz[1] - sc.POST_XY[1]):.1f}mm, z {xyz[2]:.1f}')
            return
        req.revolution = float(sp['rev'])
        req.max_radius = 0.0
        req.max_length = 0.0
        req.target_pos = [float(v) for v in sp['target']]
        req.spiral_dir = 1
        ccw = sp.get('ccw', sc.SPIRAL_CCW)
        req.rot_dir = sc.SPIRAL_ROT_DIR_CCW if ccw else 1 - sc.SPIRAL_ROT_DIR_CCW
        req.sync_type = 1
        watch = plan.SpiralWatch(sc.POST_XY, sp['stop_deg'], ccw=ccw)
        if not self._spiral_call(req, timeout=5.0):
            raise self.r_error(f'{what} 시작 실패. {self.r.fault_text()}')
        t0, busy = time.monotonic(), False
        limit = 3.0 * (2 * math.pi * sc.SPIRAL_R_OUT * sp['rev']) / sc.SPIRAL_VEL[0] + 10.0
        t_log, last_xyz, t_moved = t0, None, t0
        try:
            while True:
                self.r.check_stop()
                xyz = self.r.cur_xyz()
                xy = xyz[:2]
                res = watch.update(xy)
                now = time.monotonic()
                if last_xyz is None or max(abs(a - b) for a, b in zip(xyz, last_xyz)) > 0.5:
                    last_xyz, t_moved = xyz, now
                if now - t_log >= 2.0:
                    t_log = now
                    self.log(f'    감는 중 {now - t0:4.0f}s: {watch.swept:5.0f}도, 반지름 {watch.r:5.1f}mm, '
                             f'z {xyz[2]:6.1f}, 상태 {self.dr.check_motion()}')
                if now - t_moved > 5.0:
                    self.r.halt()
                    raise self.r_error(f'{what}: 5초 동안 움직이지 않아 멈췄습니다 ({watch.swept:.0f}도, 반지름 {watch.r:.1f}mm). '
                                       f'{self.r.fault_text()}')
                if res == 'stop':
                    self.r.stop_motion()
                    break
                if res in ('too_close', 'wrong_dir'):
                    self.r.halt()
                    raise self.r_error(
                        f'{what}: ' + ('감는 나사에 너무 가까워져' if res == 'too_close' else
                                       '반대 방향으로 돌아 (rot_dir 부호 확인: string_v1_config.SPIRAL_ROT_DIR_CCW)')
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

    def j6_prepare(self, turn_deg, what):
        """돌리기 전에 J6 를 미리 돌려 둔다 (11-22 형님 방식): turn_deg 반대쪽 절대각 J6_PRESET 으로.
        예) +360 돌릴 거면 J6 -180 → 돌린 뒤 +180. -360 이면 +180 → -180. J6 범위(±360) 안에서 한 번에 돈다."""
        j = self.r.cur_posj()
        want = -math.copysign(sc.J6_PRESET, turn_deg) if abs(turn_deg) >= 2 * sc.J6_PRESET - 1 else plan.j6_start(j[5], turn_deg)
        if abs(want + turn_deg) > sc.J6_LIMIT:
            raise self.r_error(f'{what}: J6 {want:.0f} 에서 {turn_deg:+.0f}도 돌리면 J6 범위를 넘습니다.')
        if abs(want - j[5]) < 0.5:
            return
        self.log(f'  {what}: J6 {j[5]:.1f} → {want:.1f} 미리 돌림 (그 다음 {turn_deg:+.0f}도)')
        self.r.require_standby(what)
        self.r._begin(what)
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

    def ratchet(self, reps, sign, what, push=True, loosen=False, first_deg=None):
        """잡은 상태에서 시작 (J6 는 j6_prepare 로 미리 돌려 둠). 한 번 =
        잡은 채로 TURN 만큼 돌리기 → 놓기 → 살짝 위 → 그리퍼만 되돌리기 → 나사 높이로 내려가 다시 잡기.
        조이기(loosen=False): 순응 제어(x·y 부드럽게 → 구멍에 제대로 자리잡게) + 아래로 누르며. 마지막엔 놓고 위로.
        풀기(loosen=True): z 만 부드럽게(나사가 올라오는 만큼 따라감), 누르지 않음. 마지막엔 잡은 채로 끝 (뽑기 위해)."""
        deg = sign * sc.TIGHTEN_TURN_DEG
        zs = []
        for i in range(reps):
            if first_deg is not None:
                deg = sign * (first_deg if i == 0 else sc.TIGHTEN_TURN_DEG)
            last = i == reps - 1
            self.log(f'  {what} {i + 1}/{reps}: {deg:+.0f}도' + (f' (아래로 {sc.TIGHTEN_PUSH_N:g}N)' if push and not loosen else ''))
            if loosen:
                self._pushing = False
                if self.dr.task_compliance_ctrl(list(sc.LOOSEN_COMPLIANCE)) != 0:
                    raise self.r_error(f'{what}: 순응 제어를 켜지 못했습니다. {self.r.fault_text()}')
                self.r.sleep(sc.TIGHTEN_SETTLE)
            else:
                self.press_on(what, push)
            z_start = self.r.cur_z()
            try:
                self.turn(deg, f'{what} 회전', advance=-1 if loosen else 1)
                z_end = self.r.cur_z()
            finally:
                self.press_off()
            if not loosen and i == 0 and z_start - z_end < sc.TIGHTEN_MIN_ADVANCE:
                # 한 바퀴 돌렸는데 피치만큼 안 내려감 = 표면에서 헛돎 (11-22 실물)
                # 11-22 형님: 실제로는 얼추 고정된 경우가 있음 → 멈추지 말고 경고만 남기고 그리퍼를 풀어 빠져나간다
                self.log(f'    ※ 경고: {abs(deg):.0f}도 돌렸는데 {z_start - z_end:.2f}mm 만 내려감 (나사산 1바퀴 {sc.SCREW_PITCH}mm). '
                         '헛돌았을 수 있으니 고정됐는지 눈으로 확인하세요 → 그리퍼 풀고 빠짐')
                last = True
            if not loosen:
                zs.append(z_end)
                if len(zs) >= 3 and zs[-3] - zs[-1] < 0.3:
                    # 두 번 돌렸는데 머리가 안 내려옴 → 나사산이 안 물림 (구멍을 못 찾았거나 끝까지 들어감)
                    if z_end <= sc.HOLE_GRIP_Z + 3.0:
                        self.log(f'    머리 z {z_end:.1f} 에서 더 안 들어감 → 다 조인 것으로 봄 ({i + 1}번)')
                        last = True
                    else:
                        raise self.r_error(f'{what}: 두 번 돌려도 나사가 안 들어갑니다 (머리 z {z_end:.1f}). 구멍을 못 찾았거나 '
                                           f'나사산이 안 물렸습니다. 그리퍼는 잡은 채로 둡니다.')
            if not loosen and z_end <= sc.HOLE_GRIP_Z + sc.TIGHTEN_DONE_TOL:
                self.log(f'    머리 z {z_end:.1f} → 끝까지 들어감 ({i + 1}번)')
                last = True
            elif not loosen and i % 4 == 3:
                self.log(f'    머리 z {z_end:.1f} (목표 {sc.HOLE_GRIP_Z:.1f})')
            if loosen and self.pull_free(what):
                return True                     # 빠짐: 잡은 채로 → 부르는 쪽이 뽑는다
            if last and loosen:
                return False                    # 최대 횟수까지 돌렸는데도 안 빠짐
            self.r.grip(sc.OPEN_KEY)
            self.r.move_z(sc.TIGHTEN_LIFT, vel=sc.SLOW_VEL, acc=sc.SLOW_ACC)
            if last:
                break
            self.turn(-deg, f'{what} 그리퍼 되돌리기')
            # 나사가 내려간(풀 때는 올라온) 높이 그대로 가서 머리를 다시 잡는다 (돌린 만큼 피치로 이미 따라갔음)
            self.r.move_z(z_end - self.r.cur_z(), vel=sc.SLOW_VEL, acc=sc.SLOW_ACC)
            self.r.grip(sc.GRIP_KEY)

    def read_mz(self, n=5):
        """툴 Z(=베이스 Z) 둘레 반발 토크 평균 [Nm] (나사산이 물리면 커짐)."""
        vals = []
        for _ in range(n):
            f = self.dr.get_tool_force(ref=self.dr.DR_BASE)
            if f != -1 and f is not None and len(f) >= 6:
                vals.append(f[5])
        return sum(vals) / len(vals) if vals else 0.0

    def approach_hole(self, over_z, what='구멍으로'):
        """잡은 나사를 구멍에 다시 꽂으러 간다 (11-22 형님: 풀 때 잰 좌표 기준).
        구멍 위(over_z) → J6 미리 돌림 → 기준 z + 나사산 1바퀴 + ENTRY_GAP 까지 위치로 천천히 →
        그 다음은 순응 제어(누르지 않음)로 조금씩 내려가며 z 힘이 CONTACT_N 늘면 닿은 것으로 멈춘다.
        (11-22 실물: 힘 제어로 내려가기만 믿었더니 허공에서 닿았다고 보고 돌리다 놓아 버림)"""
        # x·y 는 설정값 구멍 좌표(HOLE_XY) 기준 (11-22 형님: fix·unfix 모두 이 좌표), 높이는 풀 때 잰 값
        ref_z = self.hole_ref[2] if self.hole_ref is not None else sc.HOLE_GRIP_Z + sc.SCREW_PITCH
        ref = [sc.HOLE_XY[0], sc.HOLE_XY[1], ref_z]
        if self.hole_ref is not None:
            self.log(f'    구멍 x·y {sc.HOLE_XY} (풀 때 잡혀 있던 실제 x·y {self.hole_ref[0]:.2f}, {self.hole_ref[1]:.2f}), 높이 기준 z {ref_z:.2f}')
        self.line(plan.pose(ref[:2], over_z), f'{what} 위', slow=True)
        self.j6_prepare(sc.TIGHTEN_SIGN * sc.TIGHTEN_TURN_DEG, '돌릴 준비')
        # 기준 z 는 360도 다 풀고 난 높이 = 나사 끝이 나사산 시작에 걸쳐 있던 높이 → 여기서 닿을 것으로 예상
        z_entry = ref[2]
        # 위치(단단하게)로는 나사 끝이 판 윗면 INSERT_GAP 위까지만 — 그 아래(구멍 안)는 순응 제어로 조금씩
        # (시뮬 11-22: x·y 가 1mm 어긋난 채 위치로 구멍 안까지 내리면 가장자리에 걸림)
        z_pos = max(z_entry + sc.ENTRY_GAP, sc.TABLE_Z + sc.SCREW_LEN + sc.INSERT_GAP)
        self.r.move_z(z_pos - self.r.cur_z(), vel=sc.SLOW_VEL, acc=sc.SLOW_ACC)
        if self.dr.task_compliance_ctrl(list(sc.CONTACT_COMPLIANCE)) != 0:
            raise self.r_error(f'{what}: 순응 제어를 켜지 못했습니다. {self.r.fault_text()}')
        try:
            self.r.sleep(sc.TIGHTEN_SETTLE)
            f0 = self.r.read_force_vec(5)[2]
            # z 1N/mm 순응이라 닿은 뒤 CONTACT_N 만큼 힘이 생기려면 그만큼 더 내려가라는 명령이 필요 → 여유 +1mm
            z_limit = ref[2] - (sc.CONTACT_N / (sc.CONTACT_COMPLIANCE[2] / 1000.0) + 1.0)
            n_max = int((z_pos - z_limit) / sc.CONTACT_STEP) + 20     # 실제 높이가 안 내려가도(걸림) 끝나게
            for _ in range(n_max):
                f = self.r.read_force_vec(3)[2]
                if abs(f - f0) >= sc.CONTACT_N:
                    self.log(f'    닿음: z {self.r.cur_z():.2f} (예상 {z_entry:.2f}), z 힘 변화 {abs(f - f0):.1f}N')
                    break
                if self.r.cur_z() <= z_limit:
                    raise self.r_error(f'{what}: 기준 높이(z {ref[2]:.1f}) 아래까지 내려가도 닿지 않았습니다. '
                                       f'나사를 잡고 있는지 확인하세요. 그리퍼는 잡은 채로 둡니다.')
                self.r.move_z(-sc.CONTACT_STEP, vel=sc.HOLE_SEARCH_VEL[0], acc=10.0)
            else:
                raise self.r_error(f'{what}: {n_max}번 내려가도 닿는 힘이 안 잡힙니다 (지금 z {self.r.cur_z():.1f}). '
                                   '힘 센서/나사를 확인하세요. 그리퍼는 잡은 채로 둡니다.')
        finally:
            self.dr.release_compliance_ctrl()
        return ref

    def find_hole(self, what='구멍 찾기'):
        """approach_hole 로 닿은 뒤 (11-22 실물 3차 반영):
          1) 닿은 높이가 기준(풀 때 잰 나사산 시작 높이)보다 SEAT_TOL 넘게 높으면 = 구멍 가장자리/표면에 얹힘
             → 누른 채로 작은 원(0.5~2mm)을 그리며 기준 높이까지 내려가는 자리를 찾는다
          2) 나사산 시작 찾기: 누른 채로 푸는 방향으로 살살(10도씩, 최대 CLICK_MAX) 돌리다 '딸깍' 내려가면 거기가 시작점
             (못 느껴도 그대로 진행)
        돌려주는 값: 푸는 방향으로 돌린 각도(조이기는 그대로 360, J6 은 그만큼 더 여유 있음)."""
        ref_z = self.hole_ref[2] if self.hole_ref else sc.HOLE_GRIP_Z + sc.SCREW_PITCH
        self.press_on(what, True)
        try:
            self.r.sleep(sc.TIGHTEN_SETTLE)
            z_contact = self.r.cur_z()
            self.log(f'    누른 높이 z {z_contact:.2f} (나사산 시작 기준 {ref_z:.2f})')
            if z_contact > ref_z + sc.SEAT_TOL:
                self.log(f'    기준보다 {z_contact - ref_z:.2f}mm 높음 → 구멍 가장자리에 얹힘, 작은 원으로 자리 찾기')
                x0, y0 = self.r.cur_xyz()[:2]
                found = False
                for rr in sc.SEAT_SEARCH_RINGS:
                    n = max(6, int(2 * math.pi * rr / sc.SEAT_SEARCH_STEP))
                    for k in range(n + 1):
                        a = 2 * math.pi * k / n
                        cx, cy = self.r.cur_xyz()[:2]
                        self.r.require_standby(what)
                        self._ret = self.dr.movel(self.r.posx([x0 + rr * math.cos(a) - cx, y0 + rr * math.sin(a) - cy, 0.0, 0.0, 0.0, 0.0]),
                                                  vel=list(sc.HOLE_SEARCH_VEL), acc=[20.0, 20.0],
                                                  ref=self.dr.DR_BASE, mod=self.dr.DR_MV_MOD_REL)
                        self.r._done(self._ret, f'{what} 원')
                        self.r.sleep(0.2)
                        z = self.r.cur_z()
                        if z <= ref_z + sc.SEAT_TOL:
                            self.log(f'    자리 찾음: 반지름 {rr:g}mm (z {z_contact:.2f} → {z:.2f})')
                            found = True
                            break
                    if found:
                        break
                if not found:
                    raise self.r_error(f'{what}: 반지름 {sc.SEAT_SEARCH_RINGS[-1]:g}mm 안에서 구멍 자리를 못 찾았습니다 '
                                       f'(계속 기준보다 높음). 그리퍼는 잡은 채로 둡니다.')
            # 2) 나사산 시작 찾기 (푸는 방향 살살)
            z0 = self.r.cur_z()
            back = 0.0
            while back < sc.CLICK_MAX:
                self.tool_move(0.0, -sc.TIGHTEN_SIGN * sc.HOLE_ROT_STEP, f'{what} 나사산 시작 찾기', vel=sc.HOLE_ROT_VEL)
                back += sc.HOLE_ROT_STEP
                z = self.r.cur_z()
                if z0 - z >= sc.CLICK_DROP:
                    self.log(f'    나사산 시작 느낌: 푸는 쪽으로 {back:.0f}도에서 {z0 - z:.2f}mm 딸깍')
                    break
            else:
                self.log(f'    푸는 쪽으로 {sc.CLICK_MAX:.0f}도 돌려도 딸깍 없음 → 그대로 조이기')
            return back
        finally:
            self.press_off()

    def pull_free(self, what):
        """잡은 채로 살짝(PULL_TEST_DZ) 들어 보고 z 힘 변화로 나사가 구멍에서 빠졌는지 본다. 다시 원래 높이로 내려놓는다."""
        if self.dr.task_compliance_ctrl(list(sc.PULL_TEST_COMPLIANCE)) != 0:
            raise self.r_error(f'{what}: 순응 제어를 켜지 못했습니다. {self.r.fault_text()}')
        try:
            self.r.sleep(sc.TIGHTEN_SETTLE)
            f0 = self.r.read_force_vec(5)[2]
            # 11-22 형님: 다 풀고 살짝 들기 직전 좌표 = 나사가 구멍에 꽂혀 있던 실제 자리 → 다시 꽂을 때 기준
            self.hole_ref = self.r.cur_xyz()
            self.log(f'    구멍 기준 좌표 기억: x {self.hole_ref[0]:.2f} y {self.hole_ref[1]:.2f} z {self.hole_ref[2]:.2f}')
            self.r.move_z(sc.PULL_TEST_DZ, vel=sc.SLOW_VEL, acc=sc.SLOW_ACC)
            f1 = self.r.read_force_vec(5)[2]
            self.r.move_z(-sc.PULL_TEST_DZ, vel=sc.SLOW_VEL, acc=sc.SLOW_ACC)
        finally:
            self.dr.release_compliance_ctrl()
        df = abs(f1 - f0)
        self._last_pull_df = df
        free = df < sc.PULL_TEST_N
        self.log(f'    들어보기: z 힘 변화 {df:.1f}N → {"빠짐" if free else "아직 물려 있음"}')
        return free

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
            self.log(f'   잡은 채로 반대로 {sc.TIGHTEN_TURN_DEG:.0f}도씩 빠질 때까지 풀기 (매번 살짝 들어 확인, 최대 {sc.LOOSEN_MAX_REPS}번)')
            if not self.ratchet(sc.LOOSEN_MAX_REPS, -sc.TIGHTEN_SIGN, '풀기', push=False, loosen=True):
                if sc.LOOSEN_STRICT:
                    raise self.r_error(f'{what}: {sc.LOOSEN_MAX_REPS}번 돌려도 나사가 안 빠졌습니다. 그리퍼는 잡은 채로 둡니다.')
                # 11-22 실물: 실제로는 빠졌는데 실 장력 때문에 들어보기 힘이 2.2N 으로 나와 멈췄음.
                # 360도면 풀리게 해 두었으므로 들어보기는 참고만 하고 그대로 뽑는다.
                self.log('    들어보기 힘은 실 장력 영향일 수 있음 → 360도 돌렸으니 그대로 뽑음')
        if sc.LOOSEN_REPS > 0:   # 11-22: 조인 각도가 360 보다 조금 많으면 360 으로 덜 풀림 → 항상 조금 더 푼다 (빠진 나사는 더 돌려도 됨)
            # 들어볼 때 조금이라도 버팀 → 덜 풀렸을 수 있음 (시뮬 11-22: 3N 남은 채 당겨서 손가락에서 빠짐)
            # 다 풀린 나사를 더 돌려도 상관없으므로 EXTRA 만큼 더 풀고 뽑는다 (J6 범위 안)
            self.log(f'    (들어볼 때 {getattr(self, "_last_pull_df", 0.0):.1f}N) 확실히 빠지게 {sc.LOOSEN_EXTRA_DEG:.0f}도 더 풀기')
            if self.dr.task_compliance_ctrl(list(sc.LOOSEN_COMPLIANCE)) != 0:
                raise self.r_error(f'{what}: 순응 제어를 켜지 못했습니다. {self.r.fault_text()}')
            try:
                self.r.sleep(sc.TIGHTEN_SETTLE)
                self.turn(-sc.TIGHTEN_SIGN * sc.LOOSEN_EXTRA_DEG, '더 풀기', advance=-1)
            finally:
                self.dr.release_compliance_ctrl()
        self.log('   위로 뽑기')
        self.r.move_z(out_z - self.r.cur_z(), vel=sc.SLOW_VEL, acc=sc.SLOW_ACC)

    def toward_tension(self, end, what='살짝 팽팽하게'):
        """end 쪽으로 천천히 가다가 수평 외력 변화가 TENSION_N 을 넘으면(실이 살짝 팽팽) 멈춘다. 못 느끼면 end 까지."""
        self.r.require_standby(what)
        self.r._begin(what, end)
        f0 = self.r.read_force_vec(5)
        if self.dr.amovel(self.r.posx([float(v) for v in end]), vel=list(sc.TENSION_VEL), acc=[40.0, 40.0],
                          ref=self.dr.DR_BASE, mod=self.dr.DR_MV_MOD_ABS) != 0:
            raise self.r_error(f'{what} 시작 실패. {self.r.fault_text()}')
        t0, busy, df, peak = time.monotonic(), False, 0.0, 0.0
        try:
            while True:
                self.r.check_stop()
                f = self.r.read_force_vec(1)
                df = math.hypot(f[0] - f0[0], f[1] - f0[1])
                peak = max(peak, df)
                if df >= sc.TENSION_N:
                    self.r.stop_motion()
                    self.log(f'  장력 {df:.1f}N 에서 멈춤')
                    break
                st = self.dr.check_motion()
                busy = busy or st == self.dr.DR_STATE_BUSY
                if st == self.dr.DR_STATE_IDLE and (busy or time.monotonic() - t0 > 1.0):
                    self.log(f'  끝까지 감 (최대 {peak:.1f}N, 기준 {sc.TENSION_N:g}N 못 넘음)')
                    break
                time.sleep(0.02)
        except BaseException:
            if self.dr.check_motion() != self.dr.DR_STATE_IDLE:
                self.r.halt()
            raise
        self.r.require_standby(what)

    def fix(self, p):
        sp = p['spiral']
        self.log('1) 실 끝 나사 잡고 돌려 풀어서 뽑기')
        self.unscrew_out(p['pick_app'], p['pick'][2], p['pick_up'][2], '실 끝 나사')
        self.log('2) 감는 나사 옆으로 (블록 위로 지나감)')
        self.line(p['spiral_over'], '감기 시작점 위')
        self.line(p['spiral_start'], '감기 시작점', slow=True)
        self.log(f'3) move_spiral 로 {sc.SPIRAL_TURNS:g}바퀴 감기 ({sp["mode"]})')
        self.spiral(sp)
        self.log(f'4) 구멍 방향 그대로, 구멍보다 더 멀리 (최대 {sc.OVERSHOOT_MAX:g}mm) 실이 살짝 팽팽해질 때까지')
        self.r.move_z(p['beyond'][2] - self.r.cur_z())
        self.toward_tension(p['beyond'])
        self.log('5) 팽팽한 채로 구멍 위로 → 순응 제어로 구멍에 맞추고 조이기')
        self.approach_hole(p['hole_over'][2])
        turned = self.find_hole()
        self.log(f'6) {sc.TIGHTEN_TURN_DEG:.0f}도씩 머리가 z {sc.HOLE_GRIP_Z:.1f} 까지 내려올 때까지 조이기 (최대 {sc.TIGHTEN_REPS}번, 순응 제어 + 아래로 {sc.TIGHTEN_PUSH_N:g}N)')
        self.ratchet(sc.TIGHTEN_REPS, sc.TIGHTEN_SIGN, '조이기', first_deg=sc.TIGHTEN_TURN_DEG)
        self.line(p['hole_app'], '구멍 위로 빠지기')

    def unwind_spiral(self, what='풀기'):
        """해제 (04:30 형님): 감기 시작 위치에서 z 올린 뒤 → 같은 방향 반지름 UNWIND_R0(95) 자리로 →
        감는 나사 중심으로 감은 반대 방향 SPIRAL_TURNS 바퀴, 반지름 95 → 110, z +UNWIND_SPIRAL_RISE (반원 moveC 이어서)."""
        c = self.r.cur_xyz()
        d0 = math.degrees(math.atan2(c[1] - sc.POST_XY[1], c[0] - sc.POST_XY[0]))
        start, halves = plan.unwind_grow(d0, c[2])
        self.log(f'    반지름 {sc.UNWIND_R0:g}mm 자리로 (x {start[0]:.1f}, y {start[1]:.1f})')
        self.line(start, '풀기 시작 반지름으로', slow=True)
        self.log(f'    감는 나사 중심, 반지름 {sc.UNWIND_R0:g} → {sc.UNWIND_R1:g}mm 커지며, z {c[2]:.1f} → {halves[-1][1][2]:.1f} '
                 f'(두 번째 바퀴부터 바퀴마다 +{sc.UNWIND_STEP_Z:g}), '
                 f'{sc.UNWIND_VEL[0]:g}mm/s')
        for i, (via, end) in enumerate(halves):
            self.r.require_standby(what)
            self.r._begin(what, end)
            ret = self.dr.movec(self.r.posx(via), self.r.posx(end), vel=list(sc.UNWIND_VEL), acc=list(sc.UNWIND_ACC),
                                ref=self.dr.DR_BASE, mod=self.dr.DR_MV_MOD_ABS)
            self.r._done(ret, f'{what} 반원 {i + 1}/{len(halves)}')

    def unfix(self, p):
        self.log('1) 가운데 구멍의 나사 잡고 돌려 풀어서 뽑기')
        self.unscrew_out(p['hole_app'], p['hole_grip'][2], p['hole_pull'][2], '가운데 구멍 나사')
        self.log('2) move spiral 시작 위치 위로')
        self.line(p['unwind_start'], 'move spiral 시작 위치 위')
        # 11-22 16:37 형님: z 를 먼저 올리지 말고, 지금 높이 그대로 풀기 시작 반지름(UNWIND_R0) 자리까지 간 뒤 올리고 돈다
        c = self.r.cur_xyz()
        d0 = math.degrees(math.atan2(c[1] - sc.POST_XY[1], c[0] - sc.POST_XY[0]))
        r0_xy = plan.around(sc.POST_XY, sc.UNWIND_R0, d0)
        self.log(f'3) 높이 그대로 반지름 {sc.UNWIND_R0:g}mm 자리 ({r0_xy[0]:.1f}, {r0_xy[1]:.1f}) 로 간 뒤 z {sc.UNWIND_SPIRAL_Z:g} 로 수직 상승')
        self.line(plan.pose(r0_xy, c[2]), '풀기 시작 반지름으로 (낮은 높이)')
        self.r.move_z(sc.UNWIND_SPIRAL_Z - self.r.cur_z())
        self.log(f'4) 감은 반대 방향 {sc.UNWIND_TURNS:g}바퀴: 감는 나사 둘레 반지름 {sc.UNWIND_R0:g} → {sc.UNWIND_R1:g}mm 커지며 z +{sc.UNWIND_SPIRAL_RISE:g} 올라가며 풀기 (moveC)')
        self.unwind_spiral()
        z = self.r.cur_z()
        self.log(f'5) z {z:.1f} 그대로 ({sc.UNFIX_TAUT_XY[0]}, {sc.UNFIX_TAUT_XY[1]}) 로 살짝 팽팽하게')
        self.line(plan.pose(sc.UNFIX_TAUT_XY, z), '팽팽하게')
        self.log(f'6) z 그대로 ({sc.UNFIX_SWEEP_XY[0]}, {sc.UNFIX_SWEEP_XY[1]}) 로')
        self.line(plan.pose(sc.UNFIX_SWEEP_XY, z), '바깥으로 크게')
        for i, (pj, px) in enumerate(sc.UNFIX_EXIT):
            name = f'p{9 + i}'
            self.log(f'7) 관절 이동 {name} (z {px[2]:.0f})')
            self.r.require_standby(name)
            self.r._begin(name, px)
            self.r._done(self.dr.movej(self.r.posj(list(pj)), vel=sc.EXIT_VELJ, acc=sc.EXIT_ACCJ), name)
        self.log('8) 중간 나사 구멍으로 와서 순응 제어로 맞추고 꽂아 두기')
        self.approach_hole(p['place_over'][2])
        self.find_hole()
        self.ratchet(sc.TIGHTEN_REPS, sc.TIGHTEN_SIGN, '다시 꽂기', first_deg=sc.TIGHTEN_TURN_DEG)
        self.line(p['place_app'], '구멍 위로 빠지기')

def main(args=None):
    ap = argparse.ArgumentParser(description='[v1 영상 전 버전] 나사로 실 고정/고정해제')
    ap.add_argument('action', choices=['fix', 'unfix', 'points'])
    ap.add_argument('--pick', type=float, nargs=3, help='고정: 실 끝 나사를 잡는 TCP 위치 x y z')
    ap.add_argument('--place', type=float, nargs=3, help='고정해제: 풀고 나서 나사를 놓을 위치 x y z (없으면 --pick)')
    ap.add_argument('--spiral', choices=['inward', 'outward'])
    ap.add_argument('--home', type=float, nargs=6, default=cfg.HOME_POSJ)
    ap.add_argument('--virtual', action='store_true')
    ap.add_argument('--yes', action='store_true')
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
        print('[v1 고정]')
        print_points('fix', fixp)
    if unfixp and opt.action in ('unfix', 'points'):
        print('[v1 고정해제]')
        print_points('unfix', unfixp)
    if opt.action == 'points':
        if fixp is None and unfixp is None:
            print('--pick 또는 --place 를 주면 좌표를 출력합니다.')
        return

    if opt.virtual:
        cfg.VIRTUAL = True                      # 에뮬레이터: TCP 자동 설정 건너뜀 (에뮬레이터엔 GripperDA_v1 없음)
        cfg.GRIP_WIDTH['screw_open'] = 110    # RViz 에서 열고 닫는 게 잘 보이게 가상에서는 끝까지 벌림 (실물은 25mm)
    cfg.GRIPPER_MODE = sc.GRIPPER_MODE          # 나사 돌리기는 센 그리퍼 (bed 공정 설정과 따로)
    rclpy.init(args=args)
    node = rclpy.create_node('string_v1_demo', namespace=cfg.ROBOT_ID)
    DR_init.__dsr__node = node
    import DSR_ROBOT2 as dr
    from bed_inspect.bed_robot import BedError, BedRobot

    log = node.get_logger().info
    robot = BedRobot(dr, node, log)
    demo = StringDemo(robot, dr, node, log)
    demo.virtual = opt.virtual
    home = {'name': cfg.P_HOME, 'posj': list(opt.home)}
    rc = 0
    try:
        robot.wait_services()
        tcp = robot.prepare()
        log(f'[v1] TCP: {tcp or "(없음)"}, 그리퍼 모드 {cfg.GRIPPER_MODE}, 감기 {sc.SPIRAL_MODE}')
        if sc.STR_TCP != tcp:
            msg = f'현재 TCP("{tcp or "없음"}")가 좌표 기준 TCP("{sc.STR_TCP}")와 다릅니다.'
            if not opt.virtual:
                raise BedError(msg + ' 펜던트에서 TCP 를 바꾼 뒤 다시 실행하세요. (RViz 가상 모드면 --virtual)')
            node.get_logger().warn(msg + ' --virtual 이라 계속합니다.')
        if not opt.yes:
            input(f'[v1] Enter 를 누르면 {"고정" if opt.action == "fix" else "고정해제"} 동작을 시작합니다 (Ctrl+C 취소) > ')
        robot.movej_to(home)
        if opt.action == 'fix':
            log('=== [v1] 나사로 실 고정 ===')
            demo.fix(fixp)
        else:
            log('=== [v1] 실 고정해제 ===')
            demo.unfix(unfixp)
        robot.movej_to(home)
        log('끝.')
    except BedError as e:
        node.get_logger().error(str(e))
        rc = 1
    except (KeyboardInterrupt, EOFError):
        rc = 130
        print()
        node.get_logger().warn('중단됨 (로봇이 아직 움직이면 펜던트에서 정지하세요)')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)        # 실패/중단이면 0 이 아님 → 스크립트가 다음 동작(unfix)으로 넘어가지 않음


if __name__ == '__main__':
    main()
