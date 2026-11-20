# 가짜 로봇 + 가짜 침대. 실제 로봇 없이 공정 전체와 화면을 돌려 볼 수 있다.
#
#   ros2 run bed_inspect bed_sim                      # 전부 정상인 침대
#   ros2 run bed_inspect bed_sim --fault 2,5          # 2번, 5번 블록이 불량(무름)
#   ros2 run bed_inspect bed_sim --speed 20           # 동작을 20배 빠르게 (기본 5배)
#
#   그 다음 다른 터미널에서
#   ros2 run bed_inspect bed_process ~/ws_cobot_pjt/teach_points/bed_sim.json
#   ros2 run bed_inspect bed_hmi
#
# 하는 일
#   - 두산 드라이버와 같은 이름의 서비스를 제공한다 (/dsr01/dsr_controller2/...):
#     movej, movel, movec, move_stop, 현재 위치, 힘 읽기, 디지털 출력, 그리퍼(/onrobot/sendCommand)
#   - 침대의 물리를 흉내 낸다: 누르면 블록 강성만큼 힘이 나오고(노이즈 포함),
#     커버를 열어야 블록을 집을 수 있다.
#   - 공정이 말이 안 되는 동작을 하면 "위반"으로 기록한다
#     (커버를 닫은 채 블록 집기, 블록 떨어뜨리기, 경첩을 억지로 끌기 등).
#   - 시작할 때 가짜 교시 파일(bed_sim.json)을 만들고, 가짜용 재고 기록을 처음 상태로 되돌린다.
#     → bed_sim 을 다시 켰으면 bed_process 도 다시 켠다.
#
# 진짜 로봇과 다른 점: 관절-좌표 계산이 가짜(단순 비례식)라서 자세가 실제와 다르다.
#   공정 순서, 힘 측정 계산, 화면을 확인하는 용도이고, 실제 충돌 여부는 알 수 없다.

import argparse
import json
import math
import os
import random
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float64MultiArray, String

from bed_inspect import bed_config as cfg
from bed_inspect import bed_math
from bed_inspect.bed_tasks import make_cover_path

# ── 가짜 세계의 치수 (mm) ─────────────────────────────────────────────
SAFE_Z = 180.0                  # 침대/보관대/버리는 곳 교시 높이
GRIP_DEPTH = 15.0               # 블록 윗면에서 이만큼 아래를 잡는다
FLOOR_Z = SAFE_Z + cfg.BED_PICK_DZ + GRIP_DEPTH - cfg.H_REF     # 칸 바닥 높이 (설정값과 맞아떨어지게)
DISCARD_XY = (150.0, -280.0)
DOWN = [0.0, 180.0, 0.0]        # 툴이 아래를 보는 자세
POSES = {
    cfg.P_HOME: [350.0, 0.0, 350.0] + DOWN,
    cfg.P_DISCARD: [DISCARD_XY[0], DISCARD_XY[1], SAFE_Z] + DOWN,
}
# 커버: 교시 파일에 넣지 않는다(공정이 bed_config 좌표로 움직임). 가짜 세계의 경첩과 고리는 공정과 같은 계산으로 만든다.
COVER_CLOSED, COVER_REST = 'cover_closed', 'cover_rest'
_COVER = make_cover_path()     # 공정과 같은 계산 (bed_config 의 COVER_MODE)
COVER_POSES = {COVER_CLOSED: _COVER['grip'], COVER_REST: _COVER['rest']}
HINGE = _COVER['hinge']
# 가짜 뚜껑: 고리는 경첩 축(HINGE, 베이스 X 방향) 둘레 반지름 RING_R 의 원 위에서만 움직일 수 있다.
# 로봇이 그 원에서 벗어나면 벗어난 거리 x 강성만큼 힘이 생긴다 (순응 제어면 로봇이 물러서 작게, 아니면 크게).
# 놓았을 때 수직(90도)을 넘었으면 받침(COVER_REST_DEG)으로 넘어가 걸치고, 거의 닫혔으면 닫히고, 그 사이면 쾅 닫힌다.
_AXIS = [1.0, 0.0, 0.0]
_RING0 = [a - b for a, b in zip(_COVER['grip'][:3], HINGE)]               # 닫힌 상태의 고리 (경첩 기준)
RING_AX = sum(a * b for a, b in zip(_RING0, _AXIS))
RING_R = _COVER['radius']
HINGE_K_STIFF, HINGE_K_SOFT = 5.0, 1.0      # [N/mm] 로봇이 딱딱할 때 / 순응 제어일 때
HINGE_BREAK_N = 40.0                         # 경첩/포맥스가 버티는 힘 (가짜 기준)
# 고리(링)는 손가락에 걸려 있을 뿐 꽉 물린 게 아니라서, 고리 안쪽 크기만큼 로봇과 고리 사이에 놀이가 있다.
# 로봇이 원에서 이 거리 안쪽으로 벗어나는 동안은 뚜껑에 힘이 안 걸린다. --ring-slack 으로 바꾼다.
# (가짜 세계의 가정일 뿐이다. 실제 보호는 외력 감시가 한다)
RING_SLACK = 25.0                            # [mm] (0 = 고리를 꽉 물고 있음)
# 침대 칸, 보관대 자리는 bed_config 의 좌표(BED_XY, RACK_XY)를 그대로 쓴다
for _n, (_x, _y) in zip(cfg.P_BED, cfg.BED_XY):
    POSES[_n] = [_x, _y, SAFE_Z] + DOWN
for _n, (_x, _y) in zip(cfg.P_RACK, cfg.RACK_XY):
    POSES[_n] = [_x, _y, SAFE_Z] + DOWN

K_REF = 2.0                     # 기준 높이 정상 블록의 강성 [N/mm]
# 실리콘 종류 (--bed-kind / --rack-kind): 강성 배율. 실제 40A:10A 가 약 4배라 같은 비율로 둔다.
KIND = {'H': 2.0, 'S': 0.5}     # H = 단단함(40A), S = 무름(10A). 지정하지 않으면 1.0
RACK_K_FACTOR = 1.25            # 보관대(트레이)에서 누르면 천이 없어 조금 더 단단하게 나온다고 가정
TOE = 1.0                       # 천 때문에 처음 1mm 는 힘이 덜 나온다
TOOL_WEIGHT_FZ = -7.3           # 툴 무게를 설정하지 않았을 때처럼 힘에 항상 더해지는 값
PROTECT_N = 55.0                # 이 이상 힘이 걸리면 보호 정지
TCP_NAME = 'sim_tcp'
SIM_TCP_POSJ0 = [400.0, 0.0, 200.0, 0.0, 0.0, 0.0]
SCALE = [4.0, 4.0, 4.0, 2.0, 2.0, 1.0]


def ikin(pose):
    """가짜 역기구학: 좌표 → 관절각 (단순 비례식. J6 = 툴 Z축 회전)."""
    return [(p - o) / s for p, o, s in zip(pose, SIM_TCP_POSJ0, SCALE)]


def fkin(joints):
    return [j * s + o for j, o, s in zip(joints, SIM_TCP_POSJ0, SCALE)]


def write_teach_file(path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    pts = [{'name': n, 'posj': ikin(p), 'posx': p, 'tcp': TCP_NAME, 'time': time.strftime('%H:%M:%S')}
           for n, p in POSES.items()]
    with open(path, 'w') as f:
        json.dump(pts, f, indent=2, ensure_ascii=False)


class Block:
    def __init__(self, h, bad=1.0, kind=1.0):
        self.h, self.bad, self.kind = h, bad, kind

    @property
    def k(self):
        return K_REF * cfg.H_REF / self.h * self.bad * self.kind


class BedSim(Node):
    def __init__(self, opt):
        super().__init__('bed_sim')
        from dsr_msgs2 import srv as ds
        from onrobot_rg_msgs.srv import SetCommand
        self.opt = opt
        self.rnd = random.Random(opt.seed)
        self.lock = threading.RLock()
        self.cb = ReentrantCallbackGroup()

        # 로봇 상태
        self.pose = list(POSES[cfg.P_HOME])
        self.width = 110.0
        self.do = {}                # 마지막으로 쓴 DO 값 {번호: 0/1}
        self.mode = 0
        self.robot_state = 1
        self.moving = False
        self.stop_flag = False
        # 침대 상태
        faults = {int(x) - 1 for x in opt.fault.split(',') if x.strip()}
        self.bed = [None if h is None else Block(h, opt.fault_factor if i in faults else 1.0, opt.bed_kind[i])
                    for i, h in enumerate(opt.bed)]
        self.rack = [None if h is None else Block(h, 1.0, opt.rack_kind[j]) for j, h in enumerate(opt.rack)]
        self.cover_open = False
        self.held = None            # 그리퍼가 잡고 있는 것: None / 'cover' / Block
        self.compliance = False     # task_compliance_ctrl 상태
        self.async_busy = False     # amovec 가 도는 중
        self.max_cover_force = 0.0
        self.no_cover = not cfg.COVER_ENABLED      # 커버가 없으면 블록을 언제든 집고 놓을 수 있다
        self.discarded = []
        self.violations = []
        self.max_force = 0.0

        pre = f'/{cfg.ROBOT_ID}/dsr_controller2/'
        mk = lambda t, n, f: self.create_service(t, pre + n, f, callback_group=self.cb)   # noqa: E731
        mk(ds.GetRobotState, 'system/get_robot_state', self.s_state)
        mk(ds.GetRobotMode, 'system/get_robot_mode', self.s_get_mode)
        mk(ds.SetRobotMode, 'system/set_robot_mode', self.s_set_mode)
        mk(ds.GetLastAlarm, 'system/get_last_alarm', lambda q, r: self._ok(r))
        mk(ds.MoveJoint, 'motion/move_joint', self.s_movej)
        mk(ds.MoveLine, 'motion/move_line', self.s_movel)
        mk(ds.MoveCircle, 'motion/move_circle', self.s_movec)
        mk(ds.MoveStop, 'motion/move_stop', self.s_stop)
        mk(ds.CheckMotion, 'motion/check_motion', self.s_check_motion)
        mk(ds.TaskComplianceCtrl, 'force/task_compliance_ctrl', self.s_compliance_on)
        mk(ds.ReleaseComplianceCtrl, 'force/release_compliance_ctrl', self.s_compliance_off)
        mk(ds.GetCurrentPosj, 'aux_control/get_current_posj', self.s_posj)
        mk(ds.GetCurrentPosx, 'aux_control/get_current_posx', self.s_posx)
        mk(ds.GetToolForce, 'aux_control/get_tool_force', self.s_force)
        mk(ds.GetCurrentTcp, 'tcp/get_current_tcp', self.s_tcp)
        mk(ds.GetCurrentTool, 'tool/get_current_tool', lambda q, r: self._ok(r))
        mk(ds.SetCtrlBoxDigitalOutput, 'io/set_ctrl_box_digital_output', self.s_do)
        mk(ds.GetCtrlBoxDigitalInput, 'io/get_ctrl_box_digital_input', self.s_di)
        self.create_service(SetCommand, '/onrobot/sendCommand', self.s_gripper, callback_group=self.cb)
        self.pub = self.create_publisher(String, '/bed_sim/world', 10)
        self.create_subscription(String, '/bed_sim/human', self.human, 10, callback_group=self.cb)
        self.create_timer(0.2, self.publish_world, callback_group=self.cb)
        self.get_logger().info(f'가짜 로봇 준비 완료. 불량 칸: {sorted(i + 1 for i in faults) or "없음"}, '
                               f'속도 {opt.speed:g}배')

    @staticmethod
    def _ok(res):
        res.success = True
        return res

    # ── 세계 상태 ─────────────────────────────────────────────────────
    def violate(self, text):
        with self.lock:
            if text not in self.violations:
                self.violations.append(text)
                self.get_logger().error('위반: ' + text)

    def near(self, name, tol=6.0):
        p = POSES[name] if name in POSES else COVER_POSES[name]
        return math.dist(self.pose[:3], p[:3]) <= tol

    def over(self, kind):
        """TCP 아래에 있는 칸 번호(0부터). 없으면 None."""
        names = cfg.P_BED if kind == 'bed' else cfg.P_RACK
        for i, n in enumerate(names):
            if math.dist(self.pose[:2], POSES[n][:2]) <= 10.0:
                return i
        return None

    def publish_world(self):
        with self.lock:
            held = self.held.h if isinstance(self.held, Block) else self.held
            w = {'bed': [b.h if b else None for b in self.bed],
                 'bed_bad': [bool(b and b.bad != 1.0) for b in self.bed],
                 'rack': [b.h if b else None for b in self.rack],
                 'cover_open': bool(self.cover_open),
                 'held': held, 'discarded': [b.h for b in self.discarded],
                 'discarded_bad': [b.bad != 1.0 for b in self.discarded],
                 'bed_k': [round(b.k, 3) if b else None for b in self.bed],
                 'rack_k': [round(b.k, 3) if b else None for b in self.rack],
                 'violations': list(self.violations), 'pose': [round(v, 2) for v in self.pose],
                 'width': self.width, 'robot_state': self.robot_state, 'max_force': round(self.max_force, 1),
                 'max_cover_force': round(self.max_cover_force, 1), 'compliance': self.compliance}
        self.pub.publish(String(data=json.dumps(w, ensure_ascii=False)))

    def human(self, msg):
        """사람이 손으로 정리하는 것을 흉내 낸다 (테스트용). 'tidy': 그리퍼의 블록을 빼고 빈 침대 칸을 채운다."""
        if msg.data != 'tidy':
            return
        with self.lock:
            loose = []
            if isinstance(self.held, Block):
                loose.append(self.held)
            self.held, self.width = None, 110.0
            for i, b in enumerate(self.bed):
                if b is None:                       # 빈 칸: 손에 있던 블록, 없으면 보관대의 기준 높이 블록
                    if loose:
                        self.bed[i] = loose.pop()
                    else:
                        j = next(j for j, r in enumerate(self.rack) if r is not None and r.h == cfg.H_REF)
                        self.bed[i], self.rack[j] = self.rack[j], None
            for b in loose:
                self.rack[self.rack.index(None)] = b
        self.get_logger().info('사람이 블록을 정리했습니다.')

    # ── 힘 ────────────────────────────────────────────────────────────
    def true_force(self, where=None):
        """지금 자세에서 블록이 툴을 밀어 올리는 힘 [N] (노이즈 없음). where 에 ('bed'|'rack') 을 받을 리스트를 주면 채운다."""
        if self.width > 45.0 or isinstance(self.held, Block):
            return 0.0
        i, j = self.over('bed'), self.over('rack')
        if i is not None and self.bed[i] is not None:
            b, factor, kind = self.bed[i], 1.0, 'bed'
        elif j is not None and self.rack[j] is not None:
            b, factor, kind = self.rack[j], RACK_K_FACTOR, 'rack'
        else:
            return 0.0
        pen = FLOOR_Z + b.h - self.pose[2]
        if pen <= 0:
            return 0.0
        if where is not None:
            where.append(kind)
        k = b.k * factor
        return k * pen * pen / (2 * TOE) if pen < TOE else k * (pen - TOE / 2)

    def cover_strain(self):
        """커버를 잡고 있을 때 고리가 갈 수 있는 원에서 TCP 가 벗어난 벡터 [mm] 와 뚜껑 각도 [deg]."""
        rel = [a - b for a, b in zip(self.pose[:3], HINGE)]
        ax = sum(a * b for a, b in zip(rel, _AXIS))
        rad = [r - ax * a for r, a in zip(rel, _AXIS)]
        n = math.sqrt(sum(v * v for v in rad)) or 1e-9
        q = [h + RING_AX * a + v / n * RING_R for h, a, v in zip(HINGE, _AXIS, rad)]
        d = [p - qq for p, qq in zip(self.pose[:3], q)]
        slack = getattr(self.opt, 'ring_slack', RING_SLACK)
        dn = math.sqrt(sum(v * v for v in d))
        if slack > 0 and dn > 0:
            d = [v * max(0.0, dn - slack) / dn for v in d]
        r0 = [r - RING_AX * a for r, a in zip(_RING0, _AXIS)]
        cr = [r0[1] * rad[2] - r0[2] * rad[1], r0[2] * rad[0] - r0[0] * rad[2], r0[0] * rad[1] - r0[1] * rad[0]]
        ang = math.degrees(math.atan2(sum(c * a for c, a in zip(cr, _AXIS)), sum(a * b for a, b in zip(r0, rad))))
        return d, ang

    def cover_force(self):
        """뚜껑이 로봇을 미는 힘 벡터 [N]."""
        if self.held != 'cover':
            return [0.0, 0.0, 0.0]
        k = HINGE_K_SOFT if self.compliance else HINGE_K_STIFF
        return [-k * v for v in self.cover_strain()[0]]

    def check_physics(self):
        """움직이는 도중 계속 부른다. 보호 정지가 걸리면 False."""
        fc = math.sqrt(sum(v * v for v in self.cover_force()))
        self.max_cover_force = max(self.max_cover_force, fc)
        if fc >= HINGE_BREAK_N:
            self.violate(f'뚜껑을 억지로 끌어 경첩에 {fc:.0f}N 이 걸렸습니다 (경첩 파손)')
        f = self.true_force()
        self.max_force = max(self.max_force, f)
        if f >= PROTECT_N:
            self.violate(f'누르는 힘이 {PROTECT_N:.0f}N 을 넘어 보호 정지')
            self.robot_state = 5
            return False
        i = self.over('bed')
        z = self.pose[2]
        if i is not None:
            top = FLOOR_Z + (self.bed[i].h if self.bed[i] else 0.0)
            if not (self.cover_open or self.no_cover) and self.width > 45.0 and z < top - 2.0:
                self.violate('커버가 닫혀 있는데 그리퍼를 벌린 채 천 아래로 내려갔습니다')
            if z < FLOOR_Z - 1.0:
                self.violate('침대 바닥에 충돌')
        j = self.over('rack')
        if j is not None and z < FLOOR_Z - 1.0:
            self.violate('보관대 바닥에 충돌')
        return True

    # ── 이동 ──────────────────────────────────────────────────────────
    def run_motion(self, waypoints, duration):
        """자세를 waypoints 를 따라 duration 초 동안 옮긴다. 정지/보호 정지면 그 자리에서 멈춘다."""
        if self.robot_state != 1:
            return False
        with self.lock:
            if self.moving:
                return False
            self.moving, self.stop_flag = True, False
        try:
            start = list(self.pose)
            pts = [start] + [list(w) for w in waypoints]
            dur = max(duration / self.opt.speed, 0.002)
            steps = max(int(dur / 0.005), 1)
            for n in range(1, steps + 1):
                if self.stop_flag:
                    return True                  # 실제 로봇도 정지하면 이동 서비스가 끝난다
                u = n / steps * (len(pts) - 1)
                seg = min(int(u), len(pts) - 2)
                t = u - seg
                with self.lock:
                    self.pose = [a + (b - a) * t for a, b in zip(pts[seg], pts[seg + 1])]
                    if not self.check_physics():
                        return False
                time.sleep(dur / steps)
            return True
        finally:
            self.moving = False

    def s_movej(self, req, res):
        target = [float(v) for v in req.pos]
        cur = ikin(self.pose)
        if req.mode == 1:
            target = [c + d for c, d in zip(cur, target)]
        if self.held == 'cover':
            self.violate('커버를 잡은 채 관절 이동을 했습니다')
        dist = max(abs(a - b) for a, b in zip(cur, target))
        res.success = self.run_motion([fkin(target)], dist / max(req.vel, 1e-3))
        return res

    def s_movel(self, req, res):
        target = [float(v) for v in req.pos]
        if req.ref != 0:
            self.violate('가짜 로봇은 베이스 기준 movel 만 지원합니다')
            res.success = False
            return res
        if req.mode == 1:
            target = [c + d for c, d in zip(self.pose, target)]
        # 커버를 잡은 채 직선 이동하면 경첩 모델(cover_strain)이 힘과 파손을 계산한다
        dur = math.dist(self.pose[:3], target[:3]) / max(req.vel[0], 1e-3)
        if getattr(req, 'sync_type', 0) == 1:          # amovel
            return self._start_async(res, lambda: self.run_motion([target], dur))
        res.success = self.run_motion([target], dur)
        return res

    def _start_async(self, res, fn):
        """비동기 이동: 출발만 시키고 바로 돌아간다 (끝났는지는 check_motion 으로 본다)."""
        if self.robot_state != 1 or self.moving or self.async_busy:
            res.success = False
            return res
        self.async_busy = True

        def run():
            try:
                fn()
            finally:
                self.async_busy = False
        threading.Thread(target=run, daemon=True).start()
        res.success = True
        return res

    def s_movec(self, req, res):
        via, end = [float(v) for v in req.pos[0].data], [float(v) for v in req.pos[1].data]
        vel = max(req.vel[0], 1e-3)
        if getattr(req, 'sync_type', 0) == 1:          # amovec
            return self._start_async(res, lambda: self._movec(via, end, vel))
        res.success = self._movec(via, end, vel)
        return res

    def _movec(self, via, end, vel):
        start = list(self.pose)
        holding = self.held == 'cover'
        pts, length = self._arc_points(start, via, end)
        if holding:
            # 뚜껑은 경첩 둘레로만 돈다: 원호가 고리 원에서 15mm 넘게 벗어나면 뚜껑/경첩에 무리가 간다
            # (교시 방식의 두 번째 원호처럼 일부러 몇 mm 안쪽으로 지나는 것은 허용)
            dev = max(abs(math.hypot(q[1] - HINGE[1], q[2] - HINGE[2]) - RING_R) for q in [start] + list(pts))
            if dev > 15.0:
                self.violate(f'커버 원호가 경첩 원에서 {dev:.1f}mm 벗어났습니다')
        ok = self.run_motion(pts, length / vel)
        return ok

    @staticmethod
    def _arc_points(start, via, end):
        """세 점을 지나는 원호를 1도 간격 점으로 (자세는 시작 → 끝 직선 보간). 원이 안 되면 직선."""
        try:
            c, r, ax = bed_math.circle_3pt(start[:3], via[:3], end[:3])
        except ValueError:
            return [via, end], math.dist(start[:3], via[:3]) + math.dist(via[:3], end[:3])
        u = [a - b for a, b in zip(start[:3], c)]
        w = [a - b for a, b in zip(end[:3], c)]
        cr = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]]
        ang = math.degrees(math.atan2(float(ax @ cr), sum(a * b for a, b in zip(u, w)))) % 360.0
        n = max(int(ang), 2)
        pts = []
        for i in range(1, n + 1):
            t = i / n
            xyz = bed_math.rotate_about(start[:3], c, ax, ang * t)
            pts.append([float(v) for v in xyz] + [a + (b - a) * t for a, b in zip(start[3:], end[3:])])
        return pts, math.radians(ang) * r

    def s_check_motion(self, req, res):
        res.status = 2 if (self.moving or self.async_busy) else 0
        return self._ok(res)

    def s_compliance_on(self, req, res):
        self.compliance = True
        return self._ok(res)

    def s_compliance_off(self, req, res):
        self.compliance = False
        return self._ok(res)

    def s_stop(self, req, res):
        self.stop_flag = True
        return self._ok(res)

    # ── 읽기 ──────────────────────────────────────────────────────────
    def s_state(self, req, res):
        res.robot_state = 2 if self.moving else self.robot_state
        return self._ok(res)

    def s_get_mode(self, req, res):
        res.robot_mode = self.mode
        return self._ok(res)

    def s_set_mode(self, req, res):
        self.mode = req.robot_mode
        return self._ok(res)

    def s_posj(self, req, res):
        res.pos = ikin(self.pose)
        return self._ok(res)

    def s_posx(self, req, res):
        res.task_pos_info = [Float64MultiArray(data=list(self.pose) + [2.0])]
        return self._ok(res)

    def s_force(self, req, res):
        n = self.opt.noise
        with self.lock:
            fz = self.opt.sign * self.true_force() + TOOL_WEIGHT_FZ
            fc = self.cover_force()
        res.tool_force = [fc[0] + self.rnd.gauss(0, n), fc[1] + self.rnd.gauss(0, n),
                          fz + fc[2] + self.rnd.gauss(0, n), 0.0, 0.0, 0.0]
        return self._ok(res)

    def s_tcp(self, req, res):
        res.info = TCP_NAME
        return self._ok(res)

    # ── 그리퍼 ────────────────────────────────────────────────────────
    def s_do(self, req, res):
        # WebLogic 처럼 DO1·DO2 조합으로 동작한다: (1,0) 닫기, (0,1) 열기, (1,1) 너비로 잡기, (0,0) 그대로
        self.do[req.index] = req.value
        state = (self.do.get(cfg.DO_CLOSE, 0), self.do.get(cfg.DO_OPEN, 0))
        if state == (1, 0):
            self.set_width(0.0)
        elif state == (0, 1):
            self.set_width(110.0)
        elif state == (1, 1):
            self.set_width(cfg.DO_BOTH_WIDTH, grip=True)
        return self._ok(res)

    def s_di(self, req, res):
        res.value = 1 if self.held is not None else 0
        return self._ok(res)

    def s_gripper(self, req, res):
        c = req.command
        try:
            w = 0.0 if c == 'c' else 110.0 if c == 'o' else int(c) / 10.0
        except ValueError:
            res.success, res.message = False, f'unknown command {c!r}'
            return res
        time.sleep(0.6 / self.opt.speed)
        self.set_width(w)
        res.success = True
        return res

    def set_width(self, w, grip=False):
        """grip=True: 너비로 잡기 (62 mm 처럼 45 mm 보다 넓어도 열린 상태에서 오면 잡는다)."""
        with self.lock:
            old, self.width = self.width, w
            closing = (w < 45.0 <= old) or (grip and old >= 45.0 and old > w)
            opening = w >= 45.0 and w > old and not grip     # 68 mm 로 잡은 상태에서 110 으로 열어도 놓는다
            if closing:
                self._close()
            elif opening:
                self._open()

    def _slot_under(self):
        i = self.over('bed')
        if i is not None:
            return 'bed', i, self.bed
        j = self.over('rack')
        if j is not None:
            return 'rack', j, self.rack
        return None, None, None

    def _close(self):
        handle = COVER_REST if self.cover_open else COVER_CLOSED
        if self.near(handle):
            self.held = 'cover'
            return
        if self.near(COVER_REST) or self.near(COVER_CLOSED):
            self.violate('커버 손잡이가 없는 곳을 잡았습니다 (커버 열림/닫힘 상태가 기록과 다름)')
            return
        kind, i, arr = self._slot_under()
        if kind is None or self.pose[2] > SAFE_Z - 20.0:
            return                                   # 허공에서 닫음 (누르기 준비 등)
        if kind == 'bed' and not (self.cover_open or self.no_cover):
            self.violate('커버가 닫혀 있는데 블록을 집으려 했습니다')
            return
        b = arr[i]
        grip_z = FLOOR_Z + (b.h if b else 0.0) - GRIP_DEPTH
        if b is None or abs(self.pose[2] - grip_z) > 6.0:
            self.violate(f'{kind} {i + 1}번에서 블록을 못 잡았습니다 (블록 없음 또는 잡는 높이가 틀림: '
                         f'z={self.pose[2]:.1f}, 맞는 높이 {grip_z:.1f})')
            return
        self.held, arr[i] = b, None

    def _open(self):
        held, self.held = self.held, None
        if held == 'cover':
            ang = self.cover_strain()[1]
            if ang >= 90.0:                         # 수직을 넘었다 → 받침에 걸친다
                if not self.cover_open:
                    if any(b is None for b in self.bed):
                        self.violate('빈 칸이 있는데 커버 상태가 바뀜')
                if ang > _COVER['rest_deg'] + 1.0:
                    self.violate(f'뚜껑을 받침({_COVER["rest_deg"]:.0f}도)보다 더 젖혔습니다 ({ang:.0f}도)')
                self.cover_open = True
            elif ang <= 5.0:                        # 거의 닫혔다 → 제 무게로 닫힌다
                if self.cover_open and any(b is None for b in self.bed):
                    self.violate('빈 칸이 있는 채로 커버를 닫았습니다')
                self.cover_open = False
            else:
                self.violate(f'뚜껑을 {ang:.0f}도 열린 채로 놓아서 쾅 닫혔습니다')
                self.cover_open = False
            return
        if not isinstance(held, Block):
            return
        if math.dist(self.pose[:2], DISCARD_XY) <= 60.0:
            self.discarded.append(held)
            return
        kind, i, arr = self._slot_under()
        seat_z = FLOOR_Z + held.h - GRIP_DEPTH           # 블록이 바닥에 닿을 때의 손끝 높이
        if kind is None:
            self.violate('블록을 엉뚱한 곳에 떨어뜨렸습니다')
        elif kind == 'bed' and not (self.cover_open or self.no_cover):
            self.violate('커버가 닫혀 있는데 블록을 놓았습니다')
        elif arr[i] is not None:
            self.violate(f'{kind} {i + 1}번에 이미 블록이 있는데 또 놓았습니다')
        elif not (-1.0 <= self.pose[2] - seat_z <= 8.0):
            self.violate(f'{kind} {i + 1}번에 놓는 높이가 틀립니다 (z={self.pose[2]:.1f}, 맞는 높이 {seat_z:.1f})')
        else:
            arr[i] = held


def parse_list(text, n):
    out = [None if x.strip() in ('', '-') else float(x) for x in text.split(',')]
    if len(out) != n:
        raise SystemExit(f'개수가 {n}개여야 합니다: {text}')
    return out


def parse_kind(text, n):
    if not text:
        return [1.0] * n
    out = [KIND.get(x.strip().upper(), 1.0) for x in text.split(',')]
    if len(out) != n:
        raise SystemExit(f'개수가 {n}개여야 합니다: {text}')
    return out


def main(args=None):
    ap = argparse.ArgumentParser(description='가짜 로봇 + 가짜 침대')
    ap.add_argument('--fault', default='', help='불량 블록 칸 번호 (예: 2,5)')
    ap.add_argument('--fault-factor', type=float, default=0.5, help='불량 블록의 강성 비율 (기본 0.5)')
    ap.add_argument('--speed', type=float, default=5.0, help='동작 배속')
    ap.add_argument('--noise', type=float, default=0.25, help='힘 노이즈 표준편차 [N]')
    ap.add_argument('--sign', type=float, default=1.0, help='누를 때 Fz 부호 (+1 또는 -1)')
    ap.add_argument('--seed', type=int, default=None)
    ap.add_argument('--ring-slack', type=float, default=RING_SLACK,
                    help=f'커버 고리가 손가락을 따라 미끄러질 수 있는 거리 [mm] (기본 {RING_SLACK:g}, 0=꽉 물음)')
    ap.add_argument('--bed', default=None, help='침대 블록 높이 6개 (예: 30,30,30,30,30,30)')
    ap.add_argument('--rack', default=None, help='보관대 블록 높이 (빈 자리는 -)')
    ap.add_argument('--bed-kind', default=None, help='침대 블록 종류 H(단단)/S(무름)/- (예: H,H,-,-,H,H)')
    ap.add_argument('--rack-kind', default=None, help='보관대 블록 종류 H/S/- (예: S,H,S,H,S,-)')
    opt = ap.parse_args(remove_ros_args(sys.argv)[1:])
    opt.bed = parse_list(opt.bed, len(cfg.P_BED)) if opt.bed else list(cfg.BED_INIT)
    opt.rack = parse_list(opt.rack, len(cfg.P_RACK)) if opt.rack else list(cfg.RACK_INIT)
    opt.bed_kind = parse_kind(opt.bed_kind, len(cfg.P_BED))
    opt.rack_kind = parse_kind(opt.rack_kind, len(cfg.P_RACK))

    # 가짜 교시 파일과 가짜용 재고 기록을 처음 상태로
    from bed_inspect import bed_store
    write_teach_file(cfg.SIM_TEACH_FILE)
    cfg.DATA_DIR = cfg.SIM_DATA_DIR
    bed_store.save_inventory({'bed': opt.bed, 'rack': opt.rack})
    bed_store._save_json('world.json', {'cover_open': False, 'uncertain': False})
    bed_store.save_calib({})

    rclpy.init(args=args)
    node = BedSim(opt)
    node.get_logger().info(f'가짜 교시 파일: {cfg.SIM_TEACH_FILE}')
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
