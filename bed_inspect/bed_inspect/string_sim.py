# 나사 실 고정/해제(v1) 물리 시뮬레이션 — 실제 string_v1_demo 코드(fix/unfix)를 가짜 로봇에 그대로 연결해서 돌린다.
#
#   python3 -m bed_inspect.string_sim [--out 폴더] [--e0 16.9] [--hole-err 0.8 -0.6] [--string-len 510]
#
# 모델 (단위 mm, N, deg). 에뮬레이터에는 힘·실·나사산이 없어서 직접 만든다:
#   - 실: 실 고정 나사 A 에 한쪽 끝, 다른 끝은 실 끝 나사(E). 감는 나사 P(반지름 rp)에 감기면
#     필요한 길이 = |A-P| + rp*감긴각 + |P-E|, 아니면 |A-E|. 실 길이 L 보다 길어야 하면 늘어난 만큼 장력 T = k*(필요-L).
#     장력은 E 에서 실이 나가는 쪽(P 또는 A)으로 로봇을 당긴다 → 가짜 힘센서에 들어감 (toward_tension 이 실제로 멈춤).
#     감긴각: 실(A→E 직선)이 P 에 걸리기 시작한 뒤 E 가 P 둘레로 돈 각도.
#   - 나사산: 구멍에 물린 깊이 e. 잡고 툴을 돌리면 e += 피치*각/360 (조이기 +). 나사 머리 높이 = 71.08 - (e - e0).
#     잡고 있는 높이와 머리 높이 차이 δ → z 강성(순응 제어면 부드럽게)으로 힘. 그 힘이 잡는 힘의 마찰 한계를 넘으면 미끄러짐.
#     물린 상태(e>0)에서 위로 당기면 나사산이 버팀 → 미끄러짐 (실물 첫 시험 문제).
#   - 구멍: 실제 구멍 위치에 오차(hole_err)가 있고 모따기 반경 chamfer. 순응 제어(x·y 부드럽게)+누르는 힘이면
#     오차 ≤ 모따기일 때 구멍 가운데로 미끄러져 들어감. 아니면 나사 끝이 판 위에 얹혀 못 들어감.
#   - 블록: 감는 나사와 구멍 사이 가운데 40x40, 높이 BLOCK_H. 나사 끝이 블록 안을 지나면 충돌로 기록.

import argparse
import math
import os
import sys

from bed_inspect import bed_config as cfg
from bed_inspect import string_v1_config as sc
from bed_inspect import string_v1_plan as plan


class SimError(Exception):
    pass


class World:
    def __init__(self, e0, hole_err, L, k_string=0.5, rp=4.0, chamfer=1.5, mu_grip=0.35, grip_n=40.0):
        self.A = sc.ANCHOR_XY
        self.P = sc.POST_XY
        self.hole_true = (sc.HOLE_XY[0] + hole_err[0], sc.HOLE_XY[1] + hole_err[1])
        self.rp, self.chamfer = rp, chamfer
        self.L, self.k = L, k_string
        self.slip_limit = mu_grip * grip_n * 2          # 두 손가락 마찰 [N]
        self.e0 = e0
        self.tcp = [300.0, 0.0, 400.0]
        self.yaw = 0.0                                   # 툴 Z 둘레 누적 회전 (J6 변화) [deg]
        self.j6_0 = -196.4                              # 실물 구멍 위 자세의 J6 (11-21 로그)
        self.comp = None                                 # 순응 강성 [kx, ky, kz] N/mm, None = 단단
        self.push = 0.0                                  # 아래로 누르는 힘 [N]
        self.grip_closed = False
        self.holding = False
        # 나사: 처음엔 구멍에 꽂혀 있음 (e0 만큼 물림)
        self.e = e0
        self.engaged = e0 > 0
        self.screw_xy = list(self.hole_true)
        self.head_z = sc.HOLE_GRIP_Z
        # 실
        self.contact = False
        self.wrap = 0.0                                  # 감긴각 [deg], 위에서 반시계 +
        self._prev_ang = None
        self.T = 0.0
        self.t = 0
        self.phase = ''
        self.events = []
        self.block = False                               # 고정 땐 블록 없음, 해제 땐 있음
        self.trace = []                                  # (t, x, y, z, T, wrap, e, phase)
        self.fz_contact = 0.0

    # ── 기록 ──
    def ev(self, kind, msg):
        self.events.append((self.t, self.phase, kind, msg))

    def tip(self):
        return self.head_z - sc.SCREW_LEN

    def end_xy(self):
        return self.screw_xy

    # ── 실 ──
    def update_string(self):
        E = self.end_xy()
        ang = math.degrees(math.atan2(E[1] - self.P[1], E[0] - self.P[0]))
        if not self.contact:
            # A→E 직선이 감는 나사에 걸리는지 (선분과 원 거리)
            ax, ay = self.A
            dx, dy = E[0] - ax, E[1] - ay
            n2 = dx * dx + dy * dy
            u = max(0.0, min(1.0, ((self.P[0] - ax) * dx + (self.P[1] - ay) * dy) / n2)) if n2 > 0 else 0.0
            d = math.hypot(ax + u * dx - self.P[0], ay + u * dy - self.P[1])
            if d <= self.rp and 0.0 < u < 1.0:
                self.contact = True
                self._prev_ang = ang
                self.ev('실', f'실이 감는 나사에 걸리기 시작 (E {E[0]:.0f},{E[1]:.0f})')
        else:
            dang = (ang - self._prev_ang + 180.0) % 360.0 - 180.0
            self.wrap += dang
            self._prev_ang = ang
            if abs(self.wrap) < 1.0 and abs(dang) > 0 and self._unwinding:
                pass
        if self.contact:
            # 감긴 쪽 반대로 다 돌아 풀렸고 직선이 더 이상 안 걸리면 접촉 끝
            if (self.wrap <= 0.0 and self._wrap_peak > 0) or (self.wrap >= 0.0 and self._wrap_peak < 0):
                self.contact, self.wrap = False, 0.0
                self.ev('실', '감긴 실이 다 풀림')
            self._wrap_peak = self.wrap if abs(self.wrap) > abs(self._wrap_peak) else self._wrap_peak
            need = math.hypot(self.A[0] - self.P[0], self.A[1] - self.P[1]) + self.rp * math.radians(abs(self.wrap)) \
                + math.hypot(E[0] - self.P[0], E[1] - self.P[1])
            pull_from = self.P
        else:
            need = math.hypot(E[0] - self.A[0], E[1] - self.A[1])
            pull_from = self.A
        self.T = max(0.0, self.k * (need - self.L))
        dx, dy = pull_from[0] - E[0], pull_from[1] - E[1]
        n = math.hypot(dx, dy) or 1.0
        self.F_string = (self.T * dx / n, self.T * dy / n)
        if self.T > 30.0:
            self.ev('위험', f'실 장력 {self.T:.0f}N (끊어지거나 나사가 휠 수 있음)')

    _unwinding = False
    _wrap_peak = 0.0

    # ── 한 걸음 ──
    def step(self, d, dyaw):
        """TCP 를 d=[dx,dy,dz] (베이스), 툴 회전 dyaw(툴 Z 둘레, + = 툴 +Z(아래) 오른손 = 위에서 시계) 만큼."""
        self.t += 1
        self.tcp = [a + b for a, b in zip(self.tcp, d)]
        self.yaw += dyaw
        j6 = self.j6_0 + self.yaw
        if abs(j6) > 360.0 and not getattr(self, '_j6_warned', False):
            self._j6_warned = True
            self.ev('문제', f'J6 {j6:.0f} 가 ±360 범위를 넘음')
        self.fz_contact = 0.0
        self.mz = 0.0
        if self.holding and self.push > 0 and self.comp:
            # 힘 제어(아래로 누르기): 닿을 때까지 내려가고, 닿은 뒤엔 나사 머리를 따라감
            if self.engaged:
                self.tcp[2] = self.head_z - sc.SCREW_PITCH * dyaw / 360.0 * 0  # 아래 _step_engaged 뒤에 맞춤
            elif getattr(self, 'in_bore', False):
                start = sc.HOLE_GRIP_Z + self.e0             # 구멍 안 헐거운 부분: 누르면 나사산 시작까지 내려감
                if self.tcp[2] > start:
                    self.tcp[2] -= min(0.5, self.tcp[2] - start)
            elif self.tip() > sc.TABLE_Z:
                self.tcp[2] -= min(0.5, self.tip() - sc.TABLE_Z)
        if self.holding:
            if self.engaged:
                self._step_engaged(dyaw)
                if self.holding and self.push > 0 and self.comp:
                    self.tcp[2] = self.head_z
            else:
                self._step_free()
        self.update_string()
        self.trace.append((self.t, self.tcp[0], self.tcp[1], self.tcp[2], self.T, self.wrap, self.e if self.engaged else -1,
                           self.phase))

    def _step_engaged(self, dyaw):
        # 돌린 만큼 나사가 들어가거나(+) 나옴(-). 툴 + 회전 = 위에서 시계 = 오른나사 조임 (TIGHTEN_SIGN 1 가정)
        de = sc.SCREW_PITCH * dyaw * sc.TIGHTEN_SIGN / 360.0
        self.e += de
        if abs(dyaw) > 0:
            self.mz = 0.5 * (1 if dyaw > 0 else -1)       # 나사산 마찰 반발 토크 [Nm]
        self.head_z = sc.HOLE_GRIP_Z - (self.e - self.e0)
        if self.e <= 0.1 and de < 0:
            self.engaged = False
            self.e = 0.0
            self.in_bore = True
            self.ev('나사', '나사산이 다 풀려 구멍에서 빠짐')
            return
        # 잡은 높이 vs 머리 높이
        dz = self.tcp[2] - self.head_z
        kz = self.comp[2] if self.comp else 20.0
        f = kz * dz - (self.push if self.comp else 0.0)
        self.fz_contact = f
        dxy = math.hypot(self.tcp[0] - self.hole_true[0], self.tcp[1] - self.hole_true[1])
        kxy = self.comp[0] if self.comp else 20.0
        fxy = kxy * dxy
        if abs(f) > self.slip_limit or fxy > self.slip_limit:
            self.holding = False
            self.ev('문제', f'나사가 손가락에서 미끄러짐 (z 힘 {f:.0f}N, 옆 힘 {fxy:.0f}N, 한계 {self.slip_limit:.0f}N). '
                          f'머리 z {self.head_z:.1f}, 잡은 높이 {self.tcp[2]:.1f}, 물린 깊이 {self.e:.1f}mm')
            return
        if self.comp is None and abs(f) > 10.0:
            self.ev('주의', f'순응 제어 없이 나사산과 {f:.0f}N 로 버팀')

    def _step_free(self):
        if getattr(self, 'in_bore', False):
            # 나사가 아직 구멍(헐거운 윗부분) 안: x·y 는 구멍에 묶이고, 아래로는 나사산 시작에서 멈춤
            start = sc.HOLE_GRIP_Z + self.e0                 # 나사산 시작에 닿는 머리 높이
            self.screw_xy = list(self.hole_true)
            if self.tcp[2] - sc.SCREW_LEN >= sc.TABLE_Z:
                self.in_bore = False
            elif self.tcp[2] <= start:
                # 나사산 시작에 닿음 → 물림 시작 (누르며 돌리면 들어감)
                self.engaged = True
                self.e = 0.01
                self.head_z = sc.HOLE_GRIP_Z + (self.e0 - self.e)
                self.tcp[0], self.tcp[1] = self.hole_true
                return
            else:
                self.head_z = self.tcp[2]
                self.tcp[0], self.tcp[1] = self.hole_true
                return
        self.screw_xy = [self.tcp[0], self.tcp[1]]
        self.head_z = self.tcp[2]
        tip = self.tip()
        # 블록
        bx, by = (self.P[0] + sc.HOLE_XY[0]) / 2, (self.P[1] + sc.HOLE_XY[1]) / 2
        if self.block and abs(self.tcp[0] - bx) < 20 and abs(self.tcp[1] - by) < 20 and tip < sc.TABLE_Z + sc.BLOCK_H:
            self.ev('충돌', f'나사 끝이 블록에 닿음 (끝 z {tip:.1f} < 블록 윗면 {sc.TABLE_Z + sc.BLOCK_H:.0f})')
        if tip <= sc.TABLE_Z:
            m = math.hypot(self.tcp[0] - self.hole_true[0], self.tcp[1] - self.hole_true[1])
            # 모따기(45도) 위에서 누르는 힘이 옆으로 밀어 주는 거리 = 누르는 힘 / x·y 강성 (순응 제어일 때)
            slide = (max(self.push, 3.0) / self.comp[0]) if self.comp else 0.0
            if m <= 0.3 or (m <= self.chamfer and slide >= m):   # 거의 정확하거나, 모따기가 가운데로 밀어 줄 수 있으면 들어감
                # 모따기로 가운데로 밀려 들어감 (순응 x·y 부드러움) → 나사산 시작
                if self.push > 0 or self.comp:
                    # 구멍 윗부분은 나사산 없이 헐거움(머리 z 71.08 인데 360도면 풀림 → 끝 1바퀴만 물림):
                    # 가운데로 맞춰지면 구멍 안으로 들어감 → 나사산 시작에 닿을 때까지 내려갈 수 있음
                    self.in_bore = True
                    self.tcp[0], self.tcp[1] = self.hole_true
                    self.screw_xy = list(self.hole_true)
                    self.ev('나사', f'구멍 찾음: 오차 {m:.2f}mm 를 순응 제어로 맞추고 나사 끝이 구멍에 들어감')
            elif m <= 2.5 and self.comp is None:
                self.ev('문제', f'순응 제어 없이 구멍 오차 {m:.2f}mm → 나사 끝이 구멍 가장자리에 걸림')
                self.fz_contact = 20.0 * (sc.TABLE_Z - tip)
                self.tcp[2] = self.head_z = sc.TABLE_Z + sc.SCREW_LEN
            else:
                self.fz_contact = (self.comp[2] if self.comp else 20.0) * (sc.TABLE_Z - tip)
                # 판이 막음: 나사 끝은 판 아래로 못 감 (순응 제어면 로봇이 따라 멈춤)
                self.tcp[2] = self.head_z = sc.TABLE_Z + sc.SCREW_LEN
                if m > self.chamfer and m < 30:
                    self.ev('문제', f'구멍을 못 찾음: 실제 구멍과 {m:.2f}mm 어긋남 (모따기 {self.chamfer}mm 보다 큼), 나사 끝이 판 위')
                elif m >= 30:
                    self.ev('충돌', f'나사 끝이 탁자에 끌림 (끝 z {tip:.1f})')

    # ── 그리퍼 ──
    def grip(self, close):
        if not close:
            if self.holding:
                self.holding = False
                if not self.engaged:
                    # 공중에서 놓으면 떨어짐 (구멍 위에서 놓는 것만 정상)
                    m = math.hypot(self.screw_xy[0] - self.hole_true[0], self.screw_xy[1] - self.hole_true[1])
                    if m < 2.0 and self.tip() < sc.TABLE_Z + 3:
                        self.ev('나사', '구멍 위에 나사를 내려놓음')
                    else:
                        self.ev('문제', f'나사를 공중에서 놓침 (끝 z {self.tip():.1f})')
            self.grip_closed = False
            return
        self.grip_closed = True
        dxy = math.hypot(self.tcp[0] - self.screw_xy[0], self.tcp[1] - self.screw_xy[1])
        dz = self.tcp[2] - self.head_z
        if dxy <= 1.5 + (self.chamfer if not self.engaged else 1.5) and abs(dz) <= 2.0:
            self.holding = True
        else:
            self.ev('문제', f'나사 머리를 못 잡음 (xy 어긋남 {dxy:.1f}mm, 높이 차 {dz:.1f}mm)')


class FakeDR:
    DR_BASE, DR_TOOL = 0, 1
    DR_MV_MOD_ABS, DR_MV_MOD_REL = 0, 1
    DR_STATE_IDLE, DR_STATE_BUSY = 0, 2
    DR_FC_MOD_REL = 1
    _srv_name_prefix = 'sim/'

    def __init__(self, w):
        self.w = w
        self.pending = None

    # 경로를 작은 걸음으로
    def _line_steps(self, target_xyz, target_yaw, step=0.5, astep=2.0):
        w = self.w
        d = [b - a for a, b in zip(w.tcp, target_xyz)]
        dist = math.sqrt(sum(c * c for c in d))
        dyaw = target_yaw - w.yaw
        n = max(1, int(math.ceil(max(dist / step, abs(dyaw) / astep))))
        for _ in range(n):
            yield [c / n for c in d], dyaw / n

    def _run(self, gen):
        for d, a in gen:
            self.w.step(d, a)
        return 0

    def movel(self, pos, vel=None, acc=None, ref=0, mod=0, **k):
        w = self.w
        p = list(pos)
        if mod == self.DR_MV_MOD_REL and ref == self.DR_TOOL:
            # 툴 Z 가 아래 → 툴 +dz = 베이스 -z
            return self._run(self._line_steps([w.tcp[0], w.tcp[1], w.tcp[2] - p[2]], w.yaw + p[5]))
        if mod == self.DR_MV_MOD_REL:
            return self._run(self._line_steps([w.tcp[i] + p[i] for i in range(3)], w.yaw))
        # 절대: 자세는 STR_ORI (J6 기준각) 로 돌아감 → 누적 회전 0
        return self._run(self._line_steps(p[:3], 0.0))

    def amovel(self, pos, **k):
        self.pending = self._line_steps(list(pos)[:3], 0.0)
        return 0

    def advance(self, n=1):
        if self.pending is None:
            return
        for _ in range(n):
            try:
                d, a = next(self.pending)
                self.w.step(d, a)
            except StopIteration:
                self.pending = None
                return

    def check_motion(self):
        return self.DR_STATE_IDLE if self.pending is None else self.DR_STATE_BUSY

    def movej(self, pj, **k):
        for qj, qx in getattr(sc, 'UNFIX_EXIT', []):
            if all(abs(a - b) < 0.01 for a, b in zip(qj, pj)):
                # 교시한 관절 자세 → 그 posx 로 (자세 회전은 J6 차이만 반영)
                return self._run(self._line_steps(list(qx[:3]), self.w.yaw + (qj[5] - (self.w.j6_0 + self.w.yaw))))
        dj = list(pj)[5] - (self.w.j6_0 + self.w.yaw)
        if abs(list(pj)[5]) > 360.0:
            self.w.ev('문제', f'J6 {list(pj)[5]:.0f} 범위 밖')
        return self._run(self._line_steps(self.w.tcp, self.w.yaw + dj))

    def movec(self, via, end, angle=None, **k):
        w = self.w
        p1, p2, p3 = w.tcp[:2], list(via)[:2], list(end)[:2]
        ax, ay = p1; bx, by = p2; cx, cy = p3
        dd = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / dd
        uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / dd
        r = math.hypot(ax - ux, ay - uy)
        a0 = math.atan2(ay - uy, ax - ux)
        a1 = math.atan2(by - uy, bx - ux)
        sgn = 1.0 if ((a1 - a0 + math.pi) % (2 * math.pi) - math.pi) > 0 else -1.0
        total = math.radians(angle) if angle else abs(((math.atan2(cy - uy, cx - ux) - a0) % (2 * math.pi)))
        z0, z1 = w.tcp[2], list(end)[2]
        n = int(total * r / 0.5) + 1
        prev = list(w.tcp)
        for i in range(1, n + 1):
            a = a0 + sgn * total * i / n
            q = [ux + r * math.cos(a), uy + r * math.sin(a), z0 + (z1 - z0) * i / n]
            w.step([q[0] - prev[0], q[1] - prev[1], q[2] - prev[2]], 0.0)
            prev = q
        return 0

    def movesx(self, pts, **k):
        for q in pts:
            self._run(self._line_steps(list(q)[:3], self.w.yaw))
        return 0

    def get_tool_force(self, ref=None):
        fx, fy = getattr(self.w, 'F_string', (0.0, 0.0))
        return [fx, fy, self.w.fz_contact, 0.0, 0.0, getattr(self.w, 'mz', 0.0)]

    def task_compliance_ctrl(self, stx, **k):
        self.w.comp = [v / 1000.0 for v in stx[:3]]
        return 0

    def release_compliance_ctrl(self):
        self.w.comp = None
        self.w.push = 0.0
        return 0

    def set_desired_force(self, fd, dir, mod=None, **k):
        self.w.push = -fd[2]
        return 0

    def release_force(self, **k):
        self.w.push = 0.0
        return 0


class FakeRobot:
    def __init__(self, w, dr):
        self.w, self.dr = w, dr

    def posx(self, v):
        return list(v)

    def posj(self, v):
        return list(v)

    def require_standby(self, what):
        pass

    def check_stop(self):
        pass

    def sleep(self, s):
        pass

    def fault_text(self):
        return '(시뮬)'

    def _begin(self, what, target=None):
        self.w.phase = what

    def _done(self, ret, what):
        if ret != 0:
            raise SimError(f'{what} 실패')

    def movel_to(self, pose, vel, acc, what):
        self.w.phase = what
        self.dr.movel(pose, mod=0)

    def move_z(self, dz, vel=None, acc=None):
        self.w.phase = f'수직 이동 {dz:+.1f}'
        self.dr.movel([0, 0, dz, 0, 0, 0], ref=0, mod=1)

    def grip(self, key):
        self.w.grip(cfg.GRIP_WIDTH[key] < 12.7)

    def cur_posj(self):
        return [0.0, 0.0, 90.0, 0.0, 90.0, self.w.j6_0 + self.w.yaw]

    def cur_xyz(self):
        self.dr.advance(1)
        return list(self.w.tcp)

    def cur_z(self):
        return self.w.tcp[2]

    def cur_posx(self):
        return list(self.w.tcp) + list(sc.STR_ORI)

    def read_force_vec(self, n):
        self.dr.advance(1)
        fx, fy = getattr(self.w, 'F_string', (0.0, 0.0))
        return [fx, fy, self.w.fz_contact]

    def stop_motion(self):
        self.dr.pending = None

    def halt(self):
        self.dr.pending = None


def run(e0, hole_err, L, out, label):
    from bed_inspect.string_v1_demo import StringDemo
    w = World(e0=e0, hole_err=hole_err, L=L)
    dr = FakeDR(w)
    r = FakeRobot(w, dr)
    logs = []
    demo = StringDemo(r, dr, None, lambda m: logs.append((w.t, m)))
    demo.virtual = True                         # move_spiral 은 같은 나선 길(movesx)로
    res = {}
    for act in ('fix', 'unfix'):
        w.phase = act
        w.block = (act == 'unfix') or sc.FIX_WITH_BLOCK
        n0 = len(w.events)
        t0 = w.t
        try:
            p = plan.fix_points(sc.PICK_XYZ) if act == 'fix' else plan.unfix_points(sc.PLACE_XYZ)
            getattr(demo, act)(p)
            ok = True
        except Exception as e:                   # noqa: BLE001  시뮬에서 멈춘 이유 그대로 보고
            w.ev('멈춤', f'{type(e).__name__}: {e}')
            ok = False
        tr = [x for x in w.trace if x[0] > t0]
        res[act] = {
            'ok': ok,
            'events': w.events[n0:],
            'T_max': max([x[4] for x in tr] or [0]),
            'wrap_max': max([abs(x[5]) for x in tr] or [0]),
            'wrap_end': w.wrap,
            'T_end': w.T,
            'e_end': w.e if w.engaged else 0.0,
            'engaged_end': w.engaged,
            'holding_end': w.holding,
        }
        if not ok:
            break
    plot(w, os.path.join(out, f'string_sim_{label}.png'), label)
    return res, logs, w


def plot(w, path, label):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    for fp in ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', '/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc'):
        if os.path.exists(fp):
            font_manager.fontManager.addfont(fp)
            plt.rcParams['font.family'] = font_manager.FontProperties(fname=fp).get_name()
            break
    plt.rcParams['axes.unicode_minus'] = False
    tr = w.trace
    fig, ax = plt.subplots(1, 3, figsize=(17, 5.5))
    xs = [t[2] for t in tr]
    ys = [t[1] for t in tr]
    ax[0].plot(xs, ys, lw=0.8, color='#2563eb')
    for (x, y), name, c in ((sc.ANCHOR_XY, '실 고정 나사', 'k'), (sc.POST_XY, '감는 나사', 'r'), (sc.HOLE_XY, '구멍', 'g')):
        ax[0].plot([y], [x], 'o', color=c)
        ax[0].annotate(name, (y, x), textcoords='offset points', xytext=(4, 6), fontsize=8)
    ax[0].set_xlabel('y [mm]'); ax[0].set_ylabel('x [mm]'); ax[0].set_title('TCP 경로 (위에서)'); ax[0].set_aspect('equal')
    ax[0].invert_xaxis()
    t = [x[0] for x in tr]
    ax[1].plot(t, [x[4] for x in tr], color='#dc2626', label='실 장력 [N]')
    ax[1].axhline(sc.TENSION_N, ls='--', color='#dc2626', lw=0.6)
    ax1b = ax[1].twinx()
    ax1b.plot(t, [x[5] / 360.0 for x in tr], color='#7c3aed', label='감긴 바퀴')
    ax[1].set_title('실 장력과 감긴 바퀴 수'); ax[1].set_xlabel('걸음'); ax[1].set_ylabel('N'); ax1b.set_ylabel('바퀴')
    ax[2].plot(t, [x[3] for x in tr], color='#0f766e', label='TCP z')
    ax2b = ax[2].twinx()
    ax2b.plot(t, [x[6] for x in tr], color='#ea580c', label='나사 물린 깊이 (−1 = 빠짐)')
    ax[2].set_title('높이와 나사 물린 깊이'); ax[2].set_xlabel('걸음'); ax[2].set_ylabel('z [mm]'); ax2b.set_ylabel('mm')
    fig.suptitle(label)
    for a in (ax[1], ax[2]):
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='.')
    ap.add_argument('--e0', type=float, default=None, help='처음 나사가 구멍에 물린 깊이 [mm] (기본: 나사 길이와 머리 높이로 계산)')
    ap.add_argument('--hole-err', type=float, nargs=2, default=[0.8, -0.6])
    ap.add_argument('--string-len', type=float, default=None)
    ap.add_argument('--label', default='sim')
    o = ap.parse_args()
    # 11-22 형님: 360도 한 바퀴면 풀리게 해 둠 → 물린 깊이 = 피치 x 1바퀴
    e0 = o.e0 if o.e0 is not None else sc.SCREW_PITCH * sc.TIGHTEN_TURN_DEG / 360.0
    L = o.string_len
    if L is None:
        # 영상처럼 감고 구멍에 왔을 때 거의 팽팽한 길이: A→P + 2바퀴 감김 + P→구멍 + 여유 3mm
        L = (math.hypot(sc.ANCHOR_XY[0] - sc.POST_XY[0], sc.ANCHOR_XY[1] - sc.POST_XY[1])
             + 4.0 * math.radians(360 * (sc.SPIRAL_TURNS - 0.5)) + math.hypot(sc.HOLE_XY[0] - sc.POST_XY[0], sc.HOLE_XY[1] - sc.POST_XY[1]) + 3.0)
    res, logs, w = run(e0, o.hole_err, L, o.out, o.label)
    print(f'== {o.label}: 처음 물린 깊이 {e0:.1f}mm, 구멍 오차 {o.hole_err}, 실 길이 {L:.0f}mm')
    for act, r in res.items():
        print(f'-- {act}: {"완료" if r["ok"] else "멈춤"}, 최대 장력 {r["T_max"]:.1f}N, 최대 감김 {r["wrap_max"] / 360:.2f}바퀴, '
              f'끝 감김 {r["wrap_end"] / 360:.2f}바퀴, 끝 장력 {r["T_end"]:.1f}N, 끝 물린 깊이 {r["e_end"]:.1f}mm')
        groups = {}
        for t, ph, kind, msg in r['events']:
            g = groups.setdefault((kind, ph), [msg, 0])
            g[1] += 1
        for (kind, ph), (msg, n) in groups.items():
            print(f'   [{kind}] ({ph}) {msg}' + (f'  (x{n})' if n > 1 else ''))
    print('-- 동작 로그(요약)')
    for t, m in logs:
        print(f'   {t:6d} {m}')


if __name__ == '__main__':
    sys.exit(main())
