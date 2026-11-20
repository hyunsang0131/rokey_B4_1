# 로봇을 움직이는 가장 기본 동작 모음. DSR_ROBOT2 함수를 감싸서
#   - 매 동작 전에 "정지 요청"과 로봇 상태(STANDBY)를 확인하고
#   - 실패하면 상태와 알람을 넣어 BedError 를 던진다.
# 위쪽 코드(bed_tasks, bed_process)는 DSR_ROBOT2 를 직접 부르지 않는다.

import time

from bed_inspect import bed_config as cfg

STATE_STANDBY = 1
STATE_NAMES = {0: 'INITIALIZING', 1: 'STANDBY', 2: 'MOVING', 3: 'SAFE_OFF',
               4: 'TEACHING', 5: 'SAFE_STOP', 6: 'EMERGENCY_STOP', 7: 'HOMMING',
               8: 'RECOVERY', 9: 'SAFE_STOP2', 10: 'SAFE_OFF2', 15: 'NOT_READY'}

# 시작할 때 준비됐는지 기다릴 서비스 (DSR_ROBOT2 안의 클라이언트 이름)
_CLIENTS = ['_ros2_get_robot_state', '_ros2_get_robot_mode', '_ros2_set_robot_mode',
            '_ros2_get_last_alarm', '_ros2_get_current_posj', '_ros2_get_current_posx',
            '_ros2_get_current_tcp', '_ros2_get_tool_force', '_ros2_movej', '_ros2_movel',
            '_ros2_movec', '_ros2_check_motion', '_ros2_task_compliance_ctrl', '_ros2_release_compliance_ctrl']


class BedError(Exception):
    """공정을 멈춰야 하는 오류."""


class BedStop(Exception):
    """사용자가 정지를 눌렀다."""


class BedRobot:
    def __init__(self, dr, node, log, stop_event=None):
        """dr: DSR_ROBOT2 모듈, node: DSR 용 노드, log: 로그 함수(문자열 1개), stop_event: 정지 요청."""
        from DR_common2 import posj, posx
        self.dr, self.node, self.log = dr, node, log
        self.posj, self.posx = posj, posx
        self.stop_event = stop_event
        self._grip_cli = None
        self._stop_cli = None
        self.tcp = None
        self.emu = None                     # 에뮬레이터 가짜 힘 (bed_emu). 실제 로봇에서는 None
        self._stop_cli = None               # motion/move_stop (probe_down 에서 처음 쓸 때 만든다)

    # ── 준비 ──────────────────────────────────────────────────────────
    def wait_services(self, timeout=30.0):
        """필요한 서비스가 뜰 때까지 기다린다. 안 뜨면 BedError."""
        clients = [getattr(self.dr, n) for n in _CLIENTS if hasattr(self.dr, n)]
        if cfg.GRIPPER_MODE == 'io':
            clients.append(self.dr._ros2_set_digital_output)
        else:
            from onrobot_rg_msgs.srv import SetCommand
            self._grip_cli = self.node.create_client(SetCommand, '/onrobot/sendCommand')
            clients.append(self._grip_cli)
        end = time.monotonic() + timeout
        for cli in clients:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.log(f'서비스 기다리는 중: {cli.srv_name}')
                if time.monotonic() > end:
                    raise BedError(f'서비스가 없습니다: {cli.srv_name} (bringup 이 떠 있는지 확인)')

    def prepare(self):
        """움직일 수 있는 상태인지 확인하고 AUTONOMOUS 모드로 바꾼다."""
        self.require_standby('시작')
        if self.dr.get_robot_mode() != self.dr.ROBOT_MODE_AUTONOMOUS:
            self.log('MANUAL → AUTONOMOUS 모드로 전환')
            if self.dr.set_robot_mode(self.dr.ROBOT_MODE_AUTONOMOUS) != 0:
                raise BedError('AUTONOMOUS 모드 전환 실패')
        tcp = self.dr.get_tcp()
        self.tcp = tcp if isinstance(tcp, str) else ''
        return self.tcp

    # ── 상태 확인 ─────────────────────────────────────────────────────
    def state(self):
        try:
            return self.dr.get_robot_state()
        except Exception:                      # 서비스 응답이 없으면 DSR_ROBOT2 안에서 예외가 난다
            return -1

    def fault_text(self):
        s = self.state()
        text = f'로봇 상태 {s} {STATE_NAMES.get(s, "UNKNOWN")}'
        try:
            a = self.dr.get_last_alarm()
            if a != -1 and (a.level or a.group or a.index):
                text += f', 마지막 알람 level={a.level} group={a.group} index={a.index}'
        except Exception:
            pass
        return text

    def check_stop(self):
        if self.stop_event is not None and self.stop_event.is_set():
            raise BedStop()

    def require_standby(self, what):
        self.check_stop()
        if self.state() != STATE_STANDBY:
            raise BedError(f'{what}: 움직일 수 없습니다. {self.fault_text()} '
                           '→ 원인 확인 후 servo_on 으로 복구하세요.')

    def _done(self, ret, what):
        """이동 함수의 반환값 확인. 정지 요청이 먼저, 그 다음 실패."""
        self.check_stop()
        if ret != 0:
            raise BedError(f'{what} 실패. {self.fault_text()}')

    def sleep(self, sec):
        """정지 요청을 확인하면서 기다린다."""
        end = time.monotonic() + sec
        while True:
            self.check_stop()
            left = end - time.monotonic()
            if left <= 0:
                return
            time.sleep(min(0.05, left))

    # ── 이동 ──────────────────────────────────────────────────────────
    def movej_to(self, point, slow=False):
        """교시한 위치로 이동. 관절각(posj)이 있으면 movej, 없으면(팀원 파일처럼 posx 만 있으면) posx 로 직선 이동."""
        self.require_standby(point['name'])
        if not point.get('posj'):
            vel, acc = (cfg.VELX_SLOW, cfg.ACCX_SLOW) if slow else (cfg.TEACH_VELX, cfg.TEACH_ACCX)
            self._done(self.dr.movel(self.posx([float(v) for v in point['posx']]), vel=vel, acc=acc,
                                     ref=self.dr.DR_BASE, mod=self.dr.DR_MV_MOD_ABS), f'[{point["name"]}] 이동')
            return
        vel, acc = (cfg.VELJ_SLOW, cfg.ACCJ_SLOW) if slow else (cfg.VELJ, cfg.ACCJ)
        self._done(self.dr.movej(self.posj(list(point['posj'])), vel=vel, acc=acc),
                   f'[{point["name"]}] 이동')

    def move_z(self, dz, vel=None, acc=None):
        """지금 위치에서 베이스 Z 로 dz 만큼 수직 이동."""
        if abs(dz) < 1e-3:
            return
        self.require_standby('수직 이동')
        ret = self.dr.movel(self.posx([0.0, 0.0, float(dz), 0.0, 0.0, 0.0]),
                            vel=cfg.VELX if vel is None else vel, acc=cfg.ACCX if acc is None else acc,
                            ref=self.dr.DR_BASE, mod=self.dr.DR_MV_MOD_REL)
        self._done(ret, f'수직 이동 {dz:+.1f} mm')

    def movel_to(self, pose, vel, acc, what):
        """베이스 기준 절대 좌표 pose(posx 6개)로 직선 이동."""
        self.require_standby(what)
        ret = self.dr.movel(self.posx([float(v) for v in pose]), vel=vel, acc=acc,
                            ref=self.dr.DR_BASE, mod=self.dr.DR_MV_MOD_ABS)
        self._done(ret, what)

    def movec_guarded(self, via, end, vel, acc, what, f_limit, compliance=None, poll=0.05, end_tol=8.0, f0=None):
        """커버처럼 망가지기 쉬운 것을 잡고 원호 이동 (amovec). 보호 방법은 _guarded 참고."""
        def start():
            return self.dr.amovec(self.posx([float(v) for v in via]), self.posx([float(v) for v in end]),
                                  vel=vel, acc=acc, ref=self.dr.DR_BASE, mod=self.dr.DR_MV_MOD_ABS)
        return self._guarded(start, end, what, f_limit, compliance, poll, end_tol, f0)

    def movel_guarded(self, pose, vel, acc, what, f_limit, compliance=None, poll=0.05, end_tol=8.0, f0=None):
        """커버처럼 망가지기 쉬운 것을 잡고 직선 이동 (amovel, 베이스 절대 좌표). 보호 방법은 _guarded 참고."""
        def start():
            return self.dr.amovel(self.posx([float(v) for v in pose]), vel=vel, acc=acc,
                                  ref=self.dr.DR_BASE, mod=self.dr.DR_MV_MOD_ABS)
        return self._guarded(start, pose, what, f_limit, compliance, poll, end_tol, f0)

    def _guarded(self, start, end, what, f_limit, compliance, poll, end_tol, f0):
        """비동기 이동 start() 를 외력 감시하며 끝까지 기다린다. 경첩 보호를 위해

          - (compliance 가 강성 6개면) 순응 제어를 켜고 움직인다: 원호가 실제 궤적과 조금 달라도 억지로 끌지 않는다
          - amovec/amovel(비동기)로 출발시키고, poll 초마다 외력을 읽는다. 출발 전 값과의 차이(크기)가 f_limit 을 넘으면
            즉시 정지하고 BedError. 정지 버튼도 같은 방법으로 받는다.
          - 끝나면 목표에서 end_tol 이내에 왔는지 확인한다.
        멈춘 경우에도 그리퍼는 열지 않는다(놓으면 뚜껑이 떨어진다). 순응 제어는 항상 끈다.
        f0: 외력 기준값 [Fx, Fy, Fz]. None 이면 출발 전에 잰다. 이어지는 구간은 앞 구간의 기준값을 그대로 넘긴다
            (뚜껑을 들고 있는 동안 다시 재면 이미 걸린 힘이 0 으로 지워진다).
        돌려주는 값: (이동 중 최대 외력 [N], 기준값 f0).
        """
        self.require_standby(what)
        peak = 0.0
        if compliance is not None:
            if self.dr.task_compliance_ctrl(list(compliance)) != 0:
                raise BedError(f'{what}: 순응 제어를 켜지 못했습니다. {self.fault_text()}')
        try:
            # 순응 제어를 켠 직후에 바로 이동 명령을 보내면 실물 컨트롤러가 명령을 무시한다 (2026-11-19 실물: 두 번째 구간이
            # 매번 출발 안 함. 첫 구간은 기준값을 재느라 기다려서 됐음). 그래서 구간마다 항상 기다린다.
            if compliance is not None:
                self.sleep(cfg.COVER_COMPLIANCE_SETTLE)
            if f0 is None:
                # 기준값: 잡을 때 생긴 미세한 당김이 풀린 상태
                f0 = self.read_force_vec(5)
            ret = start()
            if ret != 0:
                raise BedError(f'{what} 시작 실패. {self.fault_text()}')
            t0, seen_busy = time.monotonic(), False
            while True:
                if self.stop_event is not None and self.stop_event.is_set():
                    self.halt()
                    raise BedStop()
                f = self.read_force_vec(1)
                df = sum((a - b) ** 2 for a, b in zip(f, f0)) ** 0.5
                peak = max(peak, df)
                if df > f_limit:
                    self.halt()
                    raise BedError(f'{what}: 외력 {df:.1f}N 이 한계 {f_limit:.0f}N 을 넘어 멈췄습니다 '
                                   '(경첩 보호). 그리퍼는 잡은 채로 두었습니다. 뚜껑을 손으로 받친 뒤 그리퍼를 여세요.')
                st = self.dr.check_motion()
                seen_busy = seen_busy or st == self.dr.DR_STATE_BUSY
                # 명령 직후 아직 출발 전이라 IDLE 로 보일 수 있으므로, 움직이는 것을 한 번 봤거나 1초가 지나야 끝으로 본다
                if st == self.dr.DR_STATE_IDLE and (seen_busy or time.monotonic() - t0 > 1.0):
                    break
                if st == -1 or self.state() not in (STATE_STANDBY, 2):      # 2 = MOVING
                    raise BedError(f'{what} 중 로봇이 멈췄습니다. {self.fault_text()}')
                time.sleep(poll)
        finally:
            if compliance is not None:
                self.dr.release_compliance_ctrl()
        self.require_standby(what)
        cur = self.cur_posx()
        err = sum((a - b) ** 2 for a, b in zip(cur[:3], end[:3])) ** 0.5
        if err > end_tol:
            if not seen_busy:                   # 명령은 받았는데 움직이지 않음 (컨트롤러가 거부: 관절 한계, 특이점 등)
                raise BedError(f'{what}: 이동이 시작되지 않았습니다 (목표까지 {err:.1f}mm 남음). 컨트롤러가 명령을 '
                               f'거부한 것으로 보입니다. 펜던트 알람을 확인하세요. {self.fault_text()} '
                               f'목표 {[round(v, 2) for v in end]}. 그리퍼는 잡은 채로 두었습니다.')
            raise BedError(f'{what}: 끝난 위치가 목표에서 {err:.1f}mm 벗어났습니다 (허용 {end_tol:.0f}mm). '
                           '그리퍼는 잡은 채로 두었습니다.')
        return peak, f0

    def halt(self):
        """지금 동작을 빠르게 정지 (DR_QSTOP, 서보는 켜진 채)."""
        import rclpy
        from dsr_msgs2.srv import MoveStop
        if self._stop_cli is None:
            name = self.dr._ros2_movej.srv_name.replace('move_joint', 'move_stop')
            self._stop_cli = self.node.create_client(MoveStop, name)
        req = MoveStop.Request()
        req.stop_mode = 1
        future = self._stop_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
    def probe_down(self, dist, vel, acc, tare, f_contact):
        """멈추지 않고 dist 만큼 천천히 내려가면서 힘을 계속 읽는다. f_contact 가 넘으면 바로 멈춘다.

        반환 (닿았나, 멈춘 높이, 내려가는 동안 본 가장 큰 |힘-영점|).
        움직이는 중의 힘 추정에는 관절 마찰이 섞일 수 있다 → 잘못 멈춰도 부르는 쪽이 계단식으로 다시 확인한다.
        """
        self.require_standby('탐색 하강')
        z_end = self.cur_z() - dist
        ret = self.dr.amovel(self.posx([0.0, 0.0, -float(dist), 0.0, 0.0, 0.0]), vel=vel, acc=acc,
                             ref=self.dr.DR_BASE, mod=self.dr.DR_MV_MOD_REL)
        if ret != 0:
            raise BedError(f'탐색 하강 시작 실패. {self.fault_text()}')
        hit, f_peak, stopped = False, 0.0, False
        try:
            while True:
                self.check_stop()
                f = abs(self.read_fz(1) - tare)
                f_peak = max(f_peak, f)
                if f >= f_contact:
                    hit = True
                    break
                if self.cur_z() <= z_end + 0.05 or self.dr.check_motion() == self.dr.DR_STATE_IDLE:
                    break
            self.stop_motion()
            stopped = True
        finally:
            if not stopped:
                self.stop_motion()
        return hit, self.cur_z(), f_peak

    def stop_motion(self):
        """진행 중인 이동을 부드럽게 멈추고(soft stop) 끝날 때까지 기다린다."""
        if self.dr.check_motion() != self.dr.DR_STATE_IDLE:
            from dsr_msgs2.srv import MoveStop
            if self._stop_cli is None:      # halt() 와 같은 클라이언트를 같이 쓴다
                self._stop_cli = self.node.create_client(MoveStop, self.dr._srv_name_prefix + 'motion/move_stop')
                self._stop_cli.wait_for_service(timeout_sec=2.0)
            req = MoveStop.Request()
            req.stop_mode = self.dr.DR_SSTOP
            import rclpy
            fut = self._stop_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=2.0)
        self.dr.mwait()
        end = time.monotonic() + 3.0
        while self.state() != STATE_STANDBY and time.monotonic() < end:
            time.sleep(0.05)

    def movec(self, via, end, vel, acc):
        """지금 위치 → via → end 를 지나는 원호 이동 (posx 사용: 교시 때와 TCP 가 같아야 함)."""
        self.require_standby('원호 이동')
        ret = self.dr.movec(self.posx(list(via['posx'])), self.posx(list(end['posx'])),
                            vel=vel, acc=acc, ref=self.dr.DR_BASE)
        self._done(ret, f'원호 이동 [{via["name"]} → {end["name"]}]')

    # ── 읽기 ──────────────────────────────────────────────────────────
    def cur_posj(self):
        p = self.dr.get_current_posj()
        if p == -1 or p is None:
            raise BedError('현재 관절각을 읽지 못했습니다.')
        return list(p)

    def cur_posx(self):
        """현재 TCP 좌표 [x, y, z, A, B, C] (베이스 기준)."""
        res = self.dr.get_current_posx(ref=self.dr.DR_BASE)
        if not isinstance(res, tuple) or res[0] is None:
            raise BedError('현재 위치를 읽지 못했습니다.')
        return [float(v) for v in res[0]]

    def cur_z(self):
        return self.cur_posx()[2]

    def read_force_vec(self, n):
        """베이스 기준 외력 [Fx, Fy, Fz] 를 n 번 읽은 평균 [N]."""
        vals = []
        for _ in range(n):
            f = self.dr.get_tool_force(ref=self.dr.DR_BASE)
            if f != -1 and f is not None:
                vals.append(f[:3])
        if not vals:
            raise BedError('힘을 읽지 못했습니다.')
        return [sum(v[i] for v in vals) / len(vals) for i in range(3)]

    def cur_xyz(self):
        res = self.dr.get_current_posx(ref=self.dr.DR_BASE)
        if not isinstance(res, tuple) or res[0] is None:
            raise BedError('현재 위치를 읽지 못했습니다.')
        return [float(v) for v in res[0][:3]]

    def read_fz(self, n):
        """베이스 Z축 힘을 n 번 읽은 평균 [N]."""
        vals = self.read_fz_list(n)
        return sum(vals) / len(vals)

    def read_fz_list(self, n):
        """베이스 Z축 힘을 n 번 읽은 값 목록 [N] (노이즈 크기를 볼 때 쓴다)."""
        if self.emu is not None:
            xyz = self.cur_xyz()
            return [self.emu.fz(xyz) for _ in range(n)]
        vals = []
        for _ in range(n):
            f = self.dr.get_tool_force(ref=self.dr.DR_BASE)
            if f != -1 and f is not None:
                vals.append(f[2])
        if not vals:
            raise BedError('힘을 읽지 못했습니다.')
        return vals

    # ── 그리퍼 ────────────────────────────────────────────────────────
    def grip(self, key):
        """cfg.GRIP_WIDTH 의 이름으로 그리퍼를 움직인다 (예: 'block_open', 'block_grip', 'press')."""
        self.check_stop()
        width = cfg.GRIP_WIDTH[key]
        if cfg.GRIPPER_MODE == 'io':
            # 닫기 (DO1, DO2) = (1, 0), 열기 (0, 1), 너비로 잡기 (1, 1)
            opening = key.endswith('open')
            want = {cfg.DO_CLOSE: 0 if opening else 1,
                    cfg.DO_OPEN: 1 if opening or key in cfg.GRIP_DO_BOTH else 0}
            # 끄는 것부터 쓴다: 바뀌는 사이에 (0, 0) 만 거치고 다른 동작 조합을 잠깐이라도 지나지 않는다
            for idx, val in sorted(want.items(), key=lambda kv: kv[1]):
                self.dr.set_digital_output(idx, val)
        else:
            import rclpy
            from onrobot_rg_msgs.srv import SetCommand
            req = SetCommand.Request()
            # 'c' 완전히 닫기, 'o' 완전히 열기, 숫자는 1/10 mm 단위 폭 (실제 OnRobot 드라이버)
            req.command = 'c' if width <= 0 else 'o' if width >= 110 else str(int(round(width * 10)))
            if self.emu is not None and req.command not in ('c', 'o'):
                # 에뮬레이터의 가상 그리퍼(gripper_virtual_node)는 숫자를 손가락 관절각[rad]으로 받는다
                # (열림 -0.5585 ~ 닫힘 0.7854). 폭 mm 를 그 범위로 바꿔야 RViz 에서 벌어지고 닫히는 게 보인다
                req.command = f'{0.785398 - (0.785398 + 0.558505) * width / 110.0:.4f}'
            future = self._grip_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            res = future.result() if future.done() else None
            if res is None or not res.success:
                raise BedError(f'그리퍼 명령 실패 ({key}, 폭 {width} mm)')
        self.sleep(cfg.GRIP_WAIT)
        if self.emu is not None:
            self.emu.on_grip(key, self.cur_xyz())
