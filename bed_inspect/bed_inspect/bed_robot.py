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
            '_ros2_movec']


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
        self.tcp = None

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
        """교시한 위치(관절각)로 이동."""
        self.require_standby(point['name'])
        vel, acc = (cfg.VELJ_SLOW, cfg.ACCJ_SLOW) if slow else (cfg.VELJ, cfg.ACCJ)
        self._done(self.dr.movej(self.posj(list(point['posj'])), vel=vel, acc=acc),
                   f'[{point["name"]}] 이동')

    def turn_j6(self, deg, vel, acc):
        """6번 관절만 deg 만큼 돌린다(그리퍼가 제자리에서 회전)."""
        self.require_standby('J6 회전')
        j6 = self.cur_posj()[5]
        if abs(j6 + deg) > cfg.J6_LIMIT:
            raise BedError(f'J6 가 범위를 넘습니다 (현재 {j6:.1f} + {deg:+.1f} deg, 한계 ±{cfg.J6_LIMIT:.0f}). '
                           '교시할 때 J6 를 반대쪽으로 돌려 두거나 회전 각도를 줄이세요.')
        self._done(self.dr.movej(self.posj([0.0, 0.0, 0.0, 0.0, 0.0, float(deg)]), vel=vel, acc=acc,
                                 mod=self.dr.DR_MV_MOD_REL), f'J6 {deg:+.1f} deg 회전')

    def move_z(self, dz, vel=None, acc=None):
        """지금 위치에서 베이스 Z 로 dz 만큼 수직 이동."""
        if abs(dz) < 1e-3:
            return
        self.require_standby('수직 이동')
        ret = self.dr.movel(self.posx([0.0, 0.0, float(dz), 0.0, 0.0, 0.0]),
                            vel=cfg.VELX if vel is None else vel, acc=cfg.ACCX if acc is None else acc,
                            ref=self.dr.DR_BASE, mod=self.dr.DR_MV_MOD_REL)
        self._done(ret, f'수직 이동 {dz:+.1f} mm')

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

    def cur_z(self):
        res = self.dr.get_current_posx(ref=self.dr.DR_BASE)
        if not isinstance(res, tuple) or res[0] is None:
            raise BedError('현재 위치를 읽지 못했습니다.')
        return float(res[0][2])

    def read_fz(self, n):
        """베이스 Z축 힘을 n 번 읽은 평균 [N]."""
        vals = []
        for _ in range(n):
            f = self.dr.get_tool_force(ref=self.dr.DR_BASE)
            if f != -1 and f is not None:
                vals.append(f[2])
        if not vals:
            raise BedError('힘을 읽지 못했습니다.')
        return sum(vals) / len(vals)

    # ── 그리퍼 ────────────────────────────────────────────────────────
    def grip(self, key):
        """cfg.GRIP_WIDTH 의 이름으로 그리퍼를 움직인다 (예: 'block_open', 'block_grip', 'press')."""
        self.check_stop()
        width = cfg.GRIP_WIDTH[key]
        if cfg.GRIPPER_MODE == 'io':
            opening = key.endswith('open')
            self.dr.set_digital_output(cfg.DO_CLOSE, 0 if opening else 1)
            self.dr.set_digital_output(cfg.DO_OPEN, 1 if opening else 0)
        else:
            import rclpy
            from onrobot_rg_msgs.srv import SetCommand
            req = SetCommand.Request()
            # 'c' 완전히 닫기, 'o' 완전히 열기, 숫자는 1/10 mm 단위 폭
            req.command = 'c' if width <= 0 else 'o' if width >= 110 else str(int(round(width * 10)))
            future = self._grip_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            res = future.result() if future.done() else None
            if res is None or not res.success:
                raise BedError(f'그리퍼 명령 실패 ({key}, 폭 {width} mm)')
        self.sleep(cfg.GRIP_WAIT)
