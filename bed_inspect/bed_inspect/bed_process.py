# 공정 노드: 화면(bed_hmi)에서 명령을 받아 9단계 파이프라인을 실행한다.
#
#   ros2 run bed_inspect bed_process                 # 교시 파일: ~/ws_cobot_pjt/teach_points/bed.json
#   ros2 run bed_inspect bed_process 파일경로.json    # 교시 파일 직접 지정
#
# 먼저 떠 있어야 하는 것: m0609_rg2_bringup (실제 로봇) 또는 bed_sim (가짜 로봇)
#
# 주고받는 토픽 (전부 std_msgs/String, 내용은 JSON)
#   /bed/cmd     화면 → 공정   {"cmd": "start", "mode": "replace"} 등 (아래 표)
#   /bed/status  공정 → 화면   지금 상태 전체(단계, 측정 결과, 교체 계획, 재고). 바뀔 때마다 통째로 보낸다
#   /bed/log     공정 → 화면   로그 한 줄
#
# 명령
#   {"cmd": "start", "mode": "measure"}                        측정만 (1~3단계)
#   {"cmd": "start", "mode": "replace"}                        불량 블록 교체 모드 (1~9단계)
#   {"cmd": "start", "mode": "repair", "target": "auto"}       자동 보수: 빈칸 채우기 + 주변과 맞지 않는 모듈 교체
#   {"cmd": "start", "mode": "repair", "target": "user", "profile": ["hard", "soft", ...]}
#       사용자 지정: 칸(1~6번 순서)마다 원하는 종류. 다른 종류가 들어 있으면 바꾸고, 빈칸도 그 종류로 채운다
#       ※ start 에 "confirm": true 를 넣으면 교체 계획을 보여주고 "confirm"/"cancel" 을 기다린다
#   {"cmd": "calibrate"}                                       지금 블록들을 측정해서 높이별 기준 강성으로 저장
#   {"cmd": "home"}                                            홈으로 이동
#   {"cmd": "set_inventory", "bed": [...], "rack": [...], "cover_open": false}
#   {"cmd": "stop"}                                            정지 (실행 중에도 받는다)
#
# 스레드 구조
#   - 메인 스레드: DSR_ROBOT2 로 로봇을 움직인다. (DSR_ROBOT2 는 한 스레드에서만 써야 한다)
#   - 통신 스레드: /bed/cmd 를 받는다. 그래서 로봇이 움직이는 중에도 "정지"가 바로 들어온다.

import contextlib
import json
import queue
import statistics
import sys
import threading
import os
import time
import traceback

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
import DR_init

from bed_inspect import bed_config as cfg
from bed_inspect import bed_math, bed_store
from bed_inspect.bed_math import PlanError

DR_init.__dsr__id = cfg.ROBOT_ID
DR_init.__dsr__model = cfg.ROBOT_MODEL

# 늦게 켠 화면도 마지막 상태를 받을 수 있게 "마지막 1개 보관" 으로 보낸다
STATUS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


class BedIO:
    """화면과 주고받는 통로. 별도 노드 + 별도 스레드로 돌아서, 로봇이 움직이는 동안에도 명령을 받는다."""

    def __init__(self, stop_srv_name):
        from dsr_msgs2.srv import MoveStop
        self._MoveStop = MoveStop
        self.node = rclpy.create_node('bed_process_io')
        self.pub_status = self.node.create_publisher(String, '/bed/status', STATUS_QOS)
        self.pub_log = self.node.create_publisher(String, '/bed/log', 50)
        self.pub_curve = self.node.create_publisher(String, '/bed/curve', 20)   # 누르기 한 번의 힘-변위 곡선 (화면 그래프용)
        self.node.create_subscription(String, '/bed/cmd', self._on_cmd, 10)   # 터미널 bed_cmd 용 (응답 없음)
        # 서비스: HMI(cushion_bridge 경유)가 명령을 보내고 받았는지 응답을 받는다 (2026-11-20 형님)
        self.last_state = 'starting'
        try:
            from bed_interfaces.srv import Command
            self.node.create_service(Command, '/bed/command', self._on_cmd_srv)
        except ImportError:                 # bed_interfaces 를 빌드/소스 안 한 터미널: 토픽만으로 동작
            self.node.get_logger().warn('bed_interfaces 가 없어 명령 서비스(/bed/command) 없이 토픽만 씁니다. '
                                        'source install/setup.bash 를 다시 하세요.')
        self.stop_cli = self.node.create_client(MoveStop, stop_srv_name)
        # 비상정지 (HMI 비상정지 버튼, 2026-11-20): 빠른 정지 + 서보 끄기, 해제는 서보 켜기
        from dsr_msgs2.srv import ServoOff, SetRobotControl
        self._ServoOff, self._SetRobotControl = ServoOff, SetRobotControl
        sys_prefix = stop_srv_name.replace('motion/move_stop', 'system/')
        self.servo_off_cli = self.node.create_client(ServoOff, sys_prefix + 'servo_off')
        self.ctl_cli = self.node.create_client(SetRobotControl, sys_prefix + 'set_robot_control')
        self.estopped = False
        self.cmds = queue.Queue()           # 메인 스레드가 꺼내 가는 명령
        self.stop_event = threading.Event()
        self.answer = queue.Queue()         # 교체 전 확인의 답 ('confirm' / 'cancel')
        self.busy = False
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            self._exec.spin()
        except Exception:                   # 종료할 때 나는 예외는 무시
            pass

    def _on_cmd(self, msg):
        self._handle(msg.data)

    def _on_cmd_srv(self, req, res):
        res.accepted, res.message = self._handle(req.command)
        res.state = self.last_state or ''
        return res

    def _handle(self, text):
        """명령 하나 처리. (받았나, 설명) 을 돌려준다 (서비스 응답에 그대로 씀)."""
        try:
            d = json.loads(text)
            c = d['cmd']
        except (ValueError, KeyError, TypeError):
            self.log(f'알 수 없는 명령: {text!r}')
            return False, f'알 수 없는 명령: {text!r}'
        if c == 'estop':
            return self._estop()
        if c == 'estop_reset':
            return self._estop_reset()
        if c == 'stop':
            self.stop_event.set()
            sent = False
            if self.busy and self.stop_cli.service_is_ready():
                req = self._MoveStop.Request()
                req.stop_mode = 1           # DR_QSTOP: 빠른 정지(서보는 켜진 채)
                self.stop_cli.call_async(req)
                sent = True
            self.log('정지 요청을 받았습니다.')
            return True, '정지 요청 받음' + (' (로봇 빠른 정지 명령 보냄)' if sent else ' (움직이는 중 아님)')
        if c in ('confirm', 'cancel'):
            if self.last_state != 'confirm':
                return False, '확인을 기다리는 중이 아닙니다.'
            self.answer.put(c)
            return True, '진행' if c == 'confirm' else '취소'
        if self.estopped:
            return False, '비상정지 상태입니다. 안전을 확인하고 HMI "초기화"(비상정지 해제)를 먼저 하세요.'
        if self.busy:
            self.log(f'실행 중이라 "{c}" 명령은 무시합니다. (정지만 받습니다)')
            return False, f'실행 중이라 "{c}" 명령은 무시합니다. (정지만 받습니다)'
        self.cmds.put(d)
        return True, f'"{c}" 명령 받음'

    def _estop(self):
        """비상정지: 공정 정지 표시를 먼저 세우고(충돌 자동 복구가 서보를 다시 켜지 않게) 빠른 정지 → 서보 끄기."""
        self.estopped = True
        self.stop_event.set()
        sent = []
        if self.stop_cli.service_is_ready():
            req = self._MoveStop.Request()
            req.stop_mode = 0               # DR_QSTOP_STO: 빠른 정지
            self.stop_cli.call_async(req)
            sent.append('빠른 정지')
        if self.servo_off_cli.service_is_ready():
            req = self._ServoOff.Request()
            req.stop_type = 1               # STOP_TYPE_QUICK
            self.servo_off_cli.call_async(req)
            sent.append('서보 끄기')
        self.log(f'⚠ 비상정지 (HMI): {" + ".join(sent) or "로봇 서비스 없음"}')
        if not sent:
            return False, '로봇 서비스(move_stop/servo_off)가 없습니다. 펜던트 비상정지를 누르세요.'
        return True, '비상정지: ' + ' + '.join(sent) + ' 요청 보냄'

    def _estop_reset(self):
        """비상정지 해제: 서보 켜기 (SAFE_OFF → STANDBY). 공정은 다시 시작해야 한다."""
        if self.busy:
            return False, '공정이 아직 정리 중입니다. 잠시 뒤 다시 누르세요.'
        if not self.ctl_cli.service_is_ready():
            return False, '로봇 서비스(set_robot_control)가 없습니다.'
        req = self._SetRobotControl.Request()
        req.robot_control = 3               # CONTROL_RESET_SAFE_OFF: 서보 켜기
        self.ctl_cli.call_async(req)
        self.estopped = False
        self.stop_event.clear()
        self.log('비상정지 해제: 서보 켜기 요청 (로봇 상태는 HMI 에서 확인)')
        return True, '비상정지 해제: 서보 켜기 요청 보냄. 다시 시작하세요.'

    def log(self, text):
        self.node.get_logger().info(text)
        self.pub_log.publish(String(data=time.strftime('%H:%M:%S ') + text))

    def send_curve(self, kind, slot, res):
        curve = res.get('curve') or []
        if curve:
            self.pub_curve.publish(String(data=json.dumps(
                {'kind': kind, 'slot': slot, 'z0': res.get('z0'), 'k': res.get('k'),
                 'curve': [[round(float(z), 3), round(float(f), 3)] for z, f in curve]})))

    def send_status(self, status):
        self.last_state = status.get('state')
        self.pub_status.publish(String(data=json.dumps(status, ensure_ascii=False)))

    def close(self):
        self._exec.shutdown(timeout_sec=1.0)
        self.node.destroy_node()


class BedProcess:
    def __init__(self, io, teach_file):
        self.io = io
        self.log = io.log
        self.teach_file = teach_file
        self.robot = None
        self.tasks = None
        self.points = {}
        self.axes = None
        # 강성 맵 (2026-11-20 형님): launch 한 번 동안 기억. 침대·트레이 칸마다 {'k', 'empty', 'tall', 'too_soft', 'src'}.
        # 처음 한 번 재고, 그 뒤로는 맵으로 판단·교체하며 옮길 때마다 고친다. 공정을 새로 켜면 비어 있다 (다시 잰다)
        self.map = {'bed': None, 'tray': None}
        self.tall_dz = None
        self.xy = []
        self.inv = bed_store.load_inventory()
        self.world = bed_store._load_json('world.json', {'cover_open': False, 'uncertain': False})
        self.world.pop('tight', None)               # 예전(롤러 쓰던 때) 기록에 남아 있을 수 있는 값
        self.calib = bed_store.load_calib()
        self.status = {
            'state': 'starting', 'mode': None, 'step': 0, 'step_name': '', 'detail': '',
            'message': '공정 노드를 시작하는 중입니다.', 'active_slot': None,
            'first': None, 'second': None, 'plan': None,
            'layout': {'rows': cfg.BED_ROWS, 'cols': cfg.BED_COLS, 'heights': cfg.INVENTORY_HEIGHTS,
                       'warn': cfg.WARN_RATIO, 'fail': cfg.FAIL_RATIO},
            'step_names': cfg.STEP_NAMES,
        }
        self.publish()

    # ── 상태 알리기 ───────────────────────────────────────────────────
    def publish(self, **changes):
        self.status.update(changes)
        self.status['inventory'] = {'bed': self.inv['bed'], 'rack': self.inv['rack']}
        self.status['world'] = self.world
        self.status['calib'] = self.calib
        self.status['map'] = getattr(self, 'map', None)       # 강성 맵 (침대·트레이, 화면 표시용)
        self.status['stamp'] = time.time()
        self.io.send_status(self.status)

    def step(self, n, detail=''):
        self.log(f'[{n}단계] {cfg.STEP_NAMES[n]}' + (f' - {detail}' if detail else ''))
        self.publish(step=n, step_name=cfg.STEP_NAMES[n], detail=detail)

    def _save_world(self, **changes):
        self.world.update(changes)
        bed_store._save_json('world.json', self.world)

    @contextlib.contextmanager
    def _critical(self):
        """도중에 멈추면 기록과 실제가 달라질 수 있는 동작(커버, 블록 옮기기)을 감싼다.

        시작할 때 '확인 필요' 표시를 파일에 남기고, 끝까지 성공했을 때만 지운다.
        중간에 정지/오류/전원 꺼짐이 생기면 표시가 남아서, 사람이 확인하기 전에는 다시 시작하지 않는다.
        """
        self._save_world(uncertain=True)
        yield
        self._save_world(uncertain=False)

    # ── 시작 준비 ─────────────────────────────────────────────────────
    def setup(self, dr, node):
        """교시 파일을 읽고 로봇 서비스를 기다린다. 문제가 있으면 BedError."""
        from bed_inspect.bed_robot import BedError, BedRobot
        from bed_inspect.bed_tasks import BedTasks
        try:
            self.points = bed_store.fill_defaults(bed_store.load_teach(self.teach_file))
        except (OSError, ValueError, KeyError) as e:
            raise BedError(f'교시 파일을 읽을 수 없습니다: {self.teach_file} ({e})')
        missing = bed_store.missing_names(self.points)
        if missing:
            raise BedError('교시 파일에 없는 위치: ' + ', '.join(missing))
        self.xy = [tuple(self.points[n]['posx'][:2]) for n in cfg.P_BED]
        self.axes = bed_math.bed_axes(self.xy, cfg.BED_ROWS, cfg.BED_COLS)
        if self.axes['pitch_col'] < 1.0 or self.axes['pitch_row'] < 1.0:
            raise BedError('침대 칸 좌표가 겹칩니다. bed_1 ~ bed_6 교시를 확인하세요.')
        self.robot = BedRobot(dr, node, self.log, self.io.stop_event)
        self.tasks = BedTasks(self.robot, self.points, self.log)
        self.robot.wait_services()
        self._wrap_tasks()
        self.log(f'교시 파일: {self.teach_file} / 칸 간격: 열 {self.axes["pitch_col"]:.1f}mm, '
                 f'행 {self.axes["pitch_row"]:.1f}mm')

    # ── 명령 하나 실행 ────────────────────────────────────────────────
    def run(self, cmd):
        from bed_inspect.bed_robot import BedError, BedStop
        c = cmd.get('cmd')
        self.io.stop_event.clear()
        while not self.io.answer.empty():
            self.io.answer.get()
        self.io.busy = True
        try:
            if c == 'set_inventory':
                self._set_inventory(cmd)
                return
            if c not in ('start', 'calibrate', 'home'):
                self.log(f'알 수 없는 명령: {c}')
                return
            mode = cmd.get('mode') if c == 'start' else c
            if mode not in ('measure', 'replace', 'repair', 'calibrate', 'home'):
                self.log(f'알 수 없는 모드: {mode}')
                return
            if self.world.get('uncertain'):
                # 홈 이동도 막는다: 그리퍼가 커버나 블록을 물고 있을 수 있어서 그대로 움직이면 위험하다
                raise BedError('지난번에 커버/블록 동작 중에 멈췄습니다. 그리퍼가 물고 있는 것을 손으로 빼고, '
                               '실제 블록 위치, 커버, 천 상태를 확인해 "재고 수정"에서 저장한 뒤 다시 시작하세요.')
            self.publish(state='running', mode=mode, message='', detail='', active_slot=None, emergency=None,
                         active_rack=None, active_move=None,
                         target=cmd.get('target') if c == 'start' else None,       # 'user' = 주문 모드 (화면 색 구분용)
                         **({} if mode == 'home' else {'first': None, 'second': None, 'plan': None}))
            tcp = self.robot.prepare()
            if mode == 'home':
                self.tasks.home()
                self.publish(state='idle', mode=None, step=0, step_name='', message='홈 위치입니다.')
            elif mode == 'calibrate':
                self._calibrate()
            else:
                self._pipeline(mode, cmd, tcp)
                if getattr(self, '_string_open', False):
                    # 예외 상황 2 (11-22 형님): 실을 풀고 블록을 썼으면, 주문을 다 만든 뒤 실을 다시 고정해서 마무리
                    self._string_op('fix')
        except BedStop:
            self.log('정지했습니다.')
            self.publish(state='stopped', active_slot=None,
                         message='정지했습니다. 로봇 주변을 확인한 뒤 "홈으로" 또는 다시 시작을 누르세요.'
                                 + self._uncertain_note())
        except (BedError, PlanError) as e:
            self.log(f'오류: {e}')
            self.publish(state='error', active_slot=None, message=str(e) + self._uncertain_note())
        except Exception as e:                  # DSR_ROBOT2 의 인자 오류(DR_Error) 등 예상 못 한 것
            self.log('예상하지 못한 오류:\n' + traceback.format_exc())
            self.publish(state='error', active_slot=None,
                         message=f'예상하지 못한 오류: {e}' + self._uncertain_note())
        finally:
            self.io.busy = False

    def _uncertain_note(self):
        return ' ※ 커버/블록 동작 중이었습니다. 실제 상태를 확인하고 "재고 수정"을 저장하세요.' \
            if self.world.get('uncertain') else ''

    def _set_inventory(self, cmd):
        bed = [None if h is None else float(h) for h in cmd.get('bed', self.inv['bed'])]
        rack = [None if h is None else float(h) for h in cmd.get('rack', self.inv['rack'])]
        err = bed_store.check_inventory(bed, rack)
        if err:
            self.log(f'재고 수정 실패: {err}')
            self.publish(message=f'재고 수정 실패: {err}')
            return
        self.inv = {'bed': bed, 'rack': rack}
        bed_store.save_inventory(self.inv)
        self._save_world(cover_open=bool(cmd.get('cover_open', False)) and cfg.COVER_ENABLED,
                         uncertain=False)
        self.log('재고를 저장했습니다.')
        self.publish(state='idle', message='재고를 저장했습니다.')

    # ── 측정 (2, 8단계) ──────────────────────────────────────────────
    def _measure(self, key, step_no, only=None):
        """6칸을 차례로 누르고, 판정과 기울기를 붙여 status[key] 에 넣는다.
        only: 이 칸들(0부터)만 누르고 나머지는 침대 맵 값을 그대로 쓴다 (재측정 때 고친 칸만)."""
        blocks = []
        for i in range(len(cfg.P_BED)):
            if only is not None and i not in only and self.map['bed'] is not None:
                b = self._blocks_from_map('bed')[i]
                b['note'] = '맵 값 (재측정 안 함)'
                blocks.append(b)
                continue
            self.step(step_no, f'{i + 1}번 블록 ({i + 1}/{len(cfg.P_BED)})')
            self.publish(active_slot=i + 1)
            res = self.tasks.press(i, self.inv['bed'][i])
            # 팀원 repair_test 방식은 다시 재지 않고 한 번 잰 결과로 빈칸을 판정한다 (빈칸 바닥에 닿은 곡선이 '직선 아님'으로
            # 나와도 접촉 높이가 낮으면 빈칸). 예전 방식만 '직선 아님'일 때 한 번 더 잰다
            if res['error'] == 'bad_fit' and not getattr(self, '_rt_active', False):
                self.log(f'{i + 1}번: {res["note"]} → 다시 측정')
                res = self.tasks.press(i, self.inv['bed'][i])
            k = '-' if res['k'] is None else f'{res["k"]:.2f} N/mm'
            pr = res.get('probe')
            if pr:      # repair_test 처럼: 빠른 탐색이 어디서 멈췄고 내려가는 동안 힘이 얼마였는지
                k += f'  [탐색 {"닿음" if pr["hit"] else "안 닿음"} z {pr["z"]:.1f}, 최대 {pr["f_peak"]:.1f}N]'
            self.log(f'{i + 1}번: 강성 {k}' + (f' ({res["note"]})' if res['note'] else ''))
            self.io.send_curve('bed', i + 1, res)
            blocks.append(res)
            self.publish(**{key: {'blocks': self._slim(blocks), 'tilt': None, 'done': False}})
        bed_math.judge(blocks, self.calib, cfg.WARN_RATIO, cfg.FAIL_RATIO)
        tilt = bed_math.measured_tilt(self.xy, [b['z_surf'] for b in blocks], cfg.BED_ROWS, cfg.BED_COLS)
        fname = bed_store.save_measure_log(key, blocks, tilt)
        self.log(f'측정 기록 저장: {fname}')
        self.publish(active_slot=None, **{key: {'blocks': self._slim(blocks), 'tilt': tilt, 'done': True}})
        return blocks, tilt

    @staticmethod
    def _slim(blocks):
        """화면으로 보낼 때는 곡선 원본을 뺀다(파일에는 저장됨)."""
        return [{k: v for k, v in b.items() if k != 'curve'} for b in blocks]

    def _check_rack_empty(self, j):
        """예외 처리: 보관대 j번(0부터)에 큐브를 놓기 전에, 빈손으로 살짝 닿아 정말 비었는지 확인한다.

        기록(재고)과 달리 큐브가 있으면, 아무것도 집지 않은 상태로 홈에 가서 멈춘다 (위에 내려놓아 부딪히지 않게).
        """
        from bed_inspect.bed_robot import BedError
        self.step(cfg.STEP_TAKE, f'보관대 {j + 1}번이 비었는지 확인 (살짝 닿기)')
        res = self.tasks.touch_rack(j)
        if res['z0'] is None:
            self.log(f'보관대 {j + 1}번: 비어 있음 확인')
            return
        self.log(f'보관대 {j + 1}번: 기록과 달리 큐브가 있습니다 (윗면 {res["z0"]:.1f} mm)')
        self.tasks.home()
        raise BedError(f'보관대 {j + 1}번이 기록과 달리 비어 있지 않습니다. 이 모듈은 옮기지 않고 멈췄습니다. '
                       '보관대를 확인해 "재고 수정"에서 고친 뒤 다시 시작하세요.')

    def _cover(self, step_no, opening):
        if not cfg.COVER_ENABLED:
            self.step(step_no, '커버가 없어 건너뜀')
            return
        self.step(step_no)
        with self._critical():
            if opening:
                self.tasks.cover_open()
            else:
                self.tasks.cover_close()
        self._save_world(cover_open=opening)

    def _recover(self):
        """지난번에 커버가 열린 채 끝났으면 먼저 닫는다."""
        if self.world.get('cover_open') and cfg.COVER_ENABLED:
            self.log('커버가 열린 상태로 기록되어 있습니다 → 먼저 닫습니다.')
            with self._critical():
                self.tasks.cover_close()
            self._save_world(cover_open=False)

    # ── 기준값 저장 ───────────────────────────────────────────────────
    def _calibrate(self):
        from bed_inspect.bed_robot import BedError
        self._rt_restore()
        if cfg.REPAIR_STYLE == 'repair_test':
            self._rt_use('bed')                         # 기준값도 같은 측정 방법으로
        self.step(1)
        self.tasks.home()
        self._recover()
        blocks, _ = self._measure('first', cfg.STEP_MEASURE)
        bad = [b['slot'] for b in blocks if b['k'] is None]
        self.tasks.home()
        if bad:
            raise BedError(f'{bad}번 블록을 측정하지 못해 기준값을 저장하지 않았습니다.')
        self.calib = bed_math.make_calib(blocks, self.calib)
        bed_store.save_calib(self.calib)
        bed_math.judge(blocks, self.calib, cfg.WARN_RATIO, cfg.FAIL_RATIO)
        text = ', '.join(f'{float(h):g}mm: {k:.2f} N/mm' for h, k in sorted(self.calib.items(), key=lambda x: float(x[0])))
        self.log(f'기준 강성 저장: {text}')
        self.publish(state='done', step=0, step_name='', detail='', message=f'기준 강성을 저장했습니다. ({text})',
                     first={'blocks': self._slim(blocks), 'tilt': self.status['first']['tilt'], 'done': True})

    # ── 9단계 파이프라인 ─────────────────────────────────────────────
    def _pipeline(self, mode, cmd, tcp):
        from bed_inspect.bed_robot import BedError
        # 높은 큐브 판별(z)은 주문 모드(target user)에서만. 첫 불량 검사(자동 보수)·측정에서는 높이를 보지 않는다 (2026-11-20 형님)
        self.tall_dz = cfg.TALL_DZ if (mode == 'repair' and cmd.get('target', 'auto') == 'user') else None
        self._restock_n = 0
        self._string_open = False          # 예외 상황 2 에서 실 고정을 풀었는지 (공정 끝에 다시 고정)
        self._order_mode = (mode == 'repair' and cmd.get('target', 'auto') == 'user')   # 주문 모드
        # 주문 모드는 트레이에서 가져갈 때와 침대에 구성한 뒤 높이를 재야 한다 → 높이를 안 본 공정(기본 공정)이 만든 맵은 쓰지 않는다
        if self.tall_dz is not None and not getattr(self, 'map_tall', False):
            self.map = {'bed': None, 'tray': None}
        self.map_tall = self.tall_dz is not None
        if mode == 'repair':
            profile = cmd.get('profile')
            if cmd.get('target', 'auto') == 'user' and (
                    not isinstance(profile, list) or len(profile) != len(cfg.P_BED)
                    or any(p not in cfg.PROFILE_NAMES for p in profile)):
                raise BedError(f'사용자 지정 목표가 잘못되었습니다: {profile!r} (칸 {len(cfg.P_BED)}개, hard/soft)')
        if mode in ('replace', 'repair') and cfg.COVER_ENABLED:
            # 커버는 좌표(posx)로 움직이므로, 좌표를 잴 때와 TCP 가 같아야 한다
            if cfg.COVER_TCP is not None and not cfg.VIRTUAL and tcp not in (cfg.COVER_TCP, cfg.SIM_TCP):   # 가짜 로봇·가상 모드는 검사 안 함
                raise BedError(f'현재 TCP("{tcp}")가 커버 좌표를 잴 때("{cfg.COVER_TCP}")와 다릅니다. '
                               'TCP 를 맞추거나 bed_config 의 COVER_* 좌표를 다시 재세요.')
            c = self.tasks.cover
            self.log(f'커버({cfg.COVER_MODE}): 경첩 y {c["hinge"][1]:.0f} z {c["hinge"][2]:.0f}, 반지름 {c["radius"]:.0f}mm, '
                     f'경유 {c["via_deg"]:.0f}도, 끝 {c["end_deg"]:.0f}도, '
                     f'순응 제어 {"켬" if cfg.COVER_COMPLIANCE else "끔"}, 외력 한계 {cfg.COVER_F_LIMIT:g}N, TCP "{tcp}"')

        self._rt_restore()                              # 지난 실행이 repair_test 설정을 바꿔 놓았으면 기본값으로
        if cfg.REPAIR_STYLE == 'repair_test':
            self._rt_use('bed')     # 모든 모드(측정만·불량 교체·자동 보수·주문) 같은 측정: repair_test --pad, 빠른 빈칸 탐색
        self.step(1)
        self.tasks.home()
        self._recover()
        use_map = (mode == 'repair' and cfg.REPAIR_STYLE == 'repair_test' and cfg.USE_STIFFNESS_MAP
                   and self.map['bed'] is not None)
        if use_map:
            self.step(cfg.STEP_MEASURE, '저장된 침대 맵 사용 (이번 launch 에서 이미 잼)')
            blocks, tilt = self._blocks_from_map('bed'), None
            self.publish(first={'blocks': self._slim(blocks), 'tilt': None, 'done': True})
            self.log('침대 맵: ' + self._map_text('bed'))
        else:
            blocks, tilt = self._measure('first', cfg.STEP_MEASURE)

        # 3단계: 확인 + 계획
        self.step(cfg.STEP_CHECK)
        fails = [b['slot'] for b in blocks if b['grade'] == 'fail']
        nodata = [b['slot'] for b in blocks if b['grade'] == 'nodata']
        note = (f' 측정 실패 {nodata}번은 다시 확인이 필요합니다.' if nodata else '')
        if mode == 'repair':
            moves, plan = self._plan_repair(blocks, cmd)
            if not moves:
                return
        elif mode == 'measure':
            self.tasks.home()
            msg = f'불량 {fails}번.' if fails else '불량 없음.'
            self.publish(state='warning' if (fails or nodata) else 'done', message=msg + note)
            return

        elif mode == 'replace':
            if not fails:
                self.tasks.home()
                self.publish(state='warning' if nodata else 'done', message='불량 블록이 없습니다.' + note)
                return
            moves, skipped = bed_math.plan_replace(self.inv['bed'], self.inv['rack'], [s - 1 for s in fails])
            plan = {'type': 'replace', 'faulty': fails, 'skipped': [s + 1 for s in skipped]}
            if not moves:
                self.tasks.home()
                self.publish(state='warning', plan=dict(plan, moves=[]),
                             message=f'불량 {fails}번. 같은 높이의 예비 블록이 없어 교체하지 못했습니다.')
                return
        if cfg.P_DISCARD not in self.points and any(m['dst'][0] == 'discard' for m in moves):
            self.tasks.home()
            raise BedError('버리는 통(discard) 위치가 교시 파일에 없습니다. 교체 트레이에 빈자리를 하나 만들거나 '
                           'discard 를 교시하세요.')
        plan['moves'] = [bed_math.move_text(m) for m in moves]
        plan['done'] = 0
        for t in plan['moves']:
            self.log('  계획: ' + t)
        self.publish(plan=plan)

        if cmd.get('confirm'):
            self.publish(state='confirm', message='교체 계획을 확인하고 "진행" 또는 "취소"를 누르세요.')
            if self._wait_answer() == 'cancel':
                self.tasks.home()
                self.publish(state='stopped', message='교체를 취소했습니다.')
                return
            self.publish(state='running', message='')

        # 4단계: 커버 열기
        self._cover(cfg.STEP_OPEN, opening=True)

        # 5~6단계: 블록 옮기기 (하나 옮길 때마다 재고 파일을 갱신)
        for n, mv in enumerate(moves):
            if mv['dst'][0] == 'rack' and self.map['tray'] is None:
                self._check_rack_empty(mv['dst'][1])        # 맵이 없을 때만 빈자리를 살짝 닿아 확인
            self.step(cfg.STEP_PUT if mv['dst'][0] == 'bed' else cfg.STEP_TAKE, bed_math.move_text(mv))
            self.publish(active_move={'src': [mv['src'][0], mv['src'][1] + 1], 'dst': [mv['dst'][0], mv['dst'][1] + 1]})
            with self._critical():
                self.tasks.move_block(mv)
                bed_math.apply_move(self.inv['bed'], self.inv['rack'], mv)
                bed_store.save_inventory(self.inv)
                self._map_move(mv)
            plan['done'] = n + 1
            self.publish(plan=plan)

        self.publish(active_move=None, active_rack=None)
        # 7단계: 커버 닫기
        self._cover(cfg.STEP_CLOSE, opening=False)

        # 8~9단계: 재측정과 확인
        only = None
        if (mode == 'repair' and cfg.REMEASURE_ONLY_FIXED and self.map['bed'] is not None
                and (plan.get('empties') or plan.get('mismatch'))):
            only = sorted(set(i - 1 for i in plan['empties'] + plan['mismatch']))
            self.log(f'재측정: 고친 칸 {[i + 1 for i in only]}번만 다시 잽니다 (나머지는 맵 값)')
        blocks2, tilt2 = self._measure('second', cfg.STEP_REMEASURE, only)
        if mode == 'repair' and cfg.REPAIR_STYLE == 'repair_test':
            bed_math.find_empties(blocks2, cfg.EMPTY_DZ, self.tall_dz)
            self._map_store('bed', blocks2, 'remeasure')
            for b in blocks2:
                b['grade'] = None                       # 판정은 _finish_repair 가 다시 한다
        if mode == 'repair':
            self._finish_repair(blocks2, plan)
            return
        self.step(cfg.STEP_DONE)
        self.tasks.home()
        fails2 = [b['slot'] for b in blocks2 if b['grade'] == 'fail']
        nodata2 = [b['slot'] for b in blocks2 if b['grade'] == 'nodata']
        note2 = (f' 측정 실패 {nodata2}번.' if nodata2 else '')
        if mode == 'replace':
            if fails2 or nodata2 or plan['skipped']:
                msg = '교체 후에도 확인이 필요합니다.'
                if fails2:
                    msg += f' 불량 {fails2}번.'
                if plan['skipped']:
                    msg += f' 예비 블록이 없어 못 바꾼 칸 {plan["skipped"]}번.'
                self.publish(state='warning', message=msg + note2)
            else:
                self.publish(state='done', message=f'{fails}번 블록을 교체했고, 재측정 결과 전부 정상입니다.')

    # ── 자동 보수: 빈칸 채우기 + 맞지 않는 모듈 교체 ──────────────────
    def _press_rack_all(self, plan):
        """보관대 큐브를 2단계로 잰다. 자리마다 측정 결과(dict) 목록을 돌려준다 (침대 기준 강성 'k_bed' 포함).

        ① 살짝 닿기: 전부 닿는 순간 멈추고 윗면 높이만 잰다 → 빈자리와 높은 큐브를 찾는다
        ② 끝까지 누르기: 보통 높이 큐브만 눌러 강성을 잰다 (높은 큐브는 더 누르지 않는다)
        """
        slots = [j for j, h in enumerate(self.inv['rack']) if h is not None]
        touch = []
        for n, j in enumerate(slots):
            self.step(cfg.STEP_CHECK, f'보관대 {j + 1}번 살짝 닿기 ({n + 1}/{len(slots)})')
            res = self.tasks.touch_rack(j)
            res['k_bed'] = None
            touch.append(res)
            z = '닿지 않음' if res['z0'] is None else f'윗면 {res["z0"]:.1f} mm'
            self.log(f'보관대 {j + 1}번 살짝 닿기: {z} (경계 {res["thr"]:.2f} N, 노이즈 σ {res["sigma"]:.2f} N)')
            plan['rack'].append({'slot': j + 1, 'k': None, 'k_bed': None, 'note': res['note'], 'z0': res['z0']})
            self.publish(plan=plan)
        bed_math.find_empties(touch, cfg.EMPTY_DZ, self.tall_dz)
        zs = sorted(r['z0'] for r in touch if r['z0'] is not None)
        z_ref = zs[len(zs) // 2] if zs else None
        for r, p in zip(touch, plan['rack']):
            p.update(empty=r['grade'] == 'empty', tall=r['tall'],
                     dz=None if r['z0'] is None or z_ref is None else r['z0'] - z_ref)
        tall = [r['slot'] for r in touch if r['tall']]
        self.log(f'높은 큐브: 보관대 {tall}번' if tall else '높은 큐브 없음')
        self.publish(plan=plan)

        out = []
        todo = [n for n, r in enumerate(touch) if r['grade'] != 'empty' and not r['tall']]
        for n, t in enumerate(touch):
            if n not in todo:
                out.append(t)
                continue
            j = t['slot'] - 1
            self.step(cfg.STEP_CHECK, f'보관대 {j + 1}번 누르기 ({todo.index(n) + 1}/{len(todo)})')
            self.publish(active_rack=j + 1)
            res = self.tasks.press_rack(j)
            if res['error'] == 'bad_fit':
                self.log(f'보관대 {j + 1}번: {res["note"]} → 다시 측정')
                res = self.tasks.press_rack(j)
            res['slot'] = j + 1
            res['k_bed'] = None if res['k'] is None else res['k'] * cfg.RACK_TO_BED
            out.append(res)
            k = '-' if res['k'] is None else f'{res["k"]:.2f} N/mm (침대 기준 {res["k_bed"]:.2f})'
            self.log(f'보관대 {j + 1}번 강성: {k}' + (f' ({res["note"]})' if res['note'] else ''))
            plan['rack'][n].update(k=res['k'], k_bed=res['k_bed'], note=res['note'])
            self.publish(plan=plan)
        # 누른 결과까지 합쳐 다시 판정 (높은 큐브는 닿은 높이, 나머지는 표면 높이로)
        bed_math.find_empties(out, cfg.EMPTY_DZ, self.tall_dz)
        for r, t in zip(out, touch):
            r['tall'] = r['tall'] or t['tall']
        return out

    def _judge_repair(self, blocks, plan):
        """빈칸과 맞지 않는 칸을 판정한다. (빈칸, 맞지 않는 칸) 번호(0부터)를 돌려준다."""
        if plan['target_mode'] == 'auto' and cfg.REPAIR_STYLE == 'repair_test':
            from pjt_test1.repair_test import judge_bed
            return judge_bed(blocks, cfg.REPAIR_RATIO)
        empties = bed_math.find_empties(blocks, cfg.EMPTY_DZ, self.tall_dz)
        if plan['target_mode'] == 'user':
            mism = bed_math.judge_profile(blocks, plan['profile'], plan['thr'], plan['centers'])
        else:
            mism = bed_math.judge_auto(blocks, cfg.MISMATCH_RATIO)
        return empties, mism

    def _fix_heights(self, blocks, rack):
        """비전이 없으므로, 잰 결과로 재고 기록(칸마다 큐브가 있는지, 높이)을 고친다. 집는 높이가 여기서 정해진다."""
        fixed = []

        def h_of(b):
            return None if b['grade'] == 'empty' else cfg.TALL_HEIGHT if b.get('tall') else cfg.H_REF
        for i, b in enumerate(blocks):
            h = h_of(b)
            if self.inv['bed'][i] != h:
                fixed.append(f'침대 {i + 1}번 {"빈칸" if h is None else f"{h:g}mm"}')
                self.inv['bed'][i] = h
        for r in rack:
            j, h = r['slot'] - 1, h_of(r)
            if self.inv['rack'][j] != h:
                fixed.append(f'보관대 {j + 1}번 {"빈칸" if h is None else f"{h:g}mm"}')
                self.inv['rack'][j] = h
        if fixed:
            bed_store.save_inventory(self.inv)
            self.log('재고 기록 수정 (측정 결과대로): ' + ', '.join(fixed))
            self.publish()

    # ── 팀원 repair_test 방식 (자동 보수) ──────────────────────────────
    def _rt_use(self, side):
        """repair_test 의 측정 설정을 적용한다. side: 'bed' / 'tray'."""
        from pjt_test1 import repair_test as rt
        if not hasattr(self, '_rt_default'):
            self._rt_default = {k: getattr(cfg, k) for k in ('F_TARGET', 'F_FIT_MIN', 'PRESS_STEP', 'SEARCH_MAX',
                                                            'PROBE_VEL', 'PRESS_DEPTH_MAX', 'PRESS_APPROACH_DZ',
                                                            'RACK_PRESS_APPROACH_DZ')}
        s = (rt.PAD if cfg.REPAIR_PAD else rt.BARE) if side == 'bed' else rt.BARE
        cfg.F_TARGET, cfg.F_FIT_MIN, cfg.PRESS_STEP = s['force'], s['from'], s['step']
        self._rt_active = True
        sim = getattr(self.robot, 'tcp', None) == cfg.SIM_TCP
        # 16/20mm 는 팀원 실물 교시 높이(접근 114/109 → 큐브 윗면 106~110/97~98)에 맞춘 값. 가짜 로봇은 높이가 달라 기본값
        cfg.SEARCH_MAX = self._rt_default['SEARCH_MAX'] if sim else rt.SEARCH[rt.BED if side == 'bed' else rt.TRAY]
        # 가짜 로봇(bed_sim)은 멈추지 않고 내려가는 빠른 탐색(amovel + 정지)을 흉내 내지 못한다 → 계단식만
        cfg.PROBE_VEL = 0.0 if sim else rt.PROBE_VEL
        cfg.PRESS_DEPTH_MAX = 0.0 if sim else (cfg.PRESS_DEPTH_MAX_BED if side == 'bed' else cfg.PRESS_DEPTH_MAX_TRAY)
        # 주문 모드: 키 큰 큐브가 있다고 보고 처음부터 높게 접근 (탐색 거리·깊이 한계도 같이 늘려 바닥 기준은 그대로)
        raise_ = cfg.ORDER_APPROACH_RAISE if (self.tall_dz is not None and not sim) else 0.0
        cfg.PRESS_APPROACH_DZ = self._rt_default['PRESS_APPROACH_DZ'] + raise_
        cfg.RACK_PRESS_APPROACH_DZ = self._rt_default['RACK_PRESS_APPROACH_DZ'] + raise_
        cfg.SEARCH_MAX += raise_
        if cfg.PRESS_DEPTH_MAX:
            cfg.PRESS_DEPTH_MAX += raise_
        if raise_:
            self.log(f'주문 모드: 키 큰 큐브 대비 {raise_:g}mm 높게 접근 (빠른 접근 {cfg.PRESS_APPROACH_DZ:g}mm)')
        self.log(f'측정 설정 (repair_test, {"침대" if side == "bed" else "트레이"}): 목표 {s["force"]:g}N, '
                 f'계산 {s["from"]:g}N~, 스텝 {s["step"]:g}mm, {cfg.SEARCH_MAX:g}mm 안 닿으면 빈칸, '
                 f'빠른 탐색 {cfg.PROBE_VEL:g}mm/s, 최대 깊이 {cfg.PRESS_DEPTH_MAX:g}mm')

    def _rt_restore(self):
        self._rt_active = False
        for k, v in getattr(self, '_rt_default', {}).items():
            setattr(cfg, k, v)

    def _plan_repair_rt(self, blocks, plan):
        """repair_test 와 같은 판정·계획: judge_bed → 트레이 누르기 → make_plan. (이동 목록, 계획)."""
        from bed_inspect.bed_robot import BedError
        from pjt_test1 import repair_test as rt
        empties, mism = rt.judge_bed(blocks, cfg.REPAIR_RATIO)
        if self.map['bed'] is None:
            self._map_store('bed', blocks, 'measure')
        self._map_soft('bed', {i: i in mism for i, b in enumerate(blocks) if b['grade'] in ('ok', 'mismatch')})
        plan.update(empties=[i + 1 for i in empties], mismatch=[i + 1 for i in mism])
        self.publish(first=dict(self.status['first'], blocks=self._slim(blocks)), plan=plan)
        self.log(f'빈칸 {plan["empties"] or "없음"}, 맞지 않는 칸 {plan["mismatch"] or "없음"} (repair_test 판정, 비율 '
                 f'{cfg.REPAIR_RATIO:g})')
        nodata = [b['slot'] for b in blocks if b['grade'] == 'nodata']
        if nodata:
            self.log(f'측정 실패 칸 {nodata} 은 판단하지 않습니다.')
        if not empties and not mism:
            self.tasks.home()
            self.publish(state='done', plan=plan, message='모든 칸이 정상입니다. 교체할 것이 없습니다.')
            return [], plan
        # 트레이: 맵이 있으면 그대로, 없으면 6자리를 모두 눌러 잰다 (repair_test 의 measure(TRAY))
        tray, _ = self._press_tray_rt()
        ok_k = [b['k'] for b in blocks if b['grade'] == 'ok' and b.get('k')]
        if ok_k:                # 화면 색: 트레이 큐브도 공정과 같은 기준 (정상 침대 칸 평균 × 비율보다 무르면 불량)
            ref = statistics.mean(ok_k)
            self._map_soft('tray', {j: bool(r.get('too_soft')) or r['k'] * cfg.RACK_TO_BED < ref * cfg.REPAIR_RATIO
                                    for j, r in enumerate(tray) if r['grade'] != 'empty' and r.get('k')})
        try:
            rp = rt.make_plan(blocks, tray, empties, mism, cfg.RACK_TO_BED, cfg.REPAIR_RATIO)
        except PlanError as e:
            if self._can_restock(e):                    # 예외 상황 2: 재고 부족 → 큐브 탐색해서 채운 뒤 다시 계획
                self._restock(str(e))
                return self._plan_repair_rt(blocks, plan)
            self.tasks.home()
            raise BedError(str(e))
        # 재고 기록을 이번에 잰 것으로 맞춘다 (빈칸 = None, 큐브 = 보통 높이 H_REF).
        # 기본 공정은 보통 큐브만 옮기므로 예전 재고 높이(예: 기준 바꾸기 전 30mm)는 쓰지 않는다
        # (2026-11-20 실물: 트레이 1번이 30 으로 남아 10mm 위에서 멈춰 딱딱한 큐브를 못 잡음)
        self.inv['bed'] = [None if i in empties else cfg.H_REF for i in range(len(cfg.P_BED))]
        self.inv['rack'] = [None if j in rp['tray_empty'] else cfg.H_REF for j in range(len(cfg.P_RACK))]
        bed_store.save_inventory(self.inv)
        # repair_test 계획은 칸 점유를 'cube' 로만 표시한다 → 이동마다 실제 블록 높이를 재고에서 넣는다
        bed_h, rack_h = list(self.inv['bed']), list(self.inv['rack'])
        for m in rp['moves']:
            (sk, si), (dk, di) = m['src'], m['dst']
            src = bed_h if sk == 'bed' else rack_h
            m['h'] = src[si] or cfg.H_REF
            src[si] = None
            if dk == 'bed':
                bed_h[di] = m['h']
            elif dk == 'rack':
                rack_h[di] = m['h']
        plan['rack'] = [{'slot': r['slot'], 'k': r.get('k'), 'grade': r.get('grade')} for r in tray]
        plan.update(choice=[{'slot': i + 1, 'rack': j + 1, 'k_bed': rp['cands'][j] * cfg.RACK_TO_BED,
                             'why': 'mismatch' if i in mism else 'empty'} for i, j in rp['fill']['assign']],
                    target=rp['fill'].get('target'))
        for i, j in rp['fill']['assign']:
            self.log(f'  선택: 트레이 {j + 1}번 ({rp["cands"][j]:.2f} N/mm) → {i + 1}번 '
                     + ('(맞지 않는 모듈 교체)' if i in mism else '(빈칸)'))
        return rp['moves'], plan

    def _press_tray_rt(self):
        """트레이 6자리를 repair_test 와 같은 설정으로 눌러 잰다 (빈자리 = 20mm 안 안 닿음).
        이번 launch 에서 이미 쟀으면(맵) 누르지 않고 맵을 쓴다."""
        if cfg.USE_STIFFNESS_MAP and self.map['tray'] is not None and any(c is None for c in self.map['tray']):
            # 예외 상황 2 로 블록을 넣은 칸만 맵이 비어 있음 → 그 칸만 누르고 나머지는 맵
            todo = [j for j, c in enumerate(self.map['tray']) if c is None]
            old = self._blocks_from_map_partial('tray')
            self._rt_use('tray')
            for j in todo:
                self.step(cfg.STEP_CHECK, f'새로 넣은 트레이 {j + 1}번만 누르기')
                self.publish(active_rack=j + 1)
                res = self.tasks.press_at(self.tasks.pt[cfg.P_RACK[j]], cfg.PRESS_APPROACH_DZ)
                res.update(slot=j + 1)
                k = '-' if res.get('k') is None else f'{res["k"]:.2f} N/mm'
                self.log(f'  트레이 {j + 1}번(새로 넣음): {k}' + (f' ({res.get("note")})' if res.get('note') else ''))
                old[j] = res
            self._rt_use('bed')
            self.publish(active_rack=None)
            tray = old
            tray_empty = bed_math.find_empties(tray, cfg.EMPTY_DZ, self.tall_dz, self._tray_z_ref(tray))
            self._map_store('tray', tray, 'measure')
            return tray, tray_empty
        if cfg.USE_STIFFNESS_MAP and self.map['tray'] is not None:
            self.step(cfg.STEP_CHECK, '저장된 트레이 맵 사용 (이번 launch 에서 이미 잼)')
            tray = self._blocks_from_map('tray')
            self.log('트레이 맵: ' + self._map_text('tray'))
            return tray, bed_math.find_empties(tray, cfg.EMPTY_DZ, self.tall_dz, self._tray_z_ref(tray))
        self._rt_use('tray')
        tray = []
        for j in range(len(cfg.P_RACK)):
            self.step(cfg.STEP_CHECK, f'트레이 {j + 1}번 누르기 ({j + 1}/{len(cfg.P_RACK)})')
            self.publish(active_rack=j + 1)             # HMI 위치 지도: 지금 누르는 트레이 칸
            res = self.tasks.press_at(self.tasks.pt[cfg.P_RACK[j]], cfg.PRESS_APPROACH_DZ)
            res.update(slot=j + 1)
            k = '-' if res.get('k') is None else f'{res["k"]:.2f} N/mm'
            zs_ = res.get('z_surf') if res.get('z_surf') is not None else res.get('z0')
            self.log(f'  트레이 {j + 1}번: {k}' + (f' (윗면 z {zs_:.1f})' if zs_ is not None else '')
                     + (f' ({res.get("note")})' if res.get('note') else ''))
            tray.append(res)
        self._rt_use('bed')
        self.publish(active_rack=None)
        tray_empty = bed_math.find_empties(tray, cfg.EMPTY_DZ, self.tall_dz, self._tray_z_ref(tray))
        self._map_store('tray', tray, 'measure')
        return tray, tray_empty

    # ── 강성 맵 ──
    def _map_store(self, side, blocks, src):
        """측정 결과(find_empties 를 거친 것)로 맵을 만든다."""
        self.map[side] = [{'k': b.get('k'), 'empty': b.get('grade') == 'empty', 'tall': bool(b.get('tall')),
                           'too_soft': bool(b.get('too_soft')), 'src': src,
                           'z': b.get('z_surf') if b.get('z_surf') is not None else b.get('z0')} for b in blocks]
        self.log(f'{"침대" if side == "bed" else "트레이"} 맵 저장: ' + self._map_text(side))

    def _map_soft(self, side, flags):
        """화면 색을 공정 판정과 같게: 맵 칸에 'soft'(무름 = 기본 공정에서는 불량) 표시. flags {칸: True/False}.
        (화면이 침대·트레이 강성을 한데 섞어 경계를 잡으면 측정 조건이 달라 색이 틀어졌음, 2026-11-20 형님)"""
        cells = self.map.get(side)
        if not cells:
            return
        for i, soft in flags.items():
            if 0 <= i < len(cells) and not cells[i].get('empty'):
                cells[i]['soft'] = bool(soft)
        self.publish()

    def _map_text(self, side):
        return ', '.join(f'{i + 1}번 ' + ('빈칸' if c['empty'] else (f'{c["k"]:.2f}' if c['k'] else '?'))
                         for i, c in enumerate(self.map[side]))

    def _blocks_from_map_partial(self, side):
        """맵에 빈(None) 칸이 섞여 있을 때: 있는 칸만 _blocks_from_map 모양으로, None 칸은 None."""
        keep = self.map[side]
        filled = [c if c is not None else {'k': None, 'empty': True, 'tall': False, 'too_soft': False, 'src': '?', 'z': None}
                  for c in keep]
        self.map[side] = filled
        try:
            out = self._blocks_from_map(side)
        finally:
            self.map[side] = keep
        return [b if keep[i] is not None else None for i, b in enumerate(out)]

    def _blocks_from_map(self, side):
        """맵 → 측정 결과와 같은 모양 (find_empties / judge 가 그대로 쓰게)."""
        out = []
        zs = [c['z'] for c in self.map[side] if not c['empty'] and not c['tall'] and c.get('z') is not None]
        z_norm = statistics.median(zs) if zs else 100.0       # 높이를 모르는 칸(옮겨 온 큐브)은 보통 큐브 윗면 높이로
        for i, c in enumerate(self.map[side]):
            if c['empty']:
                b = {'k': None, 'z0': None, 'z_surf': None, 'error': 'no_contact', 'note': '맵: 빈칸'}
            else:
                z = c['z'] if c.get('z') is not None else (
                    z_norm + (cfg.TALL_HEIGHT - cfg.H_REF if c['tall'] else 0.0))
                b = {'k': c['k'], 'z0': z, 'z_surf': z, 'error': 'too_deep' if c['too_soft'] else None,
                     'note': f'맵 ({c["src"]})'}
            b.update(slot=i + 1, grade=None, height=None, curve=[])
            out.append(b)
        return out

    def _map_move(self, mv):
        """큐브 하나를 옮긴 대로 맵을 고친다 (강성은 큐브를 따라간다. 트레이↔침대는 RACK_TO_BED 로 환산)."""
        side = {'bed': 'bed', 'rack': 'tray'}
        (sk, si), (dk, di) = mv['src'], mv['dst']
        if sk not in side or self.map.get(side[sk]) is None:
            return
        cell = dict(self.map[side[sk]][si])
        self.map[side[sk]][si] = {'k': None, 'empty': True, 'tall': False, 'too_soft': False, 'src': 'moved'}
        if dk in side and self.map.get(side[dk]) is not None:
            if cell['k'] and sk != dk:
                cell['k'] = cell['k'] * cfg.RACK_TO_BED if dk == 'bed' else cell['k'] / cfg.RACK_TO_BED
            cell['src'] = 'moved'
            cell['z'] = None                            # 다른 자리로 옮겼으니 윗면 높이는 그 자리 기준으로 모름
            self.map[side[dk]][di] = cell

    def _set_measured_inventory(self, bed_empties, tray_empty, blocks=None, tray=None):
        """재고 기록을 이번에 잰 것으로 맞춘다 (빈칸 = None, 큐브 = 보통 높이, z 로 높다고 판별된 큐브 = TALL_HEIGHT).
        높이가 맞아야 잡으러 내려가는 깊이가 맞는다 (키 큰 큐브는 그만큼 덜 내려감)."""
        def h(b):
            return cfg.TALL_HEIGHT if (b is not None and b.get('tall')) else cfg.H_REF
        self.inv['bed'] = [None if i in bed_empties else h(blocks[i] if blocks else None) for i in range(len(cfg.P_BED))]
        self.inv['rack'] = [None if j in tray_empty else h(tray[j] if tray else None) for j in range(len(cfg.P_RACK))]
        bed_store.save_inventory(self.inv)

    def _plan_order_rt(self, blocks, plan):
        """주문(사용자 지정)을 repair_test 측정으로: 말랑한 큐브를 불량으로 보지 않고, 칸마다 주문한 종류
        (부드러움/단단함)와 다르면 교체한다. 종류 경계는 침대 + 트레이에서 잰 강성을 repair_test 비율(--ratio)로 나눠 정한다."""
        from bed_inspect.bed_robot import BedError
        bed_empties = bed_math.find_empties(blocks, cfg.EMPTY_DZ, self.tall_dz)
        if self.map['bed'] is None:
            self._map_store('bed', blocks, 'measure')
        tray, tray_empty = self._press_tray_rt()
        racked = [r for r in tray if r['grade'] != 'empty' and r.get('k')]
        for r in racked:
            r['k_bed'] = r['k'] * cfg.RACK_TO_BED
        items = ([(b['k'], b.get('tall')) for b in blocks if b['grade'] is None and b.get('k')]
                 + [(r['k_bed'], r.get('tall')) for r in racked])
        thr = bed_math.split_classes([k for k, tall in items if not tall], 1.0 / cfg.REPAIR_RATIO)
        ref = self._class_ref()
        if thr is None and ref is not None:
            # 11-22 실물: 블록을 찾아 넣었는데 그것도 같은 종류(딱딱 10.6)라 두 무리로 못 나눔 → 다시 재고 부족으로 갔음.
            # 큐브가 한 종류뿐이면 전에 잰 종류 경계(저장값)로 판정한다
            thr = ref['thr']
            self.log(f'큐브가 한 종류뿐 → 저장된 종류 경계 {thr:.2f} N/mm 로 판정 (부드러움 약 {ref["soft"]:.2f}, 단단함 약 {ref["hard"]:.2f})')
        if thr is None:
            # 11-22 실물: 트레이가 비어 있고 침대 큐브가 한 종류뿐 → 주문에 맞출 다른 종류가 없음 = 재고 부족.
            # 예외 상황 2 로 처리 (실 풀기 → 블록 위치추정해서 트레이로) 하고 다시 계획
            why = Exception('침대와 트레이에 한 종류의 큐브뿐이고 트레이가 비어 주문에 맞출 큐브가 모자랍니다')
            if self._can_restock(why) and not self._string_open:   # 블록을 이미 넣었는데도 한 종류면 또 찾지 않음
                self._restock(str(why))
                return self._plan_order_rt(blocks, plan)
            self.tasks.home()
            raise BedError('침대와 트레이에서 잰 강성이 한 종류뿐이라 부드러움/단단함을 나눌 수 없습니다.')
        plan['thr'] = thr
        plan['centers'] = c = bed_math.class_centers(items, thr)
        if ref is None or thr != ref.get('thr'):
            self._save_class_ref(thr, c)
        self.log(f'종류 경계 {thr:.2f} N/mm (부드러움 약 {c["soft"] or 0:.2f}, 단단함 약 {c["hard"] or 0:.2f}) '
                 f'— repair_test 측정, 비율 {cfg.REPAIR_RATIO:g}')
        mism = bed_math.judge_profile(blocks, plan['profile'], thr, c)
        self._map_soft('bed', {i: b.get('cls') == 'soft' for i, b in enumerate(blocks) if b.get('cls')})
        self._map_soft('tray', {r['slot'] - 1: bed_math.class_of(r['k_bed'], thr, r.get('too_soft'), r.get('tall')) == 'soft'
                                for r in racked})
        empties = [i for i in bed_empties]
        plan.update(empties=[i + 1 for i in empties], mismatch=[i + 1 for i in mism])
        plan['rack'] = [{'slot': r['slot'], 'k': r.get('k'), 'grade': r.get('grade'),
                         'cls': None if r['grade'] == 'empty' or not r.get('k')
                         else bed_math.class_of(r['k_bed'], thr, r.get('too_soft'), r.get('tall'))} for r in tray]
        self.publish(first=dict(self.status['first'], blocks=self._slim(blocks)), plan=plan)
        self.log(f'빈칸 {plan["empties"] or "없음"}, 주문과 다른 칸 {plan["mismatch"] or "없음"}')
        todo = sorted(empties + mism)
        if not todo:
            self.tasks.home()
            self.publish(state='done', plan=plan, message='이미 주문대로 구성되어 있습니다.')
            return [], plan
        # 침대 안의 큐브도 다른 칸으로 옮겨 쓴다 (자리만 바꾸면 되는 경우 트레이에서 찾지 않음, 2026-11-20 형님)
        self._set_measured_inventory(empties, tray_empty, blocks, tray)
        bed_cells = [None if i in empties else {'cls': b.get('cls'), 'h': self.inv['bed'][i] or cfg.H_REF, 'k': b.get('k'),
                                                'keep': b['grade'] == 'nodata'}
                     for i, b in enumerate(blocks)]
        k_of = {r['slot'] - 1: r['k_bed'] for r in racked}
        tray_cells = [None if j in tray_empty else
                      {'cls': bed_math.class_of(k_of[j], thr, tray[j].get('too_soft'), tray[j].get('tall')) if j in k_of else None,
                       'h': self.inv['rack'][j] or cfg.H_REF, 'k': k_of.get(j)} for j in range(len(tray))]
        try:
            moves, choice = bed_math.plan_profile_moves(bed_cells, tray_cells, plan['profile'], c)
        except PlanError as e:
            if self._can_restock(e):                    # 예외 상황 2: 재고 부족 → 큐브 탐색해서 채운 뒤 다시 계획
                self._restock(str(e))
                return self._plan_order_rt(blocks, plan)
            self.tasks.home()
            raise BedError(str(e))
        plan.update(choice=[{'slot': ch['slot'] + 1,
                             'rack': ch['from'][1] + 1 if ch['from'][0] == 'rack' else None,
                             'bed': ch['from'][1] + 1 if ch['from'][0] == 'bed' else None,
                             'k_bed': (k_of.get(ch['from'][1]) if ch['from'][0] == 'rack' else blocks[ch['from'][1]].get('k')),
                             'why': 'mismatch' if ch['slot'] in mism else 'empty'} for ch in choice], target=None)
        for ch in choice:
            kind, j = ch['from']
            self.log(f'  선택: {"침대" if kind == "bed" else "트레이"} {j + 1}번 → 침대 {ch["slot"] + 1}번 '
                     f'(주문 {plan["profile"][ch["slot"]]})')
        return moves, plan

    def _plan_repair(self, blocks, cmd):
        """4단계: 보관대 큐브를 눌러 보고, 빈칸과 맞지 않는 칸을 찾고, 넣을 큐브를 고른다.

        (이동 목록, 계획) 을 돌려준다. 고칠 칸이 없으면 홈으로 가고 결과를 알린 뒤 ([], plan).
        """
        from bed_inspect.bed_robot import BedError
        user = cmd.get('target', 'auto') == 'user'
        plan = {'type': 'repair', 'target_mode': 'user' if user else 'auto',
                'profile': list(cmd['profile']) if user else None, 'thr': None, 'centers': None,
                'empties': [], 'mismatch': [], 'rack': [], 'choice': [], 'moves': [], 'done': 0}
        if cfg.REPAIR_STYLE == 'repair_test':
            return self._plan_order_rt(blocks, plan) if user else self._plan_repair_rt(blocks, plan)
        bed_math.find_empties(blocks, cfg.EMPTY_DZ, self.tall_dz)
        nodata = [b['slot'] for b in blocks if b['grade'] == 'nodata']
        if nodata:
            self.tasks.home()
            raise BedError(f'{nodata}번 칸을 측정하지 못해 판정할 수 없습니다. 다시 측정하세요.')
        self.publish(plan=plan)

        # 보관대를 먼저 눌러 본다: 사용자 지정은 종류를 나누는 데 보관대 값도 쓴다
        rack = self._press_rack_all(plan)
        racked = [r for r in rack if r['grade'] != 'empty']
        if user:
            items = ([(b['k'], b['tall']) for b in blocks if b['grade'] != 'empty']
                     + [(r['k_bed'], r['tall']) for r in racked])
            thr = bed_math.split_classes([k for k, tall in items if not tall], cfg.CLASS_MIN_RATIO)
            if thr is None:
                self.tasks.home()
                raise BedError('침대와 보관대에서 잰 강성이 한 종류뿐이라 부드러움/단단함을 나눌 수 없습니다.')
            plan['thr'] = thr
            plan['centers'] = bed_math.class_centers(items, thr)
            c = plan['centers']
            self.log(f'종류 경계 {thr:.2f} N/mm (부드러움 약 {c["soft"] or 0:.2f}, 단단함 약 {c["hard"] or 0:.2f}'
                     + (f', 높음 약 {c["tall"]:.2f}' if c['tall'] else '') + ')')
            for r, p in zip(rack, plan['rack']):
                if r['grade'] != 'empty':
                    p['cls'] = bed_math.class_of(r['k_bed'], thr, r['too_soft'], r['tall'])
        empties, mism = self._judge_repair(blocks, plan)
        plan.update(empties=[i + 1 for i in empties], mismatch=[i + 1 for i in mism])
        self.publish(first=dict(self.status['first'], blocks=self._slim(blocks)), plan=plan)
        self.log(f'빈칸 {plan["empties"] or "없음"}, 맞지 않는 칸 {plan["mismatch"] or "없음"}')
        self._fix_heights(blocks, rack)

        todo = sorted(empties + mism)
        if not todo:
            self.tasks.home()
            self.publish(state='done', plan=plan, message='빈칸이나 맞지 않는 모듈이 없습니다.')
            return [], plan

        k_of = {r['slot'] - 1: r['k_bed'] for r in racked}
        try:
            if user:
                cands = {r['slot'] - 1: (bed_math.class_of(r['k_bed'], plan['thr'], r['too_soft'], r['tall']), r['k_bed'])
                         for r in racked}
                assign = bed_math.plan_profile(todo, plan['profile'], cands, plan['centers'])
                target = None
            else:
                keep = {b['slot'] - 1: b['k'] for b in blocks if b['grade'] == 'ok'}
                cands = {r['slot'] - 1: r['k_bed'] for r in racked if r['k_bed'] is not None and not r['tall']}
                p = bed_math.plan_fill(todo, keep, cands, cfg.BED_ROWS, cfg.BED_COLS)
                assign, target = p['assign'], p['target']
        except PlanError:
            self.tasks.home()
            raise
        moves = bed_math.plan_repair_moves(self.inv['bed'], self.inv['rack'], assign, mism)
        plan.update(choice=[{'slot': i + 1, 'rack': j + 1, 'k_bed': k_of[j],
                             'why': 'mismatch' if i in mism else 'empty'} for i, j in assign], target=target)
        for i, j in assign:
            k = '-' if k_of[j] is None else f'{k_of[j]:.2f} N/mm'
            self.log(f'  선택: 보관대 {j + 1}번 ({k}) → {i + 1}번 ' + ('(맞지 않는 모듈 교체)' if i in mism else '(빈칸)'))
        return moves, plan

    def _finish_repair(self, blocks2, plan):
        """9단계: 빈칸과 맞지 않는 칸이 남지 않았는지 확인한다."""
        self.step(cfg.STEP_DONE)
        empties, mism = self._judge_repair(blocks2, plan)
        if plan.get('target_mode') == 'user':
            self._map_soft('bed', {i: b.get('cls') == 'soft' for i, b in enumerate(blocks2) if b.get('cls')})
        else:
            self._map_soft('bed', {i: i in mism for i, b in enumerate(blocks2) if b['grade'] in ('ok', 'mismatch')})
        self.publish(second=dict(self.status['second'], blocks=self._slim(blocks2)))
        self.tasks.home()
        nodata = [b['slot'] for b in blocks2 if b['grade'] == 'nodata']
        # 트레이 ↔ 침대 환산 비율 제안: 넣은 큐브의 (침대에서 잰 값 / 트레이에서 잰 값)
        rack_k = {r['slot']: r['k'] for r in plan['rack']}
        ratios = [blocks2[c['slot'] - 1]['k'] / rack_k[c['rack']] for c in plan['choice']
                  if blocks2[c['slot'] - 1].get('k') and rack_k.get(c['rack'])]
        if ratios:
            self.log(f'환산 비율 제안: RACK_TO_BED ≈ {statistics.mean(ratios):.2f} (지금 설정 {cfg.RACK_TO_BED:g})')
        fixed = sorted(plan['empties'] + plan['mismatch'])
        if empties or mism or nodata:
            msg = '보수 후에도 확인이 필요합니다.'
            if empties:
                msg += f' 빈칸으로 보이는 칸 {[i + 1 for i in empties]}번.'
            if mism:
                msg += f' 맞지 않는 칸 {[i + 1 for i in mism]}번.'
            if nodata:
                msg += f' 측정 실패 {nodata}번.'
            self.publish(state='warning', message=msg)
            return
        ks = [b['k'] for b in blocks2 if b.get('k') is not None]
        spread = (statistics.pstdev(ks) / statistics.mean(ks) * 100) if len(ks) > 1 else 0.0
        how = '사용자 지정 배치와 일치' if plan['target_mode'] == 'user' else f'강성 편차 {spread:.0f}%'
        self.publish(state='done', message=f'{fixed}번 칸을 보수했습니다. 재측정 결과 {how}.')

    # ── 예외 상황 1: 충돌 ─────────────────────────────────────────────
    def _wrap_tasks(self):
        """로봇 작업(칸 누르기, 블록 옮기기, 커버, 홈)을 충돌 복구로 감싼다: 충돌 → 복구 → 그 작업을 다시."""
        self._home_raw = self.tasks.home
        for name, kind in (('press', 'press'), ('press_at', 'press'), ('press_rack', 'press'),
                           ('touch_rack', 'press'), ('move_block', 'block'), ('home', 'home'),
                           ('cover_open', 'cover'), ('cover_close', 'cover')):
            if hasattr(self.tasks, name):
                setattr(self.tasks, name, self._collision_safe(getattr(self.tasks, name), kind))

    def _collision_safe(self, fn, kind):
        from bed_inspect.bed_robot import BedCollision, BedError

        def run(*a, **kw):
            n = 0
            while True:
                try:
                    return fn(*a, **kw)
                except BedCollision as e:
                    if not cfg.COLLISION_ENABLED:
                        raise
                    n += 1
                    if n > cfg.COLLISION_RETRY_MAX:
                        raise BedError(f'충돌이 같은 작업에서 {cfg.COLLISION_RETRY_MAX}번 넘게 반복되어 멈춥니다. ({e})')
                    self._on_collision(e, kind, n)
        return run

    def _on_collision(self, e, kind, n):
        """충돌 → 위치 저장 → 보호 정지 해제 → 반대 방향 후퇴 → (커버면 고리 놓고 위로) → 홈."""
        from bed_inspect.bed_robot import BedCollision, BedError, BedStop
        text = '충돌로 인한 긴급 프로토콜'
        pos = [round(v, 2) for v in e.pos] if e.pos else None
        rec = {'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'what': e.what, 'pos': pos,
               'start': [round(v, 2) for v in e.start] if e.start else None, 'state': e.state,
               'step': self.status.get('step'), 'step_name': self.status.get('step_name'), 'retry': n}
        hist = bed_store._load_json('collisions.json', [])           # 충돌 위치 기록 (bed_data/collisions.json)
        hist.append(rec)
        bed_store._save_json('collisions.json', hist[-200:])
        self.log(f'⚠ {text}: "{e.what}" 중 충돌, 위치 {pos} → 정지. 반대 방향으로 {cfg.COLLISION_BACKOFF:g}mm 물러난 뒤 '
                 f'홈으로 가서 이 작업을 다시 합니다 ({n}/{cfg.COLLISION_RETRY_MAX}).')
        self.publish(emergency={'kind': 'collision', 'text': text, 'what': e.what, 'pos': pos, 'retry': n},
                     message=f'{text}: {e.what} 중 충돌')
        try:
            d = self.robot.recover_collision(e, cfg.COLLISION_BACKOFF)
            self.log(f'  후퇴 방향 {[round(v, 2) for v in d]} ({cfg.COLLISION_BACKOFF:g}mm)')
            if kind == 'cover':
                self.robot.grip('cover_open')               # 고리를 놓고 위로 빠진 뒤 홈 (뚜껑을 끌고 가지 않게)
                self.robot.move_z(cfg.COLLISION_COVER_UP)
                self.robot.wait_still('고리 놓고 올라온 뒤')
            self._collision_home(lid_open=kind == 'cover' or bool(self.world.get('cover_open')))
        except BedCollision as e2:
            raise BedError(f'충돌 복구 중 다시 충돌했습니다. 로봇 주변을 확인하고 다시 시작하세요. ({e2})')
        if kind == 'cover':
            self.publish(state='confirm', message=f'{text}: 커버 작업 중 충돌해 고리를 놓았습니다. 뚜껑을 원래 상태로 '
                                                  '두고 "진행"을 누르면 커버 작업을 처음부터 다시 합니다.')
            if self._wait_answer() == 'cancel':
                raise BedStop()
            self.publish(state='running')
        self.publish(emergency=None, message='')
        self.log('충돌 복구 완료 → 하던 작업을 다시 합니다.')

    def _collision_home(self, lid_open):
        """충돌 복구 후 홈으로: 뚜껑이 열려 있고 로봇이 뚜껑 쪽이면 교시한 우회 경로(E1 → E2), 아니면 위로 올라가서 바로."""
        y = self.robot.cur_posx()[1]
        if lid_open and cfg.COVER_ENABLED and cfg.COVER_EXIT_POSJ and y < cfg.COLLISION_DETOUR_Y:
            self.log(f'  뚜껑 열림 + 뚜껑 쪽(y {y:.0f}) → 우회 경로로 홈')
            for i, j in enumerate(cfg.COVER_EXIT_POSJ, 1):
                self.robot.movej_to({'name': f'cover_exit_{i}', 'posj': list(j)}, slow=True)
        else:
            self.log(f'  {"뚜껑 열림, 뚜껑에서 떨어져 있음" if lid_open else "뚜껑 닫힘"} (y {y:.0f}) → '
                     f'{cfg.COLLISION_LIFT:g}mm 올라가서 홈')
            self.robot.move_z(cfg.COLLISION_LIFT, cfg.VELX_SLOW, cfg.ACCX_SLOW)
        self._home_raw()
        self.robot.wait_still('홈 도착 후')

    # ── 예외 상황 2: 재고 부족 ────────────────────────────────────────
    def _can_restock(self, e):
        # 11-22 형님: 재고 부족 예외(실 unfix → 블록 탐지 → 트레이 → 주문 완성 → fix)는 주문 모드에서만.
        # 기본 공정(불량 교체)에서는 아예 일으키지 않는다 (충돌 예외는 기본 공정에도 그대로)
        return (cfg.RESTOCK_ENABLED and getattr(self, '_order_mode', False) and '모자랍니다' in str(e)
                and getattr(self, '_restock_n', 0) < cfg.RESTOCK_MAX)

    def _restock(self, why):
        """cube_search 로 탐색 영역의 큐브를 찾아 잡고 교체 트레이 빈칸에 넣은 뒤 트레이 맵을 지운다(다시 잼)."""
        from bed_inspect.bed_robot import BedError
        self._restock_n += 1
        text = '재고 부족: 큐브 탐색으로 트레이 보충'
        self.log(f'⚠ {text} ({self._restock_n}/{cfg.RESTOCK_MAX}) — {why}')
        self.step(cfg.STEP_CHECK, '재고 부족 → 탐색 영역에서 큐브 찾기 (cube_search)')
        self.publish(emergency={'kind': 'restock', 'text': text, 'what': why})
        # 넣을 트레이 칸: 예외 전에 잰 트레이 맵(침대 측정 뒤 6칸 다 잰 것)의 빈칸. 실을 풀기 전에 먼저 확인 (빈칸 없으면 풀지 않고 멈춤) (11-22 형님: 트레이를 다시 확인하지 말고)
        cell = None
        if self.map.get('tray'):
            empt = [j for j, c in enumerate(self.map['tray']) if c is not None and c.get('empty')]
            if not empt:
                raise BedError('재고 부족 → 트레이 맵에 빈칸이 없어 찾은 블록을 넣을 곳이 없습니다.')
            cell = empt[0] + 1
            self.log(f'트레이 맵의 빈칸 {cell}번에 넣습니다 (트레이 다시 확인 안 함)')
        if cfg.STRING_RESTOCK and not self._string_open:
            # 11-22 형님: 블록이 실 사이에 묶여 적재돼 있다고 보고, 먼저 실 고정을 풀어(unfix, 중간 나사까지 다시 꽂아 둠)
            # 블록을 자유롭게 한 뒤 위치추정(cube_search)으로 찾아 트레이에 넣는다. 공정이 끝나면 다시 fix.
            self._string_op('unfix')
        try:
            from pjt_test1 import cube_search as cs
        except ImportError as e:
            raise BedError(f'재고가 모자라는데 cube_search 를 불러오지 못했습니다 ({e}). pjt_test1 을 빌드하세요.')
        lines = []

        def log(m):
            lines.append(m)
            self.log('[큐브 탐색] ' + m)
        cs.input = lambda prompt='': 'n'            # 탐지 경고가 있으면 묻지 않고 잡지 않는다 (사람이 확인)
        self._home_raw()
        try:
            ccfg = cs.load_cfg()
            io = cs.RobotIO(self.robot.node, ccfg, log)
            io.prepare()
            live = None
            if hasattr(cs, 'LivePub'):
                try:
                    if getattr(self, '_live_pub', None) is None:
                        self._live_pub = cs.LivePub(self.robot.node)     # /cube_search/live → HMI 실시간 탐지 창
                    live = self._live_pub
                except Exception:
                    live = None
            cs.Job(io, ccfg, log, live=live).pick_place(cell=cell)
        except cs.SearchError as e:
            raise BedError(f'재고 부족 → 큐브 탐색 실패: {e}')
        finally:
            try:
                self._home_raw()
            except Exception:
                pass
        if not any(m.startswith('완료') for m in lines):
            raise BedError('재고 부족 → 큐브를 찾았지만 경고가 있어 잡지 않았습니다. 로그를 확인하세요.')
        if cell is not None:
            self.map['tray'][cell - 1] = None        # 넣은 칸만 다시 잰다 (강성 모름), 나머지는 맵 그대로
        else:
            self.map['tray'] = None                 # 맵이 없었으면 트레이 전체를 다시 잰다
        self.publish(emergency=None)
        self.log('트레이 보충 완료 → 트레이를 다시 재고 계획부터 이어서 합니다.')

    def _string_op(self, action):
        """실 고정해제(unfix) / 고정(fix) — string_v1 최종본(11-22)을 같은 로봇으로 실행.
        나사 돌리기는 OnRobot 서비스 모드(40N) 그리퍼라서 그 동안만 그리퍼 모드를 바꾼다."""
        from bed_inspect.bed_robot import BedError
        from bed_inspect import string_v1_config as ssc
        from bed_inspect import string_v1_plan as splan
        from bed_inspect.string_v1_demo import StringDemo
        name = {'unfix': '실 고정해제 (블록 풀기)', 'fix': '실 다시 고정 (마무리)'}[action]
        # 11-22 형님: 실 풀기/고정은 탐지를 위한 단계일 뿐 예외 상황 자체가 아님 → 알람(emergency)은 재고 부족 한 번만
        self.log(f'{name} 시작 (재고 부족 처리 단계)')
        self.step(cfg.STEP_CHECK if action == 'unfix' else cfg.STEP_DONE, name)
        old_mode = cfg.GRIPPER_MODE
        for k, w in getattr(ssc, 'GRIP_WIDTHS', {'screw_open': 25, 'screw_grip': 10}).items():
            cfg.GRIP_WIDTH.setdefault(k, w)
        try:
            if old_mode == 'io' and ssc.GRIPPER_MODE == 'service':
                # 블록 잡기에서 켠 WebLogic DO(1·2)가 남아 있으면 OnRobot 서비스 명령과 부딪힐 수 있음 → 먼저 끈다
                for idx in (cfg.DO_CLOSE, cfg.DO_OPEN):
                    self.robot.dr.set_digital_output(idx, 0)
                self.robot.sleep(0.5)
                self.log('그리퍼: WebLogic DO 끄고 OnRobot 서비스 모드(나사용 40N)로')
            cfg.GRIPPER_MODE = ssc.GRIPPER_MODE
            if cfg.GRIPPER_MODE == 'service' and self.robot._grip_cli is None:
                from onrobot_rg_msgs.srv import SetCommand
                self.robot._grip_cli = self.robot.node.create_client(SetCommand, '/onrobot/sendCommand')
                if not self.robot._grip_cli.wait_for_service(timeout_sec=5.0):
                    raise BedError('실 고정/해제: 그리퍼 서비스(/onrobot/sendCommand)가 없습니다.')
            demo = StringDemo(self.robot, self.robot.dr, self.robot.node, self.log)
            demo.virtual = cfg.VIRTUAL
            self._home_raw()
            try:
                if action == 'unfix':
                    demo.unfix(splan.unfix_points(ssc.PLACE_XYZ))
                    self._string_open = True
                else:
                    demo.fix(splan.fix_points(ssc.PICK_XYZ))
                    self._string_open = False
            except splan.StringPlanError as e:
                raise BedError(f'{name}: 설정 오류 {e}')
            self._home_raw()
        finally:
            if cfg.GRIPPER_MODE != old_mode:
                self.log(f'그리퍼: 다시 {old_mode} 모드로 (블록 잡기)')
            cfg.GRIPPER_MODE = old_mode
        if old_mode == 'io':
            # OnRobot 서비스(Modbus)로 움직인 뒤 WebLogic(DO) 쪽을 알려진 상태(열림)로 맞춰 둔다
            try:
                self.robot.grip('block_open')
            except Exception:
                pass
        self.log(f'{name} 완료')

    def _tray_z_ref(self, tray):
        """트레이 '보통 큐브 윗면 높이' 기준. 트레이에 잰 큐브가 3개 이상이면 None(그 중앙값을 씀).
        적으면(한두 개) 그 큐브 자신이 기준이 되어 높은 큐브를 못 알아봄 → 침대 맵의 보통 큐브 윗면 중앙값을
        교시 잡는 높이 차이(침대 _down - 트레이 _down)만큼 옮겨 쓴다 (11-22 실물: 트레이 6번 높은 부드러움을 보통으로 봄)."""
        if self.tall_dz is None:
            return None
        n = sum(1 for b in tray if b is not None and b.get('k') is not None and b.get('z_surf') is not None)
        if n >= 3:
            return None
        bed = self.map.get('bed') or []
        zs = [c['z'] for c in bed if c and not c.get('empty') and not c.get('tall') and c.get('z') is not None]
        try:
            bd = [self.points[nm]['down'][2] for nm in cfg.P_BED if self.points.get(nm, {}).get('down')]
            rd = [self.points[nm]['down'][2] for nm in cfg.P_RACK if self.points.get(nm, {}).get('down')]
        except (KeyError, IndexError, TypeError):
            bd, rd = [], []
        if not zs or not bd or not rd:
            return None
        ref = statistics.median(zs) - statistics.mean(bd) + statistics.mean(rd)
        self.log(f'트레이 큐브가 {n}개뿐 → 높이 기준을 침대에서 옮겨 씀: 보통 큐브 윗면 z {ref:.1f} (높다 판단 +{self.tall_dz:.1f}mm)')
        return ref

    def _class_ref(self):
        """전에 두 종류로 나눴을 때의 경계·중심 (DATA_DIR/class_ref.json). 없으면 설정값 CLASS_THR_FALLBACK."""
        import json
        path = os.path.join(cfg.DATA_DIR, 'class_ref.json')
        try:
            with open(path) as f:
                d = json.load(f)
            if d.get('thr'):
                return d
        except (OSError, ValueError):
            pass
        if cfg.CLASS_THR_FALLBACK:
            t = cfg.CLASS_THR_FALLBACK
            return {'thr': t, 'soft': t * cfg.REPAIR_RATIO, 'hard': t / cfg.REPAIR_RATIO, 'src': 'config'}
        return None

    def _save_class_ref(self, thr, centers):
        import json
        if not (centers.get('soft') and centers.get('hard')):
            return                                   # 두 종류가 다 있을 때만 저장
        try:
            os.makedirs(cfg.DATA_DIR, exist_ok=True)
            with open(os.path.join(cfg.DATA_DIR, 'class_ref.json'), 'w') as f:
                json.dump({'thr': thr, 'soft': centers['soft'], 'hard': centers['hard'], 'time': time.time()}, f)
        except OSError:
            pass

    def _wait_answer(self):
        while True:
            self.robot.check_stop()
            try:
                return self.io.answer.get(timeout=0.1)
            except queue.Empty:
                pass


def main(args=None):
    rclpy.init(args=args)
    teach_file = bed_store.teach_path(remove_ros_args(sys.argv)[1:])
    sim = bed_store.use_sim_data_if_sim(teach_file)
    node = rclpy.create_node('bed_process', namespace=cfg.ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        import DSR_ROBOT2 as dr
    except ImportError as e:
        node.get_logger().error(f'DSR_ROBOT2 를 불러오지 못했습니다: {e}')
        rclpy.shutdown()
        return
    from bed_inspect.bed_robot import BedError

    # 정지 서비스 이름: DSR_ROBOT2 의 이름은 'dsr_controller2/motion/move_joint' 처럼 상대 이름이라,
    # 네임스페이스가 없는 bed_process_io 노드에서 쓰면 /dsr01 이 빠진다 → 앞에 /dsr01/ 을 붙인다.
    stop_name = dr._ros2_movej.srv_name.replace('move_joint', 'move_stop')
    if not stop_name.startswith('/'):
        stop_name = f'/{cfg.ROBOT_ID}/{stop_name}'
    io = BedIO(stop_name)
    proc = BedProcess(io, teach_file)
    try:
        try:
            proc.setup(dr, node)
        except BedError as e:
            io.log(f'시작 실패: {e}')
            proc.publish(state='error', message=f'시작 실패: {e} (고친 뒤 bed_process 를 다시 실행하세요)')
            while rclpy.ok():               # 화면에 오류가 보이도록 떠 있는다
                time.sleep(0.5)
            return
        note = proc._uncertain_note()
        proc.publish(state='idle', message='준비 완료. 모드를 고르고 시작을 누르세요.' + note)
        io.log('준비 완료' + (' (가짜 로봇 모드: 기록은 %s)' % cfg.DATA_DIR if sim else ''))
        while rclpy.ok():
            try:
                cmd = io.cmds.get(timeout=0.2)
            except queue.Empty:
                continue
            proc.run(cmd)
    except KeyboardInterrupt:
        pass
    finally:
        io.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
