# 공정 노드: 화면(bed_hmi)에서 명령을 받아 12단계 파이프라인을 실행한다.
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
#   {"cmd": "start", "mode": "measure"}                        측정만 (1~4단계)
#   {"cmd": "start", "mode": "replace"}                        불량 블록 교체 모드 (1~12단계)
#   {"cmd": "start", "mode": "tilt", "axis": "col", "angle": 3} 기울기 맞춤 모드. axis: col(1→3번 방향) / row(1→4번 방향)
#       ※ start 에 "confirm": true 를 넣으면 교체 계획을 보여주고 "confirm"/"cancel" 을 기다린다
#   {"cmd": "calibrate"}                                       지금 블록들을 측정해서 높이별 기준 강성으로 저장
#   {"cmd": "home"}                                            홈으로 이동
#   {"cmd": "set_inventory", "bed": [...], "rack": [...], "cover_open": false, "tight": false}
#   {"cmd": "stop"}                                            정지 (실행 중에도 받는다)
#
# 스레드 구조
#   - 메인 스레드: DSR_ROBOT2 로 로봇을 움직인다. (DSR_ROBOT2 는 한 스레드에서만 써야 한다)
#   - 통신 스레드: /bed/cmd 를 받는다. 그래서 로봇이 움직이는 중에도 "정지"가 바로 들어온다.

import contextlib
import json
import queue
import sys
import threading
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


def _deg(a):
    """각도를 글자로: 0 은 부호 없이."""
    return '0.0도' if abs(a) < 0.05 else f'{a:+.1f}도'


def _tilt_text(t):
    return f'길이 방향 {_deg(t["col_deg"])}, 폭 방향 {_deg(t["row_deg"])}'


class BedIO:
    """화면과 주고받는 통로. 별도 노드 + 별도 스레드로 돌아서, 로봇이 움직이는 동안에도 명령을 받는다."""

    def __init__(self, stop_srv_name):
        from dsr_msgs2.srv import MoveStop
        self._MoveStop = MoveStop
        self.node = rclpy.create_node('bed_process_io')
        self.pub_status = self.node.create_publisher(String, '/bed/status', STATUS_QOS)
        self.pub_log = self.node.create_publisher(String, '/bed/log', 50)
        self.node.create_subscription(String, '/bed/cmd', self._on_cmd, 10)
        self.stop_cli = self.node.create_client(MoveStop, stop_srv_name)
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
        try:
            d = json.loads(msg.data)
            c = d['cmd']
        except (ValueError, KeyError, TypeError):
            self.log(f'알 수 없는 명령: {msg.data!r}')
            return
        if c == 'stop':
            self.stop_event.set()
            if self.busy and self.stop_cli.service_is_ready():
                req = self._MoveStop.Request()
                req.stop_mode = 1           # DR_QSTOP: 빠른 정지(서보는 켜진 채)
                self.stop_cli.call_async(req)
            self.log('정지 요청을 받았습니다.')
        elif c in ('confirm', 'cancel'):
            self.answer.put(c)
        elif self.busy:
            self.log(f'실행 중이라 "{c}" 명령은 무시합니다. (정지만 받습니다)')
        else:
            self.cmds.put(d)

    def log(self, text):
        self.node.get_logger().info(text)
        self.pub_log.publish(String(data=time.strftime('%H:%M:%S ') + text))

    def send_status(self, status):
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
        self.xy = []
        self.inv = bed_store.load_inventory()
        self.world = bed_store._load_json('world.json', {'cover_open': False, 'tight': False,
                                                         'uncertain': False})
        self.calib = bed_store.load_calib()
        self.status = {
            'state': 'starting', 'mode': None, 'step': 0, 'step_name': '', 'detail': '',
            'message': '공정 노드를 시작하는 중입니다.', 'active_slot': None,
            'first': None, 'second': None, 'plan': None,
            'layout': {'rows': cfg.BED_ROWS, 'cols': cfg.BED_COLS, 'heights': cfg.BLOCK_HEIGHTS,
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
        self.status['achievable'] = self._achievable()
        self.status['stamp'] = time.time()
        self.io.send_status(self.status)

    def step(self, n, detail=''):
        self.log(f'[{n}단계] {cfg.STEP_NAMES[n]}' + (f' - {detail}' if detail else ''))
        self.publish(step=n, step_name=cfg.STEP_NAMES[n], detail=detail)

    def _achievable(self):
        if self.axes is None:
            return None
        cands = bed_math.tilt_candidates(
            self.inv['bed'], self.inv['rack'], cfg.BLOCK_HEIGHTS, cfg.BED_ROWS, cfg.BED_COLS,
            self.axes['pitch_col'], self.axes['pitch_row'], cfg.TILT_ALLOW_STEP, cfg.TILT_PLANE_TOL)
        return bed_math.achievable_angles(cands)

    def _save_world(self, **changes):
        self.world.update(changes)
        bed_store._save_json('world.json', self.world)

    @contextlib.contextmanager
    def _critical(self):
        """도중에 멈추면 기록과 실제가 달라질 수 있는 동작(롤러, 커버, 블록 옮기기)을 감싼다.

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
            self.points = bed_store.load_teach(self.teach_file)
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
            if mode not in ('measure', 'replace', 'tilt', 'calibrate', 'home'):
                self.log(f'알 수 없는 모드: {mode}')
                return
            if self.world.get('uncertain'):
                # 홈 이동도 막는다: 그리퍼가 커버나 롤러, 블록을 물고 있을 수 있어서 그대로 움직이면 위험하다
                raise BedError('지난번에 롤러/커버/블록 동작 중에 멈췄습니다. 그리퍼가 물고 있는 것을 손으로 빼고, '
                               '실제 블록 위치, 커버, 천 상태를 확인해 "재고 수정"에서 저장한 뒤 다시 시작하세요.')
            self.publish(state='running', mode=mode, message='', detail='', active_slot=None,
                         **({} if mode == 'home' else {'first': None, 'second': None, 'plan': None}))
            tcp = self.robot.prepare()
            if mode == 'home':
                self.tasks.home()
                self.publish(state='idle', mode=None, step=0, step_name='', message='홈 위치입니다.')
            elif mode == 'calibrate':
                self._calibrate()
            else:
                self._pipeline(mode, cmd, tcp)
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
        return ' ※ 롤러/커버/블록 동작 중이었습니다. 실제 상태를 확인하고 "재고 수정"을 저장하세요.' \
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
        self._save_world(cover_open=bool(cmd.get('cover_open', False)),
                         tight=bool(cmd.get('tight', False)), uncertain=False)
        self.log('재고를 저장했습니다.')
        self.publish(state='idle', message='재고를 저장했습니다.')

    # ── 측정 (3, 11단계) ──────────────────────────────────────────────
    def _measure(self, key, step_no):
        """6칸을 차례로 누르고, 판정과 기울기를 붙여 status[key] 에 넣는다."""
        blocks = []
        for i in range(len(cfg.P_BED)):
            self.step(step_no, f'{i + 1}번 블록 ({i + 1}/{len(cfg.P_BED)})')
            self.publish(active_slot=i + 1)
            res = self.tasks.press(i, self.inv['bed'][i])
            if res['error'] == 'bad_fit':                       # 한 번 더 시도
                self.log(f'{i + 1}번: {res["note"]} → 다시 측정')
                res = self.tasks.press(i, self.inv['bed'][i])
            k = '-' if res['k'] is None else f'{res["k"]:.2f} N/mm'
            self.log(f'{i + 1}번: 강성 {k}' + (f' ({res["note"]})' if res['note'] else ''))
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

    def _tighten(self, step_no):
        self.step(step_no)
        if self.world.get('tight'):
            self.log('천이 이미 팽팽한 상태로 기록되어 있어 건너뜁니다.')
            return
        with self._critical():
            self.tasks.roller(+1)
        self._save_world(tight=True)

    def _recover(self):
        """지난번에 커버가 열린 채 끝났으면 먼저 닫는다."""
        if self.world.get('cover_open'):
            self.log('커버가 열린 상태로 기록되어 있습니다 → 먼저 닫습니다.')
            if self.world.get('tight'):
                with self._critical():
                    self.tasks.roller(-1)
                self._save_world(tight=False)
            with self._critical():
                self.tasks.cover_close()
            self._save_world(cover_open=False)

    # ── 기준값 저장 ───────────────────────────────────────────────────
    def _calibrate(self):
        from bed_inspect.bed_robot import BedError
        self.step(1)
        self.tasks.home()
        self._recover()
        self._tighten(2)
        blocks, _ = self._measure('first', 3)
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

    # ── 12단계 파이프라인 ─────────────────────────────────────────────
    def _pipeline(self, mode, cmd, tcp):
        from bed_inspect.bed_robot import BedError
        if mode in ('replace', 'tilt'):
            # 커버 원호 이동은 posx 를 쓰므로, 교시할 때와 TCP 가 같아야 한다
            bad = [n for n in (cfg.P_COVER_GRIP, cfg.P_COVER_VIA, cfg.P_COVER_OPEN)
                   if (self.points[n].get('tcp') or '') != tcp]
            if bad:
                raise BedError(f'현재 TCP("{tcp}")가 교시할 때와 다릅니다: {bad}. TCP 를 맞추거나 다시 교시하세요.')

        self.step(1)
        self.tasks.home()
        self._recover()
        self._tighten(2)
        blocks, tilt = self._measure('first', 3)

        # 4단계: 확인 + 계획
        self.step(4)
        fails = [b['slot'] for b in blocks if b['grade'] == 'fail']
        nodata = [b['slot'] for b in blocks if b['grade'] == 'nodata']
        note = (f' 측정 실패 {nodata}번은 다시 확인이 필요합니다.' if nodata else '')
        if mode == 'measure':
            self.tasks.home()
            msg = f'불량 {fails}번.' if fails else '불량 없음.'
            self.publish(state='warning' if (fails or nodata) else 'done', message=msg + note)
            return

        if mode == 'replace':
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
        else:
            axis = cmd.get('axis', 'col')
            angle = float(cmd.get('angle', 0.0))
            tc, tr = (angle, 0.0) if axis == 'col' else (0.0, angle)
            p = bed_math.plan_tilt(tc, tr, self.inv['bed'], self.inv['rack'], cfg.BLOCK_HEIGHTS,
                                   cfg.BED_ROWS, cfg.BED_COLS, self.axes['pitch_col'], self.axes['pitch_row'],
                                   cfg.TILT_ALLOW_STEP, cfg.TILT_PLANE_TOL)
            moves = p['moves']
            plan = {'type': 'tilt', 'axis': axis, 'request_deg': angle, 'heights': p['heights'],
                    'col_deg': p['col_deg'], 'row_deg': p['row_deg'], 'rms': p['rms'], 'faulty': fails}
            self.log(f'목표 {_deg(angle)} → 가장 가까운 배치: {_tilt_text(p)}, '
                     '높이 ' + ' / '.join(f'{h:g}' for h in p['heights']))
            if not moves:
                self.tasks.home()
                ok = self._tilt_ok(tilt, p)
                self.publish(state='done' if ok else 'warning', plan=dict(plan, moves=[]),
                             message='이미 목표에 가장 가까운 배치입니다. 교체할 블록이 없습니다.'
                                     + ('' if ok else ' (측정한 기울기가 예상과 다릅니다. 재고 기록을 확인하세요)'))
                return
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

        # 5~6단계: 천 풀고 커버 열기
        self.step(5)
        with self._critical():
            self.tasks.roller(-1)
        self._save_world(tight=False)
        self.step(6)
        with self._critical():
            self.tasks.cover_open()
        self._save_world(cover_open=True)

        # 7~8단계: 블록 옮기기 (하나 옮길 때마다 재고 파일을 갱신)
        for n, mv in enumerate(moves):
            self.step(8 if mv['dst'][0] == 'bed' else 7, bed_math.move_text(mv))
            with self._critical():
                self.tasks.move_block(mv)
                bed_math.apply_move(self.inv['bed'], self.inv['rack'], mv)
                bed_store.save_inventory(self.inv)
            plan['done'] = n + 1
            self.publish(plan=plan)

        # 9~10단계: 커버 닫고 천 팽팽하게
        self.step(9)
        with self._critical():
            self.tasks.cover_close()
        self._save_world(cover_open=False)
        self._tighten(10)

        # 11~12단계: 재측정과 확인
        blocks2, tilt2 = self._measure('second', 11)
        self.step(12)
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
        else:
            ok = self._tilt_ok(tilt2, plan)
            got = '측정 실패' if tilt2 is None else _tilt_text(tilt2)
            want = _tilt_text(plan)
            if ok and not fails2 and not nodata2:
                self.publish(state='done', message=f'기울기를 맞췄습니다. 목표 배치: {want} / 측정: {got}')
            else:
                msg = f'목표 배치: {want} / 측정: {got}.'
                if not ok:
                    msg += f' 차이가 {cfg.TILT_OK_DEG:g}도를 넘습니다.'
                if fails2:
                    msg += f' 불량 {fails2}번.'
                self.publish(state='warning', message=msg + note2)

    @staticmethod
    def _tilt_ok(tilt, plan):
        return (tilt is not None
                and abs(tilt['col_deg'] - plan['col_deg']) <= cfg.TILT_OK_DEG
                and abs(tilt['row_deg'] - plan['row_deg']) <= cfg.TILT_OK_DEG)

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

    io = BedIO(dr._ros2_movej.srv_name.replace('move_joint', 'move_stop'))
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
