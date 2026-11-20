# 영역 안의 실리콘 큐브를 '위에서 누르기'만으로 찾아서, 잡고, 교체 트레이의 빈칸에 넣는다.
#
#   ros2 run pjt_test1 cube_search                      # 실제 로봇, 대화형
#   ros2 run pjt_test1 cube_search --sim                # 가짜 로봇 (ROS 없이 알고리즘 확인). 큐브 임의 위치
#   ros2 run pjt_test1 cube_search --sim 520,30,25      # 가짜 로봇, 큐브 중심 (520, 30), 긴 축 25°
#   ros2 run pjt_test1 cube_search --sim --place        # 가짜 로봇으로 탐지 → 잡기 → 트레이까지 순서 확인
#
# 설정 파일: ~/ws_cobot_pjt/cube_search/config.json (처음 실행하면 기본값으로 만든다)
#   반드시 채울 것: area (큐브가 놓일 수 있는 영역, 베이스 x/y [mm]), z_floor (영역 바닥에 손가락 끝이 닿는 TCP z)
#   z_floor 는 메뉴 f 로 직접 잴 수 있다.
# 기록: ~/ws_cobot_pjt/cube_search/logs/ (결과 json + 위에서 본 그림 png)
#
# 메뉴
#   s   탐지만 (중심, 각도, 높이 출력)
#   g   탐지 → 잡기 → 트레이 빈칸에 넣기
#   t   트레이 빈칸 확인 (칸마다 위에서 터치)
#   f   영역 바닥 높이 재기 (z_floor 저장)
#   tf  트레이 바닥 높이 재기 (빈 칸에서, tray_floor_z 저장)
#   c   설정 보기 | q 종료
#
# 영역 조건 (중요)
#   - 영역 안에는 큐브 1개만, 큐브보다 높은 물체(벽, 틀)가 없어야 한다. 영역 경계는 벽에서 그리퍼 반폭 이상 떨어뜨린다.
#   - 큐브 둘레에 그리퍼를 벌릴 공간(열기 폭 + 손가락 두께)이 있어야 한다.
# 그리퍼 (WebLogic, DO1/DO2): (1,0) 닫기 0 mm 5 N | (0,1) 열기 66 mm 10 N | (1,1) 잡기 60 mm 20 N
#   탐지 중에는 닫기로 손가락 끝을 모아 누르고, 잡기는 (1,1).
#   grasp_side 'long' = repair_test/bring_test 와 같이 넓은 쪽(64 mm 변)을 잡는다 (손가락이 큐브 긴 축 방향으로 벌어짐).
#   열기 폭(grip_open_width)이 큐브 긴 변 + 여유보다 좁으면 손가락이 큐브 윗면에 얹혀 잡으러 내려가다 멈출 수 있다.

import copy
import json
import math
import os
import random
import sys
import time

from pjt_test1.cube_search_core import CubeSearcher, SearchError, grasp_c, rect_sdf

import numpy as np

BASE_DIR = os.path.expanduser('~/ws_cobot_pjt/cube_search')
CONFIG_PATH = os.path.join(BASE_DIR, 'config.json')
LOG_DIR = os.path.join(BASE_DIR, 'logs')

DEFAULT_CFG = {
    '_설명': 'area/z_floor 는 반드시 실물에 맞게. 단위 mm, N, mm/s. 여기 없는 항목은 cube_search.py 의 DEFAULT_CFG 값 (같은 이름으로 넣으면 덮어씀)',
    'area': {'x_min': None, 'x_max': None, 'y_min': None, 'y_max': None},
    'z_floor': None,
    'cube': {'long': 64.0, 'short': 41.0, 'h_min': 10.0, 'h_max': 30.0},
    'tcp': 'GripperDA_v1',
    'probe_ori': [168.25, 179.97, -11.77],
    'grid_step': 25.0,
    'coarse_min_rise': 5.0,
    'hover': 8.0,
    'press_depth': 2.0,            # 큐브 윗면보다 이만큼 더 눌러 '있음' 확인 [mm] (실측 강성 11~14 N/mm → 약 24 N)
    'vel_coarse': 20.0,            # 거친 격자 하강 [mm/s] (힘 갱신 0.1 s → 최대 2 mm 더 눌림 ≈ 24 N)
    'vel_press': 5.0,
    'vel_move': 100.0,
    'vel_slow': 20.0,
    'f_static_min': 4.0,           # 멈춘 뒤 접촉 판단 [N] (실측: 움직인 뒤 멈추면 힘 추정이 1~2 N 남음)
    'f_dyn_min': 6.0,              # 내려가는 중 접촉 판단 [N] (실측: 공중에서 움직이기만 해도 약 3 N)
    'f_limit': 45.0,               # 이 이상이면 단단한 물체 → 중단 [N] (힘 갱신 0.1 s 라 큐브도 순간 20~30 N 가능)
    'ray_count': 8,
    'ray_tol': 4.0,
    'edge_tol': 0.5,
    'edge_window': 8.0,
    'edge_offset': 16.0,
    'grip_open_width': 66.0,       # WebLogic 열기 (DO 0,1) 폭 [mm]. 잡기 전 큐브가 손가락 사이에 들어가는지 확인용
    'floor_search': 60.0,          # 메뉴 f: 시작 높이에서 이만큼까지 내려가며 바닥을 찾는다 [mm]
    'tare_settle': 0.3,            # 터치 위치에 온 뒤 힘 영점 잡기 전 대기 [s]
    'tare_samples': 5,             # 영점 읽기 횟수 (force_period 간격)
    'force_period': 0.1,           # 컨트롤러 힘 값 갱신 주기 [s] (실측 10 Hz)
    'floor_vel_fine': 1.0,
    'floor_f_min': 8.0,            # 메뉴 f/tf: 단단한 바닥 접촉 판단 [N] (움직일 때 힘 추정이 2~3 N 흔들리므로 크게)         # 메뉴 f: 바닥에 닿으면 조금 올라가 이 속도로 다시 재서 정확히 [mm/s]
    'edge_margin': 25.0,           # 가장자리 탐색이 영역 밖으로 나가도 되는 거리 [mm] (> 손가락 끝 반폭). 경계에 벽이 있으면 0
    'finger_ex': 10.0,
    'finger_ey': 6.0,
    'fit_rms_max': 4.0,
    'refine_repeat_deg': 4.0,
    'refine_max': 3,
    'parallel_warn_deg': 3.0,
    'finger_max': 20.0,
    'size_tol_mm': 3.0,
    'grasp_side': 'long',
    'grasp_depth': 15.0,
    'floor_clear': 4.0,
    'grasp_f_limit': 12.0,
    'place_clear': 1.0,
    'settle': 0.3,
    'z_travel': 170.0,
    'tray_file': '~/ws_cobot_pjt/teach_points/exchange_fin.json',
    'tray_floor_z': None,
    'tray_check': True,
    'tray_check_from': 45.0,
    'tray_order': [1, 2, 3, 4, 5, 6],
}

# 시뮬레이터 기본 영역 (설정 파일의 area 가 비어 있을 때)
SIM_AREA = {'x_min': 420.0, 'x_max': 620.0, 'y_min': -80.0, 'y_max': 80.0}
SIM_FLOOR = 20.0


# 처음 만드는 설정 파일에는 실물마다 다른 값만 넣는다. 나머지는 DEFAULT_CFG 값을 쓰고, 바꾸고 싶으면 같은 이름으로 추가한다.
CFG_TEMPLATE_KEYS = ('_설명', 'area', 'z_floor', 'tray_floor_z', 'cube', 'grasp_side', 'grasp_depth', 'z_travel',
                     'tray_file')


def packaged_cfg_path():
    """저장소에 들어 있는 기본 설정(실측값) 경로. 설치본(share) → 소스 폴더 순서로 찾는다."""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('pjt_test1'), 'config', 'cube_search_config.json')
        if os.path.exists(p):
            return p
    except Exception:
        pass
    p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config', 'cube_search_config.json')
    return p if os.path.exists(p) else None


def load_cfg(path=CONFIG_PATH):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if not os.path.exists(path):
        src = packaged_cfg_path()
        if src:   # 저장소의 실측 설정(영역, 바닥 높이 등)으로 시작 → 같은 로봇·배치면 바로 쓸 수 있다
            with open(src) as f:
                user = json.load(f)
            with open(path, 'w') as f:
                json.dump(user, f, indent=1, ensure_ascii=False)
            print(f'설정 파일을 만들었습니다: {path}  (저장소 기본값 {src} 복사. 배치가 다르면 area 를 고치고 f/tf 로 다시 재세요)')
        else:
            with open(path, 'w') as f:
                json.dump({k: DEFAULT_CFG[k] for k in CFG_TEMPLATE_KEYS}, f, indent=1, ensure_ascii=False)
            print(f'설정 파일을 만들었습니다: {path}  (area, z_floor 를 채우세요)')
    with open(path) as f:
        user = json.load(f)
    cfg = copy.deepcopy(DEFAULT_CFG)
    for k, v in user.items():
        if isinstance(v, dict) and isinstance(cfg.get(k), dict):
            cfg[k].update(v)
        else:
            cfg[k] = v
    return cfg


def save_cfg_value(key, value, path=CONFIG_PATH):
    with open(path) as f:
        user = json.load(f)
    user[key] = value
    with open(path, 'w') as f:
        json.dump(user, f, indent=1, ensure_ascii=False)


def cfg_problems(cfg):
    a = cfg['area']
    out = []
    if any(a[k] is None for k in ('x_min', 'x_max', 'y_min', 'y_max')):
        out.append('area (x_min, x_max, y_min, y_max) 가 비어 있습니다')
    elif a['x_max'] - a['x_min'] < cfg['cube']['short'] or a['y_max'] - a['y_min'] < cfg['cube']['short']:
        out.append('area 가 큐브보다 작습니다')
    if cfg['z_floor'] is None:
        out.append('z_floor 가 비어 있습니다 (메뉴 f 로 재기)')
    elif cfg['z_floor'] + cfg['cube']['h_max'] + cfg['hover'] > cfg['z_travel']:
        out.append(f'z_floor {cfg["z_floor"]:.1f} 이 너무 높습니다 (바닥 + 큐브 최대 높이가 이동 높이 z_travel '
                   f'{cfg["z_travel"]:g} 보다 위). 바닥을 잘못 쟀을 가능성 → 메뉴 f 로 다시 재세요')
    if cfg['grid_step'] > cfg['cube']['short'] / math.sqrt(2):
        out.append(f'grid_step 이 {cfg["cube"]["short"] / math.sqrt(2):.1f} mm 보다 크면 큐브를 놓칠 수 있습니다')
    return out


def load_tray(cfg):
    path = os.path.expanduser(cfg['tray_file'])
    with open(path) as f:
        pts = {p['name']: p for p in json.load(f)}
    tray = {}
    for n in range(1, 7):
        top, down = pts.get(f'tray_b{n}'), pts.get(f'tray_b{n}_down')
        if top and down:
            tray[n] = {'top': list(top['posx']), 'down': list(down['posx']), 'tcp': top.get('tcp', '')}
    return tray


# ══════════════════════════════════════════════════════════════════════
# 실제 로봇
# ══════════════════════════════════════════════════════════════════════
class RobotIO:
    STATE_STANDBY = 1

    def __init__(self, node, cfg, log):
        import DSR_ROBOT2
        from DR_common2 import posx
        from pjt_test1.gripper import Gripper
        self.dr, self.posx, self.node, self.cfg, self.log = DSR_ROBOT2, posx, node, cfg, log
        self._stop_cli = None
        for n in ('_ros2_get_robot_state', '_ros2_get_robot_mode', '_ros2_set_robot_mode', '_ros2_get_last_alarm',
                  '_ros2_get_current_posx', '_ros2_get_current_tcp', '_ros2_get_tool_force', '_ros2_movel',
                  '_ros2_amovel', '_ros2_check_motion', '_ros2_mwait'):
            cli = getattr(DSR_ROBOT2, n, None)
            if cli is None:
                continue
            while not cli.wait_for_service(timeout_sec=1.0):
                log(f'서비스 기다리는 중: {cli.srv_name}')
        self.grip = Gripper(node, use_do=True)

    # 준비 ---------------------------------------------------------------
    def prepare(self):
        self.require_standby('시작')
        if self.dr.get_robot_mode() != self.dr.ROBOT_MODE_AUTONOMOUS:
            self.log('MANUAL → AUTONOMOUS 모드로 전환')
            if self.dr.set_robot_mode(self.dr.ROBOT_MODE_AUTONOMOUS) != 0:
                raise SearchError('AUTONOMOUS 모드 전환 실패')
        tcp = self.dr.get_tcp()
        if tcp != self.cfg['tcp']:
            raise SearchError(f'현재 TCP({tcp or "없음"}) 가 설정의 tcp({self.cfg["tcp"]}) 와 다릅니다. TCP 를 맞추세요.')

    def require_standby(self, what):
        s = self.dr.get_robot_state()
        if s != self.STATE_STANDBY:
            text = f'{what}: 로봇 상태 {s} (STANDBY 아님)'
            a = self.dr.get_last_alarm()
            if a != -1:
                text += f', 알람 level={a.level} group={a.group} index={a.index}'
            raise SearchError(text + ' → 원인 확인 후 servo_on 으로 복구하세요.')

    # 이동 ---------------------------------------------------------------
    def pose(self):
        res = self.dr.get_current_posx(ref=self.dr.DR_BASE)
        if not isinstance(res, tuple) or res[0] is None:
            raise SearchError('현재 위치를 읽지 못했습니다.')
        return [float(v) for v in res[0][:6]]

    def movel(self, p6, vel, acc=None, what='이동'):
        self.require_standby(what)
        ret = self.dr.movel(self.posx([float(v) for v in p6]), vel=vel, acc=acc or 2 * vel, ref=self.dr.DR_BASE)
        if ret != 0:
            raise SearchError(f'{what} 실패 → {[round(v, 2) for v in p6]}')

    def sleep(self, s):
        time.sleep(s)

    # 힘 -----------------------------------------------------------------
    def fz(self, n):
        """베이스 Z 힘 n 개. 컨트롤러 힘 값은 약 0.1 s(10 Hz)마다 갱신되므로 여러 개면 0.1 s 간격으로 읽는다."""
        vals = []
        for i in range(n):
            if i:
                time.sleep(self.cfg['force_period'])
            f = self.dr.get_tool_force(ref=self.dr.DR_BASE)
            if f != -1 and f is not None:
                vals.append(float(f[2]))
        if not vals:
            raise SearchError('힘을 읽지 못했습니다.')
        return vals

    def stop_motion(self):
        if self.dr.check_motion() != self.dr.DR_STATE_IDLE:
            import rclpy
            if self._stop_cli is None:
                from dsr_msgs2.srv import MoveStop
                self._MoveStop = MoveStop
                self._stop_cli = self.node.create_client(MoveStop, self.dr._srv_name_prefix + 'motion/move_stop')
                self._stop_cli.wait_for_service(timeout_sec=2.0)
            req = self._MoveStop.Request()
            req.stop_mode = self.dr.DR_SSTOP
            fut = self._stop_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=2.0)
        self.dr.mwait()
        end = time.monotonic() + 3.0
        while self.dr.get_robot_state() != self.STATE_STANDBY and time.monotonic() < end:
            time.sleep(0.05)

    def descend(self, target6, vel, tare, thr_dyn, f_limit):
        """target6 까지 amovel 로 내려가며 힘을 본다. 반환 ('hit' | 'hard' | 'end', 멈춘 z)."""
        self.require_standby('탐색 하강')
        if self.dr.amovel(self.posx([float(v) for v in target6]), vel=vel, acc=4 * vel, ref=self.dr.DR_BASE) != 0:
            raise SearchError('탐색 하강 시작 실패')
        z_goal = float(target6[2])
        t0, cnt, reason = time.monotonic(), 0, 'end'
        trace = []
        started = False          # amovel 직후에는 아직 출발 전이라 IDLE 로 읽힐 수 있다 → 움직이기 시작한 뒤에만 '끝' 판정
        try:
            while True:
                f = abs(self.fz(1)[0] - tare)
                if not trace or trace[-1][1] != round(f, 2):          # 값이 바뀔 때만 기록
                    trace.append((round(time.monotonic() - t0, 2), round(f, 2)))
                if f >= f_limit:
                    reason = 'hard'
                    break
                if time.monotonic() - t0 > 0.25 and f >= thr_dyn:   # 출발 가속 구간의 튐은 무시
                    cnt += 1
                    if cnt >= 2:
                        reason = 'hit'
                        break
                else:
                    cnt = 0
                idle = self.dr.check_motion() == self.dr.DR_STATE_IDLE
                if not idle:
                    started = True
                elif started or time.monotonic() - t0 > 2.0:
                    break
        finally:
            self.stop_motion()
        z = self.pose()[2]
        if os.environ.get('CUBE_DEBUG'):
            self.log(f'  [하강] {reason} z {z:.2f} (목표 {z_goal:.1f}) |힘-영점| (t,F): {trace[:12]}{" ..." if len(trace) > 12 else ""}')
        if reason == 'end' and z > z_goal + 1.0:
            a = self.dr.get_last_alarm()
            al = f', 마지막 알람 level={a.level} group={a.group} index={a.index}' if a != -1 else ''
            raise SearchError(f'하강이 목표 z {z_goal:.1f} 까지 가지 않고 z {z:.1f} 에서 멈췄습니다 '
                              f'(이동 명령이 실행되지 않았거나 중간에 멈춤{al}). 목표가 작업 범위 밖이면 생길 수 있습니다.')
        return reason, z

    def probe(self, x, y, z_from, z_to, vel, ori, f_limit=None, f_min=None):
        """f_min: 접촉 판단 최소 힘 [N] (없으면 설정값). 단단한 바닥을 잴 때는 크게 줘서 움직일 때 생기는 힘 추정 오차를 무시."""
        c = self.cfg
        f_limit = f_limit or c['f_limit']
        cur = self.pose()
        if cur[2] < z_from - 0.5:                                      # 낮으면 먼저 수직으로 올라간다
            self.movel(cur[:2] + [z_from] + cur[3:], c['vel_move'], what='상승')
        self.movel([x, y, z_from] + list(ori), c['vel_move'], what='터치 위치로')
        self.sleep(c['tare_settle'])
        vals = self.fz(c['tare_samples'])
        tare = sum(vals) / len(vals)
        sigma = (sum((v - tare) ** 2 for v in vals) / (len(vals) - 1)) ** 0.5
        thr_st = max(f_min or c['f_static_min'], 5 * sigma)
        thr_dyn = max(f_min or c['f_dyn_min'], 6 * sigma)
        out = {'z': None, 'f': 0.0, 'hard': False, 'sigma': sigma}
        try:
            for _ in range(30):                                        # 가짜 감지면 그 자리에서 다시 내려간다
                reason, z = self.descend([x, y, z_to] + list(ori), vel, tare, thr_dyn, f_limit)
                if reason == 'hard':
                    out['hard'], out['z'] = True, z
                    break
                self.sleep(0.15)
                f = abs(sum(self.fz(4)) / 4 - tare)
                out['f'] = f
                if os.environ.get('CUBE_DEBUG'):
                    self.log(f'  [정지 확인] z {z:.2f} 정지 힘 {f:.2f} N (기준 {thr_st:.2f}, 영점 {tare:.2f}, σ {sigma:.3f})')
                if f >= thr_st:                                        # 멈춘 상태에서도 힘이 남아 있으면 진짜 접촉
                    out['z'] = z
                    break
                if reason == 'end' or z <= z_to + 0.1:
                    break
        finally:
            self.movel([x, y, z_from] + list(ori), c['vel_move'], what='터치 후 상승')
        return out

    # 그리퍼 -------------------------------------------------------------
    def gripper(self, what):
        ok = {'open': self.grip.open, 'close': self.grip.close, 'grip': self.grip.grip}[what]()
        if not ok:
            raise SearchError(f'그리퍼 {what} 실패')


# ══════════════════════════════════════════════════════════════════════
# 가짜 로봇 (알고리즘 확인용): 무른 큐브(스프링) + 단단한 바닥 + 손가락 끝 크기 + 힘 노이즈
# ══════════════════════════════════════════════════════════════════════
class SimIO:
    K_CUBE = 12.0         # 큐브 강성 [N/mm] (repair_test 실측 11~14)
    FINGER = (28.0, 16.0)  # 닫힌 손가락 끝 크기 (x = 벌어지는 방향, y) [mm]. 두 손가락 + 사이 10 mm
    NOISE = 0.25          # 힘 노이즈 [N]
    MOTION_BIAS = 3.0     # 움직이는 중 힘 추정 치우침 [N] (실측)
    STATIC_BIAS = 2.0     # 멈춘 뒤 남는 치우침 최대 [N] (실측 1~2)
    F_PERIOD = 0.1        # 힘 값 갱신 주기 [s] (실측 10 Hz) → 그만큼 늦게 멈춘다

    def __init__(self, cfg, log, cube):
        self.cfg, self.log, self.cube = cfg, log, cube        # cube = (cx, cy, 각도 deg, 높이)
        self.z = cfg['z_travel']
        self.xy = (0.0, 0.0)
        self.n_probe = 0
        self.t_sim = 0.0
        self.tray_full = set()
        self.held = False
        fx, fy = self.FINGER
        gx, gy = np.meshgrid(np.linspace(-fx / 2, fx / 2, 7), np.linspace(-fy / 2, fy / 2, 11))
        self._fp = np.stack([gx.ravel(), gy.ravel()], axis=1)

    def prepare(self):
        pass

    def pose(self):
        return [self.xy[0], self.xy[1], self.z] + list(self.cfg['probe_ori'])

    def movel(self, p6, vel, acc=None, what=''):
        d = math.dist(self.pose()[:3], p6[:3])
        self.t_sim += d / vel + 0.3
        self.xy, self.z = (p6[0], p6[1]), p6[2]

    def sleep(self, s):
        self.t_sim += s

    def _surface(self, x, y):
        """(x, y) 에서 손가락 끝 아래의 큐브 겹침 비율과 윗면 높이."""
        cx, cy, deg, h = self.cube
        if self.held:
            return 0.0, None
        pts = self._fp + np.array([x, y])
        inside = rect_sdf(pts, cx, cy, math.radians(deg), self.cfg['cube']['long'] / 2,
                          self.cfg['cube']['short'] / 2) <= 0
        frac = float(inside.mean())
        return frac, self.cfg['z_floor'] + h

    def probe(self, x, y, z_from, z_to, vel, ori, f_limit=None, f_min=None):
        c = self.cfg
        self.n_probe += 1
        self.movel([x, y, z_from] + list(ori), c['vel_move'])
        frac, top = self._surface(x, y)
        floor = c['z_floor']
        thr = max(f_min or c['f_dyn_min'], 6 * self.NOISE)
        thr_st = max(f_min or c['f_static_min'], 5 * self.NOISE)
        f_limit = f_limit or c['f_limit']
        out = {'z': None, 'f': 0.0, 'hard': False, 'sigma': self.NOISE}
        if self.MOTION_BIAS + 3 * self.NOISE >= thr:
            raise SearchError('시뮬: 움직임 치우침이 접촉 기준보다 커서 공중에서도 감지됨 (f_dyn_min 확인)')
        # 바닥 (단단함)
        if z_to < floor:
            out.update(z=floor - 0.2, f=50.0, hard=True)
            return out
        if frac > 0:
            k = self.K_CUBE * frac
            pen = max(0.0, thr - self.MOTION_BIAS) / k      # 이만큼 눌려야 감지 (치우침이 기준 쪽으로 더해짐)
            pen += vel * self.F_PERIOD * random.random()    # 힘 갱신이 늦어 더 내려간 만큼
            z_hit = top - pen + random.gauss(0, 0.15)
            if z_hit >= z_to:
                # 실제 RobotIO.probe 처럼: 감지 → 멈춰서 정지 힘 확인 → 약하면 그 자리에서 다시 내려간다 (반복)
                while True:
                    f = k * (top - z_hit) + random.uniform(-1, 1) * self.STATIC_BIAS
                    if k * (top - z_hit) + self.MOTION_BIAS >= f_limit:
                        out.update(z=z_hit, f=f, hard=True)
                        break
                    if f >= thr_st:
                        out.update(z=z_hit, f=f)
                        break
                    z_hit -= max(0.0, thr - self.MOTION_BIAS - k * (top - z_hit)) / k + vel * self.F_PERIOD * random.random() + 0.05
                    if z_hit < z_to:
                        f = k * (top - z_to) + random.uniform(-1, 1) * self.STATIC_BIAS
                        if f >= thr_st:
                            out.update(z=z_to, f=f)
                        break
            else:
                f = k * max(0.0, top - z_to) + random.uniform(-1, 1) * self.STATIC_BIAS   # 끝까지 내려가 정지 상태에서 잼
                if f >= thr_st:
                    out.update(z=z_to, f=f)
        elif random.uniform(0, self.STATIC_BIAS) >= thr_st:            # 공중인데 멈춘 뒤 치우침이 기준을 넘는 경우
            out.update(z=z_to, f=thr_st)
        self.t_sim += abs(z_from - (out['z'] or z_to)) / vel * 2 + 1.0
        return out

    def gripper(self, what):
        self.t_sim += 1.0
        if what == 'grip':
            self.held = True
        if what == 'open':
            self.held = False


# ══════════════════════════════════════════════════════════════════════
# 작업
# ══════════════════════════════════════════════════════════════════════
class Job:
    def __init__(self, io, cfg, log, log_dir=LOG_DIR):
        self.io, self.cfg, self.log, self.log_dir = io, cfg, log, log_dir
        self.tray = load_tray(cfg)

    def lift_travel(self):
        p = self.io.pose()
        if p[2] < self.cfg['z_travel'] - 0.5:
            self.io.movel(p[:2] + [self.cfg['z_travel']] + p[3:], self.cfg['vel_move'], what='이동 높이로 상승')

    def search(self):
        probs = cfg_problems(self.cfg)
        if probs:
            raise SearchError('설정 문제: ' + ' / '.join(probs))
        t0 = time.monotonic()
        self.io.gripper('close')
        self.lift_travel()
        s = CubeSearcher(self.io, self.cfg, self.log)
        try:
            res = s.run()
        finally:
            self.lift_travel()
            self.save_log(s, locals().get('res'))
        res['elapsed'] = time.monotonic() - t0
        self.log('=' * 60)
        self.log(f'큐브 중심 ({res["cx"]:.2f}, {res["cy"]:.2f}) mm, 긴 축 {res["theta"]:.1f}°, '
                 f'윗면 z {res["z_top"]:.2f} (높이 {res["height"]:.1f} mm)')
        f1 = lambda x: '잘림' if x is None else f'{x:.1f}'
        self.log(f'잰 폭 긴 {f1(res["span_long"])} / 짧은 {f1(res["span_short"])} mm '
                 f'(큐브 + 손가락 끝 {f1(res["fp_long"])} / {f1(res["fp_short"])} mm씩), 터치 {res["touches"]}번')
        for w in res['warnings']:
            self.log(f'⚠ {w}')
        self.log('=' * 60)
        return res

    def tray_status(self):
        """칸마다 위에서 터치해서 큐브가 있는지 본다. 반환 {칸: True(있음)/False(비었음)}."""
        c = self.cfg
        self.io.gripper('close')
        self.lift_travel()
        out = {}
        for n in c['tray_order']:
            if n not in self.tray:
                continue
            t = self.tray[n]
            z_down = t['down'][2]
            z_end = z_down + 3.0 if c['tray_floor_z'] is None else c['tray_floor_z'] + 8.0   # 바닥에 닿지 않게
            r = self.io.probe(t['top'][0], t['top'][1], z_down + c['tray_check_from'], z_end,
                              c['vel_coarse'], t['top'][3:])
            if r['hard']:
                raise SearchError(f'트레이 {n}번 칸에서 딱딱한 물체에 걸렸습니다.')
            out[n] = r['z'] is not None
            self.log(f'  트레이 {n}번: {"큐브 있음 (윗면 z %.1f)" % r["z"] if out[n] else "비었음"}')
        self.lift_travel()
        return out

    def pick_place(self):
        c = self.cfg
        if not self.tray:
            raise SearchError(f'트레이 좌표가 없습니다: {c["tray_file"]}')
        if c['tray_check']:
            self.log('[0단계] 트레이 빈칸 확인')
            st = self.tray_status()
            empty = [n for n in c['tray_order'] if st.get(n) is False]
        else:
            empty = [n for n in c['tray_order'] if n in self.tray]
        if not empty:
            raise SearchError('트레이에 빈칸이 없습니다.')
        n = empty[0]
        t = self.tray[n]
        res = self.search()
        side = c['cube']['long'] if c['grasp_side'] == 'long' else c['cube']['short']
        if c['grip_open_width'] < side + 4.0:
            # 묻지 않고 알리기만 한다: 손가락이 걸리면 잡으러 내려갈 때 힘(grasp_f_limit)으로 멈추고 중단한다
            self.log(f'  참고: 열기 폭 {c["grip_open_width"]:g} mm, 잡는 변 {side:g} mm → 한쪽 여유 '
                     f'{(c["grip_open_width"] - side) / 2:.1f} mm (걸리면 힘으로 멈춤)')
        for w in res['warnings']:
            if input(f'⚠ {w}\n  그래도 잡을까요? (y/N) ').strip().lower() != 'y':
                return res
        c_tray = t['top'][5]
        cg = grasp_c(res['theta'], c['grasp_side'], c['probe_ori'][2], c_tray)
        z_grasp = max(res['z_top'] - c['grasp_depth'], c['z_floor'] + c['floor_clear'])
        g_above_floor = z_grasp - c['z_floor']
        if c['tray_floor_z'] is not None:
            z_place = c['tray_floor_z'] + g_above_floor + c['place_clear']
        else:
            z_place = t['down'][2]
            self.log('  tray_floor_z 가 없어 트레이 교시 높이(tray_b_down)에 놓습니다 (메뉴 tf 로 재면 큐브 높이에 맞춰 놓음)')
        ori = [c['probe_ori'][0], c['probe_ori'][1], cg]
        self.log(f'[4단계] 잡기: C = {cg:.2f}, 잡는 z = {z_grasp:.2f} (윗면 -{res["z_top"] - z_grasp:.1f}) '
                 f'→ 트레이 {n}번, 놓는 z = {z_place:.2f}')
        zt = c['z_travel']
        self.io.movel([res['cx'], res['cy'], zt] + ori, c['vel_move'], what='큐브 위로')
        self.io.gripper('open')
        self.io.movel([res['cx'], res['cy'], res['z_top'] + c['hover'] + 5] + ori, c['vel_move'], what='접근')
        # 마지막 하강은 힘을 보면서: 손가락이 큐브 윗면에 걸리면 멈춘다
        self.descend_guarded([res['cx'], res['cy'], z_grasp] + ori)
        self.io.gripper('grip')
        self.io.movel([res['cx'], res['cy'], zt] + ori, c['vel_slow'], what='들어 올리기')
        self.io.movel([t['top'][0], t['top'][1], zt] + t['top'][3:], c['vel_move'], what=f'트레이 {n}번 위로')
        self.io.movel(t['top'], c['vel_move'], what=f'트레이 {n}번 접근')
        self.io.movel(t['top'][:2] + [z_place] + t['top'][3:], c['vel_slow'], what='내려놓기')
        self.io.sleep(c['settle'])
        self.io.gripper('open')
        self.io.movel(t['top'], c['vel_slow'], what='상승')
        self.log(f'완료: 큐브를 트레이 {n}번 칸에 넣었습니다.')
        return res

    def last_result(self):
        """가장 최근 실물 탐지 기록(logs/search_*.json)의 결과."""
        import glob
        files = sorted(glob.glob(os.path.join(LOG_DIR, 'search_*.json')))
        for f in reversed(files):
            with open(f) as fp:
                r = json.load(fp).get('result')
            if r:
                return r, os.path.basename(f)
        raise SearchError('저장된 탐지 결과가 없습니다. 먼저 s 로 탐지하세요.')

    def pick_test(self, res=None, lift=30.0, hold=2.0):
        """탐지 결과로 잡아서 lift mm 들어 올렸다가 제자리에 내려놓는다 (탐지가 맞는지 확인용)."""
        c = self.cfg
        if res is None:
            res, name = self.last_result()
            self.log(f'기록 {name}: 중심 ({res["cx"]:.2f}, {res["cy"]:.2f}), 긴 축 {res["theta"]:.1f}°, 윗면 z {res["z_top"]:.2f}')
        cg = grasp_c(res['theta'], c['grasp_side'], c['probe_ori'][2], c['probe_ori'][2])
        z_grasp = max(res['z_top'] - c['grasp_depth'], c['z_floor'] + c['floor_clear'])
        ori = [c['probe_ori'][0], c['probe_ori'][1], cg]
        self.log(f'[잡기 확인] C = {cg:.2f}, 잡는 z = {z_grasp:.2f}, 들어 올림 {lift:g} mm')
        zt = c['z_travel']
        self.lift_travel()
        self.io.movel([res['cx'], res['cy'], zt] + ori, c['vel_move'], what='큐브 위로')
        self.io.gripper('open')
        self.io.movel([res['cx'], res['cy'], res['z_top'] + c['hover'] + 5] + ori, c['vel_move'], what='접근')
        self.descend_guarded([res['cx'], res['cy'], z_grasp] + ori)
        self.io.gripper('grip')
        self.io.sleep(0.5)
        self.io.movel([res['cx'], res['cy'], z_grasp + lift] + ori, c['vel_slow'], what='들어 올리기')
        self.log(f'  {lift:g} mm 들어 올림. {hold:g} 초 대기 (큐브가 잡혀 있는지 보세요)')
        self.io.sleep(hold)
        self.io.movel([res['cx'], res['cy'], z_grasp + 1.0] + ori, c['vel_slow'], what='내려놓기')
        self.io.gripper('open')
        self.io.sleep(0.5)
        self.io.movel([res['cx'], res['cy'], zt] + ori, c['vel_slow'], what='상승')
        self.log('잡기 확인 끝 (큐브는 제자리에 다시 놓음)')

    def descend_guarded(self, target6):
        """벌린 손가락으로 큐브 옆까지 내려간다. 손가락이 큐브에 걸리면(힘) 멈추고 중단."""
        c = self.cfg
        if isinstance(self.io, RobotIO):
            io = self.io
            io.sleep(0.15)
            vals = io.fz(8)
            tare = sum(vals) / len(vals)
            reason, z = io.descend(target6, min(c['vel_slow'], 10.0), tare, c['grasp_f_limit'], c['grasp_f_limit'] * 1.5)
            if reason != 'end' or z > target6[2] + 1.0:
                io.movel(target6[:2] + [c['z_travel']] + target6[3:], c['vel_slow'], what='중단 후 상승')
                raise SearchError(f'잡으러 내려가다 손가락이 걸렸습니다 (z {z:.1f}). 각도/중심 오차 또는 그리퍼가 덜 벌어짐.')
        else:
            self.io.movel(target6, c['vel_slow'])

    def measure_floor(self, x, y, ori, z_from, z_lo, key):
        """빈 바닥을 위에서 터치해서 높이를 잰다."""
        self.io.gripper('close')
        self.lift_travel()
        # 10 mm 씩 나눠 내려간다: 컨트롤러 작업 범위(알람 1206) 밖 목표는 통째로 거부되므로, 어디가 한계인지도 알 수 있게
        r, z_top_chunk, limit = {'z': None, 'hard': False, 'f': 0.0}, z_from, None
        try:
            while z_top_chunk > z_lo + 0.01:
                z_bot = max(z_lo, z_top_chunk - 10.0)
                try:
                    r = self.io.probe(x, y, z_top_chunk, z_bot, 5.0, ori, f_limit=self.cfg['f_limit'],
                                      f_min=self.cfg['floor_f_min'])
                except SearchError as e:
                    if '1206' in str(e):
                        limit = z_top_chunk
                        break
                    raise
                if r['z'] is not None or r['hard']:
                    break
                z_top_chunk = z_bot
        finally:
            self.lift_travel()
        if limit is not None:
            raise SearchError(f'z {limit:.1f} 보다 아래로는 컨트롤러 작업 범위 밖이라 내려갈 수 없습니다 (알람 1206). '
                              f'그 위에서는 바닥이 닿지 않았습니다. 이 위치({x:.0f}, {y:.0f})는 로봇 베이스에 가까워 '
                              '범위가 좁을 수 있으니 영역 안 다른 위치로 재 보세요.')
        if r['z'] is None and not r['hard']:
            raise SearchError(f'z {z_lo:.1f} 까지 내려가도 바닥이 닿지 않았습니다.')
        # 힘 갱신이 0.1 s 라 5 mm/s 에서는 늦게 멈춘다 → 닿은 곳 3 mm 위에서 느리게 다시 잰다
        z_h = r['z']
        self.log(f'  바닥 근처 z {z_h:.2f} 에서 닿음 (힘 {r["f"]:.1f} N) → 3 mm 위에서 '
                 f'{self.cfg["floor_vel_fine"]:g} mm/s 로 다시 잼')
        r = self.io.probe(x, y, z_h + 3.0, z_h - 2.0, self.cfg['floor_vel_fine'], ori, f_limit=self.cfg['f_limit'],
                          f_min=self.cfg['floor_f_min'])
        self.lift_travel()
        if r['z'] is None and not r['hard']:
            raise SearchError(f'다시 잴 때 z {z_h - 2.0:.1f} 까지 바닥이 닿지 않았습니다.')
        z = round(r['z'], 2)
        if z > z_from - 1.0:
            raise SearchError(f'시작 높이 z {z_from:.1f} 에서 내려가자마자 힘이 걸렸습니다 (z {z:.1f}). '
                              '그 자리에 물체가 있거나 시작 높이가 바닥보다 낮습니다. 저장하지 않았습니다.')
        self.log(f'바닥 z = {z:.2f} (힘 {r["f"]:.1f} N) → 설정 {key} 에 저장')
        save_cfg_value(key, z)
        self.cfg[key] = z
        return z

    def save_log(self, s, res):
        os.makedirs(self.log_dir, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        base = os.path.join(self.log_dir, f'search_{stamp}')

        def clean(o):
            if isinstance(o, dict):
                return {k: clean(v) for k, v in o.items()}
            if isinstance(o, (list, tuple)):
                return [clean(v) for v in o]
            if isinstance(o, (np.floating, np.integer)):
                return o.item()
            return o
        with open(base + '.json', 'w') as f:
            json.dump(clean({'result': res, 'touches': s.touches, 'cfg': self.cfg}), f, indent=1, ensure_ascii=False)
        try:
            self.plot(s, res, base + '.png')
            self.log(f'기록: {base}.json / .png')
        except Exception as e:  # matplotlib 없음 등
            self.log(f'기록: {base}.json (그림 실패: {e})')

    def plot(self, s, res, path):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon, Rectangle
        fig, ax = plt.subplots(figsize=(8, 7))
        a = self.cfg['area']
        ax.add_patch(Rectangle((a['x_min'], a['y_min']), a['x_max'] - a['x_min'], a['y_max'] - a['y_min'],
                               fill=False, ls='--', ec='gray', label='area'))
        style = {'coarse': ('s', 'tab:gray'), 'press': ('o', 'tab:blue'), 'top': ('*', 'tab:purple')}
        for kind, (m, col) in style.items():
            hit = [(t['x'], t['y']) for t in s.touches if t['kind'] == kind and t['z'] is not None]
            miss = [(t['x'], t['y']) for t in s.touches if t['kind'] == kind and t['z'] is None]
            if hit:
                ax.scatter(*zip(*hit), marker=m, c=col, s=22, label=f'{kind} hit')
            if miss:
                ax.scatter(*zip(*miss), marker=m, facecolors='none', edgecolors=col, s=22, label=f'{kind} miss')
        if res:
            L, S = self.cfg['cube']['long'] / 2, self.cfg['cube']['short'] / 2
            th = math.radians(res['theta'])
            u = np.array([math.cos(th), math.sin(th)])
            v = np.array([-math.sin(th), math.cos(th)])
            ctr = np.array([res['cx'], res['cy']])
            corners = [ctr + i * L * u + j * S * v for i, j in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
            ax.add_patch(Polygon(corners, closed=True, fc='tab:orange', alpha=0.3, ec='tab:orange',
                                 label=f'cube {res["theta"]:.1f} deg'))
            ax.scatter([res['cx']], [res['cy']], marker='x', c='k', s=60, label='center')
            e2 = np.array(res['edges2'])
            ax.scatter(e2[:, 0], e2[:, 1], marker='D', c='tab:red', s=18, label='edge (fine)')
        ax.set_aspect('equal')
        ax.set_xlabel('base x [mm]')
        ax.set_ylabel('base y [mm]')
        ax.grid(True, ls=':')
        ax.legend(fontsize=7, loc='best')
        fig.savefig(path, dpi=140, bbox_inches='tight')
        plt.close(fig)


def show_cfg(cfg):
    print(f'설정 파일: {CONFIG_PATH}')
    a = cfg['area']
    print(f'  area x {a["x_min"]} ~ {a["x_max"]}, y {a["y_min"]} ~ {a["y_max"]} | z_floor {cfg["z_floor"]}')
    print(f'  큐브 {cfg["cube"]} | 격자 {cfg["grid_step"]} mm | 누르기 {cfg["press_depth"]} mm, '
          f'경계 {cfg["f_static_min"]} N')
    print(f'  잡기 {cfg["grasp_side"]} 변, 윗면 -{cfg["grasp_depth"]} mm | 트레이 {cfg["tray_file"]} '
          f'(바닥 z {cfg["tray_floor_z"]})')
    for p in cfg_problems(cfg):
        print(f'  ⚠ {p}')


def menu(job, cfg, log):
    help_txt = ('s: 탐지만 | g: 탐지→잡기→트레이 빈칸 | t: 트레이 빈칸 확인 | f: 영역 바닥 재기 | '
                'tf: 트레이 바닥 재기 | p: 마지막 탐지 결과로 잡았다 놓기 | c: 설정 | q: 종료')
    show_cfg(cfg)
    print(help_txt)
    while True:
        sel = input('> ').strip().lower()
        try:
            if sel == 'q':
                return
            if sel == 's':
                job.search()
            elif sel == 'p':
                job.pick_test()
            elif sel == 'g':
                job.pick_place()
            elif sel == 't':
                job.tray_status()
            elif sel == 'f':
                a = cfg['area']
                if a['x_min'] is None:
                    print('  area 를 먼저 설정하세요.')
                    continue
                txt = input(f'  잴 위치 x,y (Enter = 영역 구석 {a["x_min"] + 10:.0f},{a["y_min"] + 10:.0f}). '
                            '그 자리에 큐브가 없어야 합니다: ').strip()
                x, y = (map(float, txt.split(','))) if txt else (a['x_min'] + 10, a['y_min'] + 10)
                z_hi = float(input('  지금 바닥보다 확실히 높은 z (예 150): ').strip() or 150)
                job.measure_floor(x, y, cfg['probe_ori'], z_hi, z_hi - cfg['floor_search'], 'z_floor')
            elif sel == 'tf':
                n = int(input('  빈 트레이 칸 번호: ').strip())
                t = job.tray[n]
                job.measure_floor(t['top'][0], t['top'][1], t['top'][3:], t['down'][2] + 10.0,
                                  t['down'][2] - 40.0, 'tray_floor_z')
            elif sel == 'c':
                show_cfg(cfg)
            else:
                print(help_txt)
        except SearchError as e:
            log(f'중단: {e}')
        except ValueError as e:
            print(f'  입력 오류: {e}')


def main(args=None):
    argv = sys.argv[1:]
    sim = '--sim' in argv
    cfg = load_cfg()

    def log(msg):
        print(f'{time.strftime("%H:%M:%S")} {msg}', flush=True)

    if sim:
        rest = [a for a in argv if a not in ('--sim', '--place') and not a.startswith('--ros-args')]
        if cfg['area']['x_min'] is None:
            cfg['area'] = dict(SIM_AREA)
        if cfg['z_floor'] is None:
            cfg['z_floor'] = SIM_FLOOR
        a = cfg['area']
        if rest and ',' in rest[0]:
            cx, cy, deg = map(float, rest[0].split(','))
        else:
            deg = random.uniform(0, 180)   # 큐브 전체가 영역 안에 (경계에 붙는 경우 포함)
            t = math.radians(deg)
            L2, S2 = cfg['cube']['long'] / 2, cfg['cube']['short'] / 2
            hx = L2 * abs(math.cos(t)) + S2 * abs(math.sin(t))
            hy = L2 * abs(math.sin(t)) + S2 * abs(math.cos(t))
            cx = random.uniform(a['x_min'] + hx, a['x_max'] - hx)
            cy = random.uniform(a['y_min'] + hy, a['y_max'] - hy)
        cube = (cx, cy, deg, 30.0)
        log(f'[시뮬] 실제 큐브: 중심 ({cx:.2f}, {cy:.2f}), 긴 축 {deg:.1f}°, 높이 30')
        io = SimIO(cfg, log, cube)
        job = Job(io, cfg, log, log_dir=os.path.join(LOG_DIR, 'sim'))
        try:
            res = job.pick_place() if '--place' in argv else job.search()
        except SearchError as e:
            log(f'실패: {e}')
            sys.exit(1)
        derr = math.dist((res['cx'], res['cy']), (cx, cy))
        aerr = ((res['theta'] - deg + 90) % 180) - 90
        log(f'[시뮬] 오차: 중심 {derr:.2f} mm, 각도 {aerr:+.2f}°, 터치 {io.n_probe}번, 예상 시간 {io.t_sim / 60:.1f} 분')
        return

    import rclpy
    from rclpy.utilities import remove_ros_args
    import DR_init
    remove_ros_args(sys.argv)
    DR_init.__dsr__id = 'dsr01'
    DR_init.__dsr__model = 'm0609'
    rclpy.init(args=args)
    node = rclpy.create_node('pjt_cube_search', namespace='dsr01')
    DR_init.__dsr__node = node
    try:
        io = RobotIO(node, cfg, log)
        io.prepare()
        menu(Job(io, cfg, log), cfg, log)
    except SearchError as e:
        log(f'오류: {e}')
    except (KeyboardInterrupt, EOFError):
        print()
        log('중단됨 (로봇이 아직 움직이면 펜던트에서 정지하세요)')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
