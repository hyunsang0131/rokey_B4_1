# 큐브 탐지 알고리즘 (로봇과 무관한 순수 계산). cube_search 가 실제 로봇 / 가짜 로봇(시뮬)으로 같은 코드를 돌린다.
#
# 원칙: 큐브를 옆에서 밀지 않는다. 모든 접촉은 '위에서 수직으로 살짝 누르기'뿐이다.
#   무른 실리콘 큐브는 바닥 마찰보다 접촉 판정 힘(수 N)이 커서, 옆에서 찌르면 감지 전에 밀려 버린다.
#   위에서 누르면 큐브가 바닥에 눌려 움직이지 않는다.
#
# 순서
#   1) 거친 격자 탐색: 영역을 grid_step 간격으로 위에서 터치 (영역 가운데부터 바깥으로). 처음 닿은 곳 = 큐브 안쪽 점 P0, 윗면 높이
#      격자 간격 25 mm < 41/√2 (29) → 큐브가 어떤 방향으로 놓여도 격자점 하나는 반드시 큐브 위에 있다.
#   2) 1차 윤곽: P0 에서 ray_count 방향으로 '있다/없다' 이분 탐색 → 경계까지 거리
#      → '큐브(크기 앎) ⊕ 손가락 끝(베이스 축 방향 직사각형)' 모양을 맞춤 (중심, 각도 대략, 손가락 끝 크기)
#   3) 2차 정밀: 맞춘 직사각형의 네 변에 수직으로 다시 이분 탐색 (긴 변 2점씩, 짧은 변 1점씩)
#      → 같은 변 두 점으로 각도, 마주 보는 변의 중간으로 중심
#      손가락 끝 크기만큼 가장자리가 바깥으로 잡히지만, 마주 보는 변은 같은 만큼 밀리므로 중심과 각도에는 영향이 없다.
#
# '있다' 판정 = 윗면보다 hover 위에서 영점을 잡고, 윗면 아래 press_depth 까지 내려가는 동안 힘이 경계를 넘으면 있음.

import math

import numpy as np


class SearchError(Exception):
    """탐지를 계속할 수 없는 오류."""


def wrap180(deg):
    """각도를 [0, 180) 로 (직사각형은 180도 돌려도 같다)."""
    return deg % 180.0


def ang_diff180(a, b):
    """180도 주기 각도 차이 (-90, 90]."""
    d = (a - b) % 180.0
    return d - 180.0 if d > 90.0 else d


def rect_sdf(pts, cx, cy, th, a, b):
    """점들의 직사각형(중심 cx, cy, 긴 축 각도 th[rad], 반폭 a, b) 경계까지 부호 거리 (밖 +, 안 -)."""
    u = np.array([math.cos(th), math.sin(th)])
    v = np.array([-math.sin(th), math.cos(th)])
    d = pts - np.array([cx, cy])
    dx = np.abs(d @ u) - a
    dy = np.abs(d @ v) - b
    outside = np.hypot(np.maximum(dx, 0.0), np.maximum(dy, 0.0))
    inside = np.minimum(np.maximum(dx, dy), 0.0)
    return outside + inside


def _gauss_newton(fun, x0, iters=20, eps=1e-4):
    x = np.array(x0, dtype=float)
    for _ in range(iters):
        r = fun(x)
        J = np.empty((r.size, x.size))
        for k in range(x.size):
            dx = np.zeros_like(x)
            dx[k] = eps
            J[:, k] = (fun(x + dx) - r) / eps
        step, *_ = np.linalg.lstsq(J, -r, rcond=None)
        x = x + step
        if np.linalg.norm(step) < 1e-6:
            break
    return x


def support(n, cx, cy, th, a, b, ex, ey):
    """큐브(중심, 각도, 반폭 a, b)와 손가락 끝(베이스 축에 나란한 반폭 ex, ey)을 합친 도형의 지지 함수 h(n).

    손가락 끝이 큐브에 조금이라도 닿으면 '있음' → 실제로 재는 모양은 큐브 ⊕ 손가락 끝 (민코프스키 합).
    """
    u = (math.cos(th), math.sin(th))
    v = (-math.sin(th), math.cos(th))
    return (n[0] * cx + n[1] * cy + a * abs(n[0] * u[0] + n[1] * u[1]) + b * abs(n[0] * v[0] + n[1] * v[1])
            + ex * abs(n[0]) + ey * abs(n[1]))


def ray_length(p0, d, cx, cy, th, a, b, ex, ey):
    """p0(안쪽 점)에서 d 방향으로 합친 도형 경계까지 거리. 면의 법선은 ±u, ±v, ±x, ±y 8개뿐이다."""
    u = (math.cos(th), math.sin(th))
    v = (-math.sin(th), math.cos(th))
    best = float('inf')
    for n in (u, v, (1.0, 0.0), (0.0, 1.0)):
        for sg in (1.0, -1.0):
            nn = (sg * n[0], sg * n[1])
            nd = nn[0] * d[0] + nn[1] * d[1]
            if nd > 1e-9:
                h = support(nn, cx, cy, th, a, b, ex, ey)
                best = min(best, (h - (nn[0] * p0[0] + nn[1] * p0[1])) / nd)
    return best


def fit_rays(p0, dirs, s_meas, long_mm, short_mm, ex0, ey0, prior_w=0.3, clipped=None):
    """한 안쪽 점에서 여러 방향으로 잰 경계 거리에 '큐브 ⊕ 손가락 끝' 모양을 맞춘다.

    맞추는 값: 중심 cx, cy, 긴 축 각도, 손가락 끝 반폭 ex, ey (설정값 ex0, ey0 쪽으로 약하게 끌어당김)
    clipped[k] 가 참인 방향은 영역 경계에서 잘린 값(실제 경계는 그보다 바깥)이라 '모델 ≥ 잰 값' 으로만 쓴다.
    반환 (cx, cy, 각도[deg, 0~180), ex, ey, 잔차 RMS[mm] (잘리지 않은 방향만))
    """
    a, b = long_mm / 2.0, short_mm / 2.0
    p0 = (float(p0[0]), float(p0[1]))

    clipped = list(clipped) if clipped is not None else [False] * len(dirs)

    def res(x):
        r = []
        for d, s, cl in zip(dirs, s_meas, clipped):
            e = ray_length(p0, d, x[0], x[1], x[2], a, b, x[3], x[4]) - s
            r.append(min(e, 0.0) if cl else e)
        r += [prior_w * (x[3] - ex0), prior_w * (x[4] - ey0)]
        return np.array(r)
    best = None
    for th_deg in np.arange(0.0, 180.0, 5.0):
        for x0 in (p0,):
            x = _gauss_newton(res, [x0[0], x0[1], math.radians(th_deg), ex0, ey0], iters=15)
            cost = float(np.sum(res(x) ** 2))
            if best is None or cost < best[0]:
                best = (cost, x)
    x = _gauss_newton(res, best[1], iters=30)
    r = np.array([e for e, cl in zip(res(x)[:-2], clipped) if not cl])
    return (float(x[0]), float(x[1]), wrap180(math.degrees(x[2])), float(x[3]), float(x[4]),
            float(np.sqrt(np.mean(r ** 2))))


def grasp_c(theta_deg, side, c_ref, c_near):
    """긴 축이 theta_deg 인 큐브를 잡는 툴 C 각도.

    c_ref : 그리퍼가 베이스 x 방향으로 벌어지는 자세의 C (교시 자세 [168.25, 179.97, -11.77] 의 -11.77)
            B ≈ 180 이면 툴의 실제 회전은 A - C 라서, C 를 줄이면 벌어지는 방향이 반시계로 돈다.
    side  : 'long' = 긴 변(64)을 잡는다 (손가락이 긴 축 방향으로 벌어짐), 'short' = 짧은 변(41)
    c_near: 그리퍼는 180도 돌려도 같으므로 후보 둘 중 이 값에 가까운 쪽 (J6 를 덜 돌리게)
    """
    phi = theta_deg if side == 'long' else theta_deg + 90.0
    c = c_ref - phi
    cands = [c + k * 180.0 for k in range(-3, 4)]
    return min(cands, key=lambda x: abs(x - c_near))


class CubeSearcher:
    """io 가 제공하는 probe() 하나로 큐브의 중심, 각도, 윗면 높이를 찾는다.

    io.probe(x, y, z_from, z_to, vel, ori) → {'z': 닿은 높이 또는 None, 'f': 정지 상태 힘[N], 'hard': bool}
      (x, y, z_from) 으로 가서 영점을 잡고 z_to 까지 수직으로 내려가며 닿으면 멈추고, 끝나면 z_from 으로 돌아온다.
    """

    def __init__(self, io, cfg, log):
        self.io, self.cfg, self.log = io, cfg, log
        self.touches = []           # (x, y, 결과) 기록 (그림, 로그용)
        self.live = None            # 실시간 표시 콜백 live(kind, dict) (HMI). 없으면 안 보냄

    def emit(self, kind, **d):
        if self.live is not None:
            try:
                self.live(kind, d)
            except Exception:       # 표시 실패가 탐지를 멈추면 안 된다
                pass

    def n_touch(self):
        return sum(1 for t in self.touches if t['kind'] != 'outside')

    # ── 기본 동작 ─────────────────────────────────────────────────────
    def in_area(self, x, y, margin=0.0):
        a = self.cfg['area']
        return (a['x_min'] - margin <= x <= a['x_max'] + margin and
                a['y_min'] - margin <= y <= a['y_max'] + margin)

    def _probe(self, x, y, z_from, z_to, vel, kind):
        r = self.io.probe(x, y, z_from, z_to, vel, self.cfg['probe_ori'])
        self.touches.append({'x': x, 'y': y, 'kind': kind, 'z': r['z'], 'f': r['f']})
        self.emit('touch', x=x, y=y, hit=r['z'] is not None, n=self.n_touch())
        if r['hard']:
            raise SearchError(f'({x:.1f}, {y:.1f}) 에서 딱딱한 물체에 걸렸습니다 (힘 한계). '
                              '영역 안에 큐브 말고 높은 물체가 없는지, z_floor 설정을 확인하세요.')
        return r

    def present(self, x, y):
        """(x, y) 에 큐브 윗면이 있나. 영역 + edge_margin 밖은 터치하지 않고 '없음'.

        가장자리 탐색은 손가락 끝이 큐브 변을 완전히 벗어나야 '없음' 이 되므로, 큐브가 영역 경계에 붙어 있으면
        TCP 가 영역 밖으로 (손가락 끝 반폭만큼) 나가야 변을 잴 수 있다. 거친 격자는 영역 안만 본다.
        """
        if not self.in_area(x, y, self.cfg['edge_margin']):
            self.touches.append({'x': x, 'y': y, 'kind': 'outside', 'z': None, 'f': 0.0})
            self.emit('touch', x=x, y=y, hit=None, n=self.n_touch())
            return False
        c = self.cfg
        r = self._probe(x, y, self.z_top + c['hover'], self.z_top - c['press_depth'], c['vel_press'], 'press')
        return r['z'] is not None

    # ── 1) 거친 격자 탐색 ─────────────────────────────────────────────
    def grid_points(self):
        a, s = self.cfg['area'], self.cfg['grid_step']
        xs = np.arange(a['x_min'], a['x_max'] + 1e-6, s)
        ys = np.arange(a['y_min'], a['y_max'] + 1e-6, s)
        # 마지막 줄이 경계에서 떨어져 있으면 경계에 한 줄 더 (큐브가 구석에 있어도 놓치지 않게)
        if a['x_max'] - xs[-1] > 1.0:
            xs = np.append(xs, a['x_max'])
        if a['y_max'] - ys[-1] > 1.0:
            ys = np.append(ys, a['y_max'])
        cx, cy = (a['x_min'] + a['x_max']) / 2, (a['y_min'] + a['y_max']) / 2
        pts = [(float(x), float(y)) for x in xs for y in ys]
        return sorted(pts, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)

    def coarse(self):
        c = self.cfg
        z_from = c['z_floor'] + c['cube']['h_max'] + c['hover']
        z_to = c['z_floor'] + c['cube']['h_min']
        pts = self.grid_points()
        self.log(f'[1단계] 거친 격자 탐색: {len(pts)}점 (간격 {c["grid_step"]} mm), 가운데부터')
        for i, (x, y) in enumerate(pts, 1):
            r = self._probe(x, y, z_from, z_to, c['vel_coarse'], 'coarse')
            if r['z'] is not None and r['z'] < z_to + c['coarse_min_rise']:
                # 끝까지 내려가서야 힘이 생김 = 손가락 끝 가장자리만 큐브 모서리에 스침. 이 점은 큐브 안이 아닐 수 있다
                self.log(f'  {i}/{len(pts)} ({x:.1f}, {y:.1f}) 가장자리만 스침 → 큐브 근처. 계속 찾는다')
                continue
            if r['z'] is not None:
                self.log(f'  {i}/{len(pts)} ({x:.1f}, {y:.1f}) 닿음: 윗면 z ≈ {r["z"]:.2f} (힘 {r["f"]:.1f} N)')
                h = r['z'] - c['z_floor']
                if h > c['cube']['h_max'] + 2:
                    raise SearchError(f'윗면 높이 {h:.1f} mm 가 큐브 최대 높이보다 큽니다. z_floor 를 다시 재세요.')
                return x, y, r['z']
        return None

    def measure_top(self, x, y):
        """(x, y) 에서 윗면 높이 (지금 아는 높이 위아래로 천천히)."""
        c = self.cfg
        if not self.in_area(x, y):
            return None
        r = self._probe(x, y, self.z_top + c['hover'] + 5, self.z_top - c['press_depth'] - 5, c['vel_press'], 'top')
        return r['z']

    # ── 가장자리 이분 탐색 ───────────────────────────────────────────
    def find_edge(self, o, d, s_in, s_out, tol, in_known=False):
        """o + s·d 위에서 '있음 → 없음' 이 바뀌는 거리 s, 그 점, 영역 경계에 잘렸는지.

        s_in 은 있음, s_out 은 없음이라고 '가정'하고 바로 이분 탐색한다 (양 끝을 미리 확인하는 터치를 아낀다).
        끝까지 한쪽만 나오면 가정이 틀린 것이므로 그쪽으로 구간을 넓혀 다시 한다.
        """
        o, d = np.asarray(o, float), np.asarray(d, float)
        span = s_out - s_in
        known_lo, known_hi = in_known, False     # 이미 터치로 확인된 끝 (있음 / 없음)
        for _ in range(5):
            lo, hi, moved_lo, moved_hi = s_in, s_out, False, False
            while hi - lo > tol:
                m = (lo + hi) / 2
                p = o + m * d
                if self.present(float(p[0]), float(p[1])):
                    lo, moved_lo = m, True
                else:
                    hi, moved_hi = m, True
            if (moved_lo or known_lo) and (moved_hi or known_hi):
                break
            if not moved_hi:                         # 전부 '있음' → 경계가 더 바깥. 마지막 '있음' 점부터 넓힌다
                s_in, s_out, known_lo, known_hi = lo, s_out + span, True, False
            else:                                    # 전부 '없음' → 경계가 더 안쪽. 마지막 '없음' 점까지로 넓힌다
                if s_in <= 0.0:
                    raise SearchError('가장자리 탐색: 안쪽 점에서도 큐브가 없습니다 (큐브가 밀렸거나 탐지 오류).')
                s_in, s_out, known_lo, known_hi = max(0.0, s_in - span), hi, False, True
        else:
            raise SearchError('가장자리 탐색: 경계를 찾지 못했습니다 (큐브가 붙어 있거나 크기 설정이 다름).')
        s = (lo + hi) / 2
        p = o + s * d
        ph = o + hi * d
        clipped = not self.in_area(float(ph[0]), float(ph[1]), self.cfg['edge_margin'])   # '없음' 쪽 끝이 영역 밖 → 실제 경계는 더 바깥일 수 있다
        return s, (float(p[0]), float(p[1])), clipped

    def h_fp(self, n):
        """손가락 끝 크기 때문에 n 방향 가장자리가 바깥으로 밀려 보이는 양."""
        return self.ex * abs(n[0]) + self.ey * abs(n[1])

    # ── 2) 1차 윤곽 ───────────────────────────────────────────────────
    def round1(self, p0):
        c = self.cfg
        L, S = c['cube']['long'], c['cube']['short']
        reach = math.hypot(L, S) + 2 * max(c['finger_ex'], c['finger_ey']) + 6.0   # 큐브 안 어느 점에서든 이 거리면 밖
        n = c['ray_count']
        self.log(f'[2단계] 1차 윤곽: {n}방향 이분 탐색 (허용 {c["ray_tol"]} mm)')
        dirs, ss, edges, clips = [], [], [], []
        for k in range(n):
            ang = math.radians(360.0 * k / n + 11.0)    # 베이스 축과 겹치지 않게 조금 돌림
            d = (math.cos(ang), math.sin(ang))
            s, p, cl = self.find_edge(p0, d, 0.0, reach, c['ray_tol'], in_known=True)
            dirs.append(d)
            ss.append(s)
            edges.append(p)
            clips.append(cl)
        if sum(not x for x in clips) < 4:
            raise SearchError('1차 윤곽: 영역 경계에 잘린 방향이 너무 많습니다 (큐브가 영역 밖으로 걸쳤나?).')
        if any(clips):
            self.log(f'  영역 경계에 잘린 방향 {sum(clips)}개 → 그 방향은 "이보다 바깥" 조건으로만 씀')
        cx, cy, th, ex, ey, rms = fit_rays(p0, dirs, ss, L, S, c['finger_ex'], c['finger_ey'], clipped=clips)
        self.ex, self.ey = max(0.0, ex), max(0.0, ey)
        self.log(f'  맞춤: 중심 ({cx:.1f}, {cy:.1f}), 긴 축 {th:.1f}°, 손가락 끝 반폭 x {ex:.1f} / y {ey:.1f} mm, '
                 f'잔차 {rms:.2f} mm')
        if rms > c['fit_rms_max']:
            raise SearchError(f'1차 윤곽이 {L}×{S} 큐브와 맞지 않습니다 (잔차 {rms:.1f} mm). '
                              '큐브 크기 설정, 큐브가 2개 붙어 있는지, 큐브가 밀렸는지, 영역 경계에 걸쳤는지 확인하세요.')
        return {'cx': cx, 'cy': cy, 'th': th, 'ex': ex, 'ey': ey, 'edges1': edges, 'rms': rms, 'clipped1': sum(clips)}

    # ── 3) 2차 정밀 ───────────────────────────────────────────────────
    def round2(self, est):
        c = self.cfg
        a, b = c['cube']['long'] / 2, c['cube']['short'] / 2
        th = math.radians(est['th'])
        u = np.array([math.cos(th), math.sin(th)])
        v = np.array([-math.sin(th), math.cos(th)])
        ctr = np.array([est['cx'], est['cy']])
        w, off = c['edge_window'], min(c['edge_offset'], a - 12.0)
        self.log(f'[3단계] 2차 정밀: 각 변에 수직으로 (긴 변 2점씩, 짧은 변 1점씩, 허용 {c["edge_tol"]} mm)')

        def edge(o, d, expect):
            _, p, cl = self.find_edge(o, d, max(0.0, expect - w), expect + w, c['edge_tol'])
            return np.array(p), cl
        pv, cv_clip = {}, {}
        for sg in (+1, -1):                              # 긴 변 두 개, 각각 두 점
            exp_v = b + self.h_fp(v)
            got = [edge(ctr + t * u, sg * v, exp_v) for t in (-off, +off)]
            pv[sg] = [g[0] for g in got]
            cv_clip[sg] = any(g[1] for g in got)
        exp_u = a + self.h_fp(u)
        pu, cu_clip = {}, {}
        for sg in (+1, -1):                              # 짧은 변 두 개, 가운데 한 점
            pu[sg], cu_clip[sg] = edge(ctr, sg * u, exp_u)
        if all(cv_clip.values()) or all(cu_clip.values()):
            raise SearchError('마주 보는 두 변이 모두 영역 경계에 잘렸습니다 (영역이 큐브보다 좁거나 큐브가 영역 밖에 걸침).')
        clipped = [f'{"+" if sg > 0 else "-"}{ax}' for ax, cl in (('v', cv_clip), ('u', cu_clip))
                   for sg in (+1, -1) if cl[sg]]
        if clipped:
            self.log(f'  영역 경계에 잘린 변 {clipped} → 반대쪽 변 + 큐브 크기 + 손가락 끝 폭으로 중심 계산')

        ok_v = [sg for sg in (+1, -1) if not cv_clip[sg]]
        angs = []
        for sg in ok_v:
            dv = pv[sg][1] - pv[sg][0]
            angs.append(math.degrees(math.atan2(dv[1], dv[0])))
        d_ang = ang_diff180(angs[1], angs[0]) if len(angs) == 2 else 0.0
        th_new = wrap180(angs[0] + d_ang / 2)
        t = math.radians(th_new)
        u_old = u
        u = np.array([math.cos(t), math.sin(t)])
        if u @ u_old < 0:                                # 0/180 경계를 넘어도 이전 축과 같은 방향으로 (부호 유지)
            u = -u
        v = np.array([-u[1], u[0]])
        if len(ok_v) == 2:
            cv = np.mean([p @ v for sg in (+1, -1) for p in pv[sg]])
            span_short = float(np.mean([(pv[+1][k] - pv[-1][k]) @ v for k in (0, 1)]))
        else:                                            # 한쪽만: 잰 변에서 (반폭 + 손가락 끝) 만큼 안쪽
            sg = ok_v[0]
            cv = np.mean([p @ v for p in pv[sg]]) - sg * (b + self.h_fp(v))
            span_short = None
        if not (cu_clip[+1] or cu_clip[-1]):
            cu = (pu[+1] @ u + pu[-1] @ u) / 2
            span_long = float((pu[+1] - pu[-1]) @ u)
        else:
            sg = +1 if not cu_clip[+1] else -1
            cu = pu[sg] @ u - sg * (a + self.h_fp(u))
            span_long = None
        center = cu * u + cv * v
        fp = lambda span, nom: None if span is None else (span - nom) / 2
        return {'cx': float(center[0]), 'cy': float(center[1]), 'th': th_new,
                'span_long': span_long, 'span_short': span_short, 'edge_parallel_deg': d_ang,
                # 잰 폭에서 큐브 크기를 빼면 손가락 끝 때문에 한쪽이 밀려 보인 양 (정상이면 0 ~ 손가락 끝 반폭)
                'fp_long': fp(span_long, c['cube']['long']), 'fp_short': fp(span_short, c['cube']['short']),
                'clipped': clipped,
                'edges2': [p.tolist() for p in pv[+1] + pv[-1] + [pu[+1], pu[-1]]]}

    # ── 전체 ──────────────────────────────────────────────────────────
    def run(self):
        hit = self.coarse()
        if hit is None:
            raise SearchError('영역 전체를 터치했지만 큐브가 없습니다.')
        x0, y0, self.z_top = hit
        est = self.round1((x0, y0))
        return self.refine(est)

    def refine(self, est):
        """대략 추정(est: cx, cy, th)에서 2차 정밀(변에 수직 이분 탐색)과 윗면 높이로 최종 결과를 만든다."""
        c = self.cfg
        z = self.measure_top(est['cx'], est['cy'])     # 가장자리가 아닌 가운데에서 윗면을 다시 재서 이후 누르는 깊이를 맞춘다
        if z is not None:
            self.z_top = z
        r2 = self.round2(est)

        def swapped(r):
            sl, ss_ = r['span_long'], r['span_short']
            return sl is not None and ss_ is not None and sl < ss_
        late_warn = []
        for _ in range(c['refine_max']):
            if swapped(r2):                                # 1차 각도가 90도 가까이 틀림: 축을 바꿔 다시
                self.log('  긴 변/짧은 변이 바뀌어 보임 → 축을 90도 돌려 다시 잰다')
                est.update(th=wrap180(est['th'] + 90.0))
            elif abs(ang_diff180(r2['th'], est['th'])) > c['refine_repeat_deg']:
                # 재는 위치가 큐브 변 끝(모서리)에 걸렸을 수 있다 → 새 축 기준으로 다시
                self.log(f'  각도가 {ang_diff180(r2["th"], est["th"]):+.1f}° 바뀜 → 새 축으로 한 번 더')
                est.update(cx=r2['cx'], cy=r2['cy'], th=r2['th'])
            else:
                break
            r2 = self.round2(est)
        else:
            if swapped(r2):
                raise SearchError('긴 변과 짧은 변을 구분하지 못했습니다 (큐브 크기 설정 또는 큐브가 밀림).')
            late_warn.append(f'정밀 측정 각도가 아직 {ang_diff180(r2["th"], est["th"]):+.1f}° 흔들림 (마지막 값 사용)')
        z = self.measure_top(r2['cx'], r2['cy'])
        z_top = z if z is not None else self.z_top
        if z is None:
            self.log('  중심에서 윗면이 안 잡힘 → 이전 높이를 씀 (주의)')
        warn = late_warn
        if abs(r2['edge_parallel_deg']) > c['parallel_warn_deg']:
            warn.append(f'마주 보는 두 긴 변이 {r2["edge_parallel_deg"]:+.1f}° 어긋남 (큐브가 휘었거나 밀림?)')
        if r2['clipped']:
            warn.append(f'영역 경계에 잘린 변 {r2["clipped"]}: 그 방향 중심은 손가락 끝 폭 추정값이라 수 mm 이상 틀릴 수 있음 '
                        '→ edge_margin 을 늘리거나 큐브를 영역 안쪽으로')
        for k, nom in (('fp_long', c['cube']['long']), ('fp_short', c['cube']['short'])):
            if r2[k] is None:
                continue
            if not -c['size_tol_mm'] <= r2[k] <= c['finger_max'] + c['size_tol_mm']:
                warn.append(f'{nom:.0f} mm 변 방향으로 잰 폭이 {nom + 2 * r2[k]:.1f} mm → 큐브 크기가 설정과 다르거나 '
                            '큐브가 밀림/휨')
        res = {'cx': r2['cx'], 'cy': r2['cy'], 'theta': r2['th'], 'z_top': z_top,
               'height': z_top - c['z_floor'], 'span_long': r2['span_long'], 'span_short': r2['span_short'],
               'fp_long': r2['fp_long'], 'fp_short': r2['fp_short'], 'finger': [self.ex, self.ey],
               'edge_parallel_deg': r2['edge_parallel_deg'], 'round1': est, 'edges2': r2['edges2'],
               'touches': len([t for t in self.touches if t['kind'] != 'outside']), 'warnings': warn}
        return res
