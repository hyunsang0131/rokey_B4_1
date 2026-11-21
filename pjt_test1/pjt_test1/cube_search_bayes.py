# 큐브 탐지: 정보 이득 기반 능동 촉각 위치 추정 (파티클 필터). 카메라 없이 터치 횟수를 줄인다.
#
# 참고: Petrovskaya & Khatib 2011 "Global Localization of Objects via Touch",
#       Hsiao, Kaelbling, Lozano-Perez 2010 "Task-driven tactile exploration".
#
# 기존 방식(cube_search_core.CubeSearcher)과의 차이
#   - 큐브 크기(64×41)는 알고 있으므로 모르는 값은 중심 x, y, 각도 θ, 손가락 끝 때문에 변이 밀려 보이는 양 du, dv 다섯 개뿐이다.
#   - 이 다섯 값의 후보(파티클) 수만 개를 두고, 매번 '있다/없다' 가 반반으로 갈리는(정보 이득이 가장 큰) 점을 골라 터치한다.
#     변마다 따로 이분 탐색하지 않고 다섯 값을 한꺼번에 좁히므로 같은 정확도에 터치가 적다.
#   - 터치 결과는 확률(가장자리 근처는 애매, 가끔 틀림)로 다루므로 터치 하나가 틀려도 결과가 크게 망가지지 않는다.
#   - 끝내는 조건은 '후보들의 표준편차' (중심 bayes_pos_std mm, 각도 bayes_ang_std°) → 정확도는 설정값으로 정한다.
#
# 접촉 방식은 기존과 같다: 위에서 수직으로 살짝 누르기만 한다 (옆에서 밀지 않음).
# 1단계 거친 격자 탐색, 윗면 높이 재기, 결과 형식은 CubeSearcher 와 같다 (잡기·트레이 코드는 그대로 쓴다).

import math

import numpy as np

from pjt_test1.cube_search_core import CubeSearcher, SearchError, wrap180

LN2 = math.log(2.0)


def _h2(p):
    """이진 엔트로피 [bit]."""
    p = np.clip(p, 1e-9, 1 - 1e-9)
    return -(p * np.log(p) + (1 - p) * np.log(1 - p)) / LN2


def local(px, py, cx, cy, th):
    """점을 큐브 좌표 (긴 축 u, 짧은 축 v) 로."""
    ux, uy = np.cos(th), np.sin(th)
    dx, dy = px - cx, py - cy
    return dx * ux + dy * uy, -dx * uy + dy * ux


def sdf(px, py, cx, cy, th, du, dv, a, b):
    """'있음' 으로 잡히는 영역 = 큐브를 변마다 바깥으로 민 직사각형 (반폭 a+du, b+dv) 까지 부호 거리 (밖 +, 안 -).

    손가락 끝이 큐브 변에 조금만 걸쳐도 '있음' 이라 변이 바깥으로 밀려 보인다. 밀리는 양은 변의 방향에만 달렸고
    (마주 보는 변은 같은 양) 변은 큐브 변과 나란하다 → 중심과 각도는 손가락 끝 모양과 무관하게 맞는다.
    모서리 근처는 손가락 끝 모양에 따라 둥글게 깎이므로 corner_zone 에서 따로 애매하게 다룬다.
    """
    pu, pv = local(px, py, cx, cy, th)
    return np.maximum(np.abs(pu) - (a + du), np.abs(pv) - (b + dv))


def corner_zone(px, py, cx, cy, th, du, dv, a, b, cz):
    """큐브 모서리에서 변을 따라 cz 이내인 영역 (손가락 끝이 두 변에 함께 걸려 경계가 둥글게 깎이는 곳)."""
    pu, pv = local(px, py, cx, cy, th)
    return (np.abs(pu) > a - cz) & (np.abs(pv) > b - cz)


def boundary_point(cx, cy, th, du, dv, a, b, s, side, cz):
    """밀린 직사각형 둘레 위의 점. side 0..3 (±u 변, ±v 변), s ∈ [-1, 1] = 변을 따라 (모서리 cz 는 뺌)."""
    A, B = a + du, b + dv
    sg = np.where(side % 2 == 0, 1.0, -1.0)
    on_u = side < 2                                   # ±u 변 (짧은 변, 길이 2B)
    pu = np.where(on_u, sg * A, s * max(a - cz, 0.0))
    pv = np.where(on_u, s * max(b - cz, 0.0), sg * B)
    ux, uy = np.cos(th), np.sin(th)
    return cx + pu * ux - pv * uy, cy + pu * uy + pv * ux


class BayesSearcher(CubeSearcher):
    """CubeSearcher 와 같은 io/cfg/결과 형식. run() 만 다르다."""

    def __init__(self, io, cfg, log, rng=None):
        super().__init__(io, cfg, log)
        self.rng = rng or np.random.default_rng()
        self.obs = []                  # (x, y, 있음 bool, 가장자리 애매함 σ[mm])

    # ── 관측 모델 ─────────────────────────────────────────────────────
    def _p_present(self, px, py, X, sigma):
        """후보 X (열: cx, cy, th, du, dv) 가 맞을 때 (px, py) 터치가 '있음' 일 확률."""
        c = self.cfg
        a, b = c['cube']['long'] / 2, c['cube']['short'] / 2
        args = (px, py, X[..., 0], X[..., 1], X[..., 2], X[..., 3], X[..., 4], a, b)
        d = sdf(*args)
        sg = np.where(corner_zone(*args, c['bayes_corner_mm']), np.maximum(sigma, c['bayes_sigma_corner']), sigma)
        eps = c['bayes_flip']
        return eps + (1 - 2 * eps) / (1 + np.exp(np.clip(d / sg, -50, 50)))

    def _loglik(self, X, obs=None):
        ll = np.zeros(len(X))
        for (x, y, hit, sg) in (self.obs if obs is None else obs):
            p = self._p_present(x, y, X, sg)
            ll += np.log(p if hit else 1 - p)
        return ll

    def _add_obs(self, x, y, hit, sigma):
        self.obs.append((float(x), float(y), bool(hit), float(sigma)))
        p = self._p_present(x, y, self.X, sigma)
        inc = np.log(p if hit else 1 - p)
        self.logw += inc
        self.ll += inc

    # ── 사전 분포 (거친 격자에서 처음 닿은 점 p0 주변) ────────────────
    def _offset0(self, th):
        """각도 th 일 때 변이 밀려 보이는 양의 예상값 (설정 finger_ex/ey 로 만든 손가락 끝 직사각형 기준)."""
        c = self.cfg
        ux, uy = np.abs(np.cos(th)), np.abs(np.sin(th))
        return c['finger_ex'] * ux + c['finger_ey'] * uy, c['finger_ex'] * uy + c['finger_ey'] * ux

    def _logprior(self, X):
        """du, dv 는 그리퍼마다 거의 같은 물리량 → 예상값 주변으로 약하게 묶는다 (나머지는 균등)."""
        du0, dv0 = self._offset0(X[:, 2])
        sd = self.cfg['bayes_offset_sd']
        return -0.5 * (((X[:, 3] - du0) / sd) ** 2 + ((X[:, 4] - dv0) / sd) ** 2)

    def _prior_ok(self, X):
        c = self.cfg
        lo, hi = c['bayes_offset_min'], c['finger_max']
        ok = (X[:, 3] >= lo) & (X[:, 3] <= hi) & (X[:, 4] >= lo) & (X[:, 4] <= hi)
        ok &= np.hypot(X[:, 0] - self.p0[0], X[:, 1] - self.p0[1]) <= self.r0
        return ok

    def _init_particles(self):
        c = self.cfg
        n = c['bayes_particles']
        a, b = c['cube']['long'] / 2, c['cube']['short'] / 2
        self.r0 = math.hypot(a, b) + c['finger_max']
        r = self.r0 * np.sqrt(self.rng.random(n))
        ang = self.rng.random(n) * 2 * math.pi
        X = np.empty((n, 5))
        X[:, 0] = self.p0[0] + r * np.cos(ang)
        X[:, 1] = self.p0[1] + r * np.sin(ang)
        X[:, 2] = self.rng.random(n) * math.pi
        du0, dv0 = self._offset0(X[:, 2])            # 사전 분포에서 뽑는다 (가중치는 터치 기록만)
        lo, hi = c['bayes_offset_min'], c['finger_max']
        X[:, 3] = np.clip(du0 + self.rng.normal(0, c['bayes_offset_sd'], n), lo, hi)
        X[:, 4] = np.clip(dv0 + self.rng.normal(0, c['bayes_offset_sd'], n), lo, hi)
        self.X = X
        self.ll = self._loglik(X)
        self.logw = self.ll.copy()
        # 거친 격자 정보만으로는 후보가 너무 퍼져 있다 → 바로 다시 뽑아 고르게
        self._resample_move()

    # ── 통계 ──────────────────────────────────────────────────────────
    def _w(self):
        w = np.exp(self.logw - self.logw.max())
        return w / w.sum()

    def _stats(self):
        w, X = self._w(), self.X
        mx, my = w @ X[:, 0], w @ X[:, 1]
        s2, c2 = w @ np.sin(2 * X[:, 2]), w @ np.cos(2 * X[:, 2])
        mth = (math.atan2(s2, c2) / 2) % math.pi
        dth = (X[:, 2] - mth + math.pi / 2) % math.pi - math.pi / 2
        mex, mey = w @ X[:, 3], w @ X[:, 4]
        sx = math.sqrt(max(0.0, w @ (X[:, 0] - mx) ** 2))
        sy = math.sqrt(max(0.0, w @ (X[:, 1] - my) ** 2))
        sth = math.degrees(math.sqrt(max(0.0, w @ dth ** 2)))
        return {'cx': mx, 'cy': my, 'th': mth, 'ex': mex, 'ey': mey, 'sx': sx, 'sy': sy, 'sth': sth,
                'sex': math.sqrt(max(0.0, w @ (X[:, 3] - mex) ** 2)),
                'sey': math.sqrt(max(0.0, w @ (X[:, 4] - mey) ** 2))}

    def _ess(self):
        w = self._w()
        return 1.0 / float(w @ w)

    # ── 다시 뽑기 + MCMC 이동 (해상도 유지) ───────────────────────────
    def _resample_move(self):
        c = self.cfg
        n = len(self.X)
        w = self._w()
        pos = (self.rng.random() + np.arange(n)) / n                 # 계통 재표본
        idx = np.minimum(np.searchsorted(np.cumsum(w), pos), n - 1)
        X, ll = self.X[idx].copy(), self.ll[idx].copy()
        lp = self._logprior(X)
        # 지금 후보들의 퍼짐에 맞춘 크기로 흔든다 (메트로폴리스-헤이스팅스: 전체 터치 기록으로 받아들일지 판단)
        mean = X.mean(0)
        dth = (X[:, 2] - X[0, 2] + math.pi / 2) % math.pi - math.pi / 2
        sd = np.array([X[:, 0].std(), X[:, 1].std(), dth.std(), X[:, 3].std(), X[:, 4].std()])
        sd = np.maximum(sd, [0.02, 0.02, math.radians(0.02), 0.02, 0.02])
        del mean
        for _ in range(c['bayes_mcmc_steps']):
            Y = X + self.rng.normal(size=X.shape) * sd * c['bayes_mcmc_scale']
            Y[:, 2] %= math.pi
            ok = self._prior_ok(Y)
            lly = np.full(n, -np.inf)
            lpy = np.zeros(n)
            if ok.any():
                lly[ok] = self._loglik(Y[ok])
                lpy[ok] = self._logprior(Y[ok])
            acc = np.log(self.rng.random(n)) < (lly + lpy - ll - lp)
            X[acc], ll[acc], lp[acc] = Y[acc], lly[acc], lpy[acc]
        self.X, self.ll, self.logw = X, ll, np.zeros(n)

    # ── 다음 터치 고르기 ──────────────────────────────────────────────
    def _next_point(self):
        """정보 이득(상호 정보량)이 가장 큰 터치 점과 그 이득[bit]."""
        c = self.cfg
        a, b = c['cube']['long'] / 2, c['cube']['short'] / 2
        w = self._w()
        k = c['bayes_candidates']
        src = self.X[self.rng.choice(len(self.X), size=k, p=w)]
        side = self.rng.integers(0, 4, k)
        sv = self.rng.uniform(-1, 1, k)
        px, py = boundary_point(src[:, 0], src[:, 1], src[:, 2], src[:, 3], src[:, 4], a, b, sv, side,
                                c['bayes_corner_mm'])
        px = px + self.rng.normal(0, c['bayes_sigma'], k)
        py = py + self.rng.normal(0, c['bayes_sigma'], k)
        m = c['edge_margin']
        ok = np.array([self.in_area(x, y, m) for x, y in zip(px, py)])
        if not ok.any():
            return None, 0.0
        px, py = px[ok], py[ok]
        ev = self.X[self.rng.choice(len(self.X), size=c['bayes_eval_particles'], p=w)]
        L1 = self._p_present(px[:, None], py[:, None], ev[None, :, :], c['bayes_sigma'])   # (후보, 후보 파티클)
        ig = _h2(L1.mean(1)) - _h2(L1).mean(1)
        i = int(np.argmax(ig))
        return (float(px[i]), float(py[i])), float(ig[i])

    def _converged(self, st):
        c = self.cfg
        return max(st['sx'], st['sy']) <= c['bayes_pos_std'] and st['sth'] <= c['bayes_ang_std']

    # ── 전체 ──────────────────────────────────────────────────────────
    def _localize(self):
        """거친 격자 → 확률 탐색. 끝내는 조건(bayes_pos_std/ang_std)까지 좁힌 후보 통계를 돌려준다."""
        c = self.cfg
        hit = self.coarse()
        if hit is None:
            raise SearchError('영역 전체를 터치했지만 큐브가 없습니다.')
        x0, y0, self.z_top = hit
        self.p0 = (x0, y0)
        # 거친 격자 기록 → 관측. 못 닿음 = 없음, 스침 = 정보 없음(버림), 닿음 = 있음 (깊이 눌러서라 애매함을 크게)
        z_to = c['z_floor'] + c['cube']['h_min']
        for t in self.touches:
            if t['kind'] != 'coarse':
                continue
            if t['z'] is None:
                self.obs.append((t['x'], t['y'], False, c['bayes_sigma']))
            elif t['z'] >= z_to + c['coarse_min_rise']:
                self.obs.append((t['x'], t['y'], True, c['bayes_sigma_coarse']))
        self.log(f'[2단계] 확률 탐색 (파티클 {c["bayes_particles"]}개, 목표 중심 ±{c["bayes_pos_std"]} mm, '
                 f'각도 ±{c["bayes_ang_std"]}°)')
        self._init_particles()
        top_done = False
        n_press = 0
        st = self._stats()
        while True:
            if self._converged(st):
                break
            if n_press >= c['bayes_max_touches']:
                self.log(f'  최대 터치 {c["bayes_max_touches"]}번 도달')
                break
            if not top_done and max(st['sx'], st['sy']) < c['bayes_top_after']:
                # 중심 근처에서 윗면을 다시 재 누르는 깊이를 맞춘다 (처음 닿은 점이 가장자리였을 수 있음)
                z = self.measure_top(st['cx'], st['cy'])
                top_done = True
                if z is not None:
                    self.z_top = z
                    self._add_obs(st['cx'], st['cy'], True, c['bayes_sigma_coarse'])
                    continue
            p, ig = self._next_point()
            if p is None or ig < c['bayes_ig_min']:
                self.log(f'  더 터치해도 얻을 정보가 없음 (이득 {ig:.3f} bit)')
                break
            hit = self.present(*p)
            n_press += 1
            self._add_obs(p[0], p[1], hit, c['bayes_sigma'])
            if self._ess() < c['bayes_ess_frac'] * len(self.X):
                self._resample_move()
            st = self._stats()
            self.log(f'  {n_press:2d}: ({p[0]:.1f}, {p[1]:.1f}) {"있음" if hit else "없음"} '
                     f'(이득 {ig:.2f} bit) → 중심 ({st["cx"]:.1f}, {st["cy"]:.1f}) ±{max(st["sx"], st["sy"]):.2f} mm, '
                     f'각도 {math.degrees(st["th"]):.1f} ±{st["sth"]:.2f}°')
        return st

    def run(self):
        return self._finish(self._localize())

    def _finish(self, st):
        c = self.cfg
        a, b = c['cube']['long'] / 2, c['cube']['short'] / 2
        warn = []
        if not self._converged(st):
            spread = max(st['sx'], st['sy'])
            msg = (f'목표 정확도에 못 미침: 중심 ±{spread:.2f} mm, 각도 ±{st["sth"]:.2f}° '
                   '(큐브가 영역 경계에 걸렸거나 밀렸을 수 있음)')
            if spread > c['bayes_fail_std'] or st['sth'] > 10 * c['bayes_ang_std']:
                raise SearchError(msg)
            warn.append(msg)
        # 모델과 안 맞는 터치 (큐브 크기 설정 틀림, 큐브 2개, 밀림 등)
        X = np.array([[st['cx'], st['cy'], st['th'], st['ex'], st['ey']]])
        bad = 0
        for (x, y, hit, sg) in self.obs:
            if sg != c['bayes_sigma'] or corner_zone(x, y, *X[0], a, b, c['bayes_corner_mm']):
                continue                              # 거친 격자/윗면 기록과 모서리 근처는 원래 애매하다
            d = float(sdf(x, y, *X[0], a, b))
            if (hit and d > c['bayes_bad_mm']) or (not hit and d < -c['bayes_bad_mm']):
                bad += 1
        if bad > c['bayes_bad_max']:
            raise SearchError(f'터치 {bad}개가 {2 * a:.0f}×{2 * b:.0f} 큐브 모양과 맞지 않습니다. '
                              '큐브 크기 설정, 큐브가 2개 붙어 있는지, 큐브가 밀렸는지 확인하세요.')
        if bad:
            warn.append(f'모양과 안 맞는 터치 {bad}개 (가장자리 애매함 이상)')
        z = self.measure_top(st['cx'], st['cy'])
        z_top = z if z is not None else self.z_top
        if z is None:
            self.log('  중심에서 윗면이 안 잡힘 → 이전 높이를 씀 (주의)')
        th = st['th']
        fp_l, fp_s = st['ex'], st['ey']               # 긴 축 / 짧은 축 방향으로 변이 밀려 보이는 양
        hits = [[x, y] for (x, y, h, sg) in self.obs if h and sg == c['bayes_sigma']]
        self.ex, self.ey = st['ex'], st['ey']
        return {'cx': st['cx'], 'cy': st['cy'], 'theta': wrap180(math.degrees(th)), 'z_top': z_top,
                'height': z_top - c['z_floor'], 'span_long': 2 * (a + fp_l), 'span_short': 2 * (b + fp_s),
                'fp_long': fp_l, 'fp_short': fp_s, 'finger': [st['ex'], st['ey']], 'edge_parallel_deg': 0.0,
                'round1': {}, 'edges2': hits or [[st['cx'], st['cy']]], 'method': 'bayes',
                'std': {'cx': st['sx'], 'cy': st['sy'], 'theta_deg': st['sth'], 'ex': st['sex'], 'ey': st['sey']},
                'touches': len([t for t in self.touches if t['kind'] != 'outside']), 'warnings': warn}


class HybridSearcher(BayesSearcher):
    """대략은 확률 탐색(터치 적음), 마무리는 기존 2차 정밀(변에 수직 이분 탐색, 실물에서 정확도 검증됨) 한 번.

    전역 위치 찾기와 국소 정밀 측정을 나누는 방식 (Petrovskaya & Khatib 2011 의 global → local 과 같은 구조).
    확률 탐색은 실물에서 ±1~2 mm 까지는 잘 좁혔지만 마지막 0.5 mm 는 가장자리 모델(애매함 폭, 모서리)에 기대므로,
    마지막은 모델 가정 없이 변 위 점만 직접 재는 2차 정밀로 끝낸다. 2차 정밀은 재는 선이 변에 조금 비스듬해도
    찾은 점은 변 위의 점이라, 같은 변 두 점의 각도와 마주 보는 변의 중간은 그대로 정확하다.
    """

    def h_fp(self, n):
        """변이 밀려 보이는 양: 확률 탐색이 잰 du(긴 축 방향 변), dv(짧은 축 방향 변)."""
        if self._u is None:
            return super().h_fp(n)
        return self._du if abs(n[0] * self._u[0] + n[1] * self._u[1]) > 0.7 else self._dv

    def run(self):
        self._u = None
        st = self._localize()
        th = st['th']
        self._u = (math.cos(th), math.sin(th))
        self._du, self._dv = max(0.0, st['ex']), max(0.0, st['ey'])
        self.ex, self.ey = self._du, self._dv
        est = {'cx': st['cx'], 'cy': st['cy'], 'th': wrap180(math.degrees(th))}
        self.log(f'  → 대략 위치 ({est["cx"]:.1f}, {est["cy"]:.1f}) ±{max(st["sx"], st["sy"]):.1f} mm, '
                 f'각도 {est["th"]:.1f} ±{st["sth"]:.1f}° → 변에 수직 이분 탐색으로 마무리')
        res = self.refine(est)
        res['method'] = 'hybrid'
        res['round1'] = {'cx': st['cx'], 'cy': st['cy'], 'th': est['th'], 'std_mm': max(st['sx'], st['sy']),
                         'std_deg': st['sth']}
        return res
