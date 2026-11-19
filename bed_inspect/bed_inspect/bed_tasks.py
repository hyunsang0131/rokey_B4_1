# 공정에서 쓰는 동작 단위. 파이프라인의 각 단계가 여기 함수 하나에 대응한다.
#
#   press(i)        i번 칸을 눌러서 강성과 표면 높이를 잰다     (3, 11단계)
#   roller(+1/-1)   롤러를 잡고 돌려 천을 팽팽하게 / 느슨하게    (2, 5, 10단계)
#   cover_open()    커버를 잡고 원호로 연다                     (6단계)
#   cover_close()   커버를 잡고 원호로 닫는다                   (9단계)
#   move_block(mv)  블록 하나를 집어서 다른 자리에 놓는다        (7, 8단계)

from bed_inspect import bed_config as cfg
from bed_inspect import bed_math
from bed_inspect.bed_robot import BedError


class BedTasks:
    def __init__(self, robot, points, log):
        self.r = robot
        self.pt = points            # {이름: 교시 위치}
        self.log = log
        from bed_inspect import bed_emu
        robot.emu = bed_emu.from_env(points, log)      # 에뮬레이터용 가짜 힘 (환경변수가 있을 때만)

    # ── 공통 ──────────────────────────────────────────────────────────
    def home(self):
        self.r.movej_to(self.pt[cfg.P_HOME])

    def _approach(self, name):
        """접근점(_app)을 거쳐 잡는 자세로 천천히 들어간다."""
        self.r.movej_to(self.pt[name + cfg.APP_SUFFIX])
        self.r.movej_to(self.pt[name], slow=True)

    def _retreat(self, name):
        self.r.movej_to(self.pt[name + cfg.APP_SUFFIX], slow=True)

    # ── 누르기 측정 ───────────────────────────────────────────────────
    def press(self, i, height):
        """i번 칸(0부터)을 누르고 결과 dict 를 돌려준다. 실패해도 예외 대신 'error' 에 이유를 적는다.

        순서: 칸 위로 이동 → 빠르게 접근 → 힘 영점 → 조금씩 내려가며 (높이, 힘) 기록
              → 목표 힘에서 멈춤 → 교시 높이로 복귀 → 직선 맞춤으로 강성 계산
        """
        res = self.press_at(self.pt[cfg.P_BED[i]], cfg.PRESS_APPROACH_DZ)
        res.update(slot=i + 1, height=height)
        return res

    def press_rack(self, j):
        """보관대 j번 자리(0부터)의 큐브를 누른다 (빈자리 채우기에서 후보 고르기용)."""
        res = self.press_at(self.pt[cfg.P_RACK[j]], cfg.RACK_PRESS_APPROACH_DZ)
        res.update(slot=j + 1)
        return res

    def touch_rack(self, j):
        """보관대 j번 자리(0부터)의 큐브에 살짝 닿기만 해서 윗면 높이를 잰다 (높은 큐브 찾기용)."""
        res = self.touch_at(self.pt[cfg.P_RACK[j]], cfg.RACK_PRESS_APPROACH_DZ)
        res.update(slot=j + 1)
        return res

    def touch_at(self, point, approach_dz):
        """교시 위치에서 내려가다가 힘이 '노이즈 경계'를 연속으로 넘으면 바로 멈추고 그 높이를 기록한다.

        경계 = max(TOUCH_F_MIN, TOUCH_SIGMA × 영점 구간의 표준편차). 누르지 않으므로 강성은 없다.
        반환: {'z0': 닿은 높이, 'thr': 쓴 경계[N], 'sigma': 노이즈[N], 'error', 'note', 'k': None, ...}
        """
        r = self.r
        res = {'slot': None, 'height': None, 'k': None, 'z_surf': None, 'r2': None, 'z0': None, 'disp': None,
               'f_max': None, 'error': None, 'note': '', 'curve': [], 'thr': None, 'sigma': None, 'touch': True}
        r.grip('press')
        r.movej_to(point)
        z_top = r.cur_z()
        try:
            r.move_z(approach_dz)
            r.sleep(cfg.PRESS_SETTLE)
            vals = r.read_fz_list(max(cfg.TARE_SAMPLES, 5))
            tare = sum(vals) / len(vals)
            sigma = (sum((v - tare) ** 2 for v in vals) / (len(vals) - 1)) ** 0.5
            thr = max(cfg.TOUCH_F_MIN, cfg.TOUCH_SIGMA * sigma)
            res.update(thr=thr, sigma=sigma)
            z_start = r.cur_z()
            first, count = None, 0
            while True:
                r.move_z(-cfg.SEARCH_STEP, cfg.STEP_VEL, cfg.STEP_ACC)
                r.sleep(cfg.STEP_SETTLE)
                f = abs(r.read_fz(cfg.READ_SAMPLES) - tare)
                z = r.cur_z()
                res['curve'].append((z, f))
                if f >= cfg.F_LIMIT:
                    res['error'], res['note'] = 'over_force', f'힘 한계 {cfg.F_LIMIT:.0f}N 초과'
                    break
                if f >= thr:
                    first = z if first is None else first
                    count += 1
                    if count >= cfg.TOUCH_CONFIRM:
                        res['z0'] = first
                        break
                else:
                    first, count = None, 0          # 한 번 튄 값이면 처음부터 다시 센다
                if z_start - z >= cfg.SEARCH_MAX:
                    res['error'], res['note'] = 'no_contact', f'{cfg.SEARCH_MAX:.0f}mm 내려가도 닿지 않음 (블록 없음?)'
                    break
        finally:
            self._lift_back(z_top)
        res['f_max'] = max((f for _, f in res['curve']), default=0.0)
        return res

    def press_at(self, point, approach_dz):
        """교시 위치 point 에서 approach_dz 만큼 빠르게 내려온 뒤 누르기 측정을 한다."""
        r = self.r
        res = {'slot': None, 'height': None, 'k': None, 'z_surf': None, 'r2': None,
               'z0': None, 'disp': None, 'f_max': None, 'error': None, 'note': '', 'curve': []}
        r.grip('press')
        r.movej_to(point)
        r.sleep(cfg.PRESS_SETTLE)           # 막 멈춘 직후의 흔들림이 힘 추정에 섞이지 않게 잠깐 쉰다
        z_top = r.cur_z()
        tare_top = r.read_fz(cfg.TARE_SAMPLES)

        raw = []                    # (높이, 영점을 뺀 힘 - 부호 있음)
        z0 = None
        try:
            r.move_z(approach_dz)
            r.sleep(cfg.PRESS_SETTLE)
            tare = r.read_fz(cfg.TARE_SAMPLES)
            z_start = r.cur_z()
            if abs(tare - tare_top) >= cfg.F_PRETOUCH:
                # 접근만 했는데 힘이 변했다 = 이미 천에 닿아 있다 → 영점이 틀리므로 측정하지 않는다
                res['error'] = 'touch_at_start'
                res['note'] = (f'접근 높이에서 이미 {abs(tare - tare_top):.1f}N 이 걸립니다. '
                               'PRESS_APPROACH_DZ 를 줄이세요.')
                return res

            if cfg.PROBE_VEL > 0:
                # 빠른 탐색: 멈추지 않고 SEARCH_MAX 까지 내려가며 힘을 본다
                hit, z_hit, f_peak = r.probe_down(cfg.SEARCH_MAX, cfg.PROBE_VEL, cfg.PROBE_ACC, tare, cfg.F_CONTACT)
                res['probe'] = {'hit': hit, 'z': z_hit, 'f_peak': f_peak}
                if not hit:
                    raw.append((z_hit, r.read_fz(cfg.READ_SAMPLES) - tare))
                    res['error'] = 'no_contact'
                    res['note'] = f'{cfg.SEARCH_MAX:g}mm 내려가도 닿지 않음 (빠른 탐색, 최대 {f_peak:.1f}N)'
                    return res
                # 닿았다(또는 잘못 감지) → 조금 올라가서 지금까지와 같은 계단식으로 접촉점부터 다시 잰다
                r.move_z(min(cfg.PROBE_BACK, z_start - z_hit))
                r.sleep(cfg.STEP_SETTLE)

            while True:
                step = cfg.PRESS_STEP if z0 is not None else cfg.SEARCH_STEP
                r.move_z(-step, cfg.STEP_VEL, cfg.STEP_ACC)
                r.sleep(cfg.STEP_SETTLE)
                df = r.read_fz(cfg.READ_SAMPLES) - tare
                z = r.cur_z()
                raw.append((z, df))
                f = abs(df)
                if z0 is None and f >= cfg.F_CONTACT:
                    z0 = z
                if f >= cfg.F_LIMIT:
                    res['error'], res['note'] = 'over_force', f'힘 한계 {cfg.F_LIMIT:.0f}N 초과'
                    break
                if z0 is None and z_start - z >= cfg.SEARCH_MAX:
                    res['error'], res['note'] = 'no_contact', f'{cfg.SEARCH_MAX:.0f}mm 내려가도 닿지 않음 (블록 없음?)'
                    break
                if z0 is not None and z0 - z >= cfg.D_MAX:
                    res['error'], res['note'] = 'too_deep', f'{cfg.D_MAX:.0f}mm 넘게 들어감 (블록이 너무 무름)'
                    break
                if z0 is not None and f >= cfg.F_TARGET:
                    break
        finally:
            # 정상이든 오류든 정지든, 누른 채로 두지 않고 교시 높이로 올라온다
            self._lift_back(z_top)

        # 힘의 부호: 로봇 설정에 따라 누를 때 +일 수도 -일 수도 있어서, 가장 큰 값의 부호를 기준으로 한다
        sign = 1.0 if max(raw, key=lambda p: abs(p[1]))[1] >= 0 else -1.0
        curve = [(z, sign * df) for z, df in raw]
        res['curve'] = curve
        res['f_max'] = max(f for _, f in curve)
        if z0 is not None:
            res['z0'] = z0
            res['disp'] = z0 - curve[-1][0]
        if res['error']:
            return res

        fit = bed_math.fit_stiffness(curve, cfg.F_FIT_MIN, cfg.MIN_FIT_POINTS)
        if fit is None or fit['r2'] < cfg.MIN_R2:
            res['error'] = 'bad_fit'
            res['note'] = '힘-변위가 직선이 아님 (다시 측정 필요)' if fit else '측정 점이 모자람'
            if fit:
                res['r2'] = fit['r2']
            return res
        res.update(k=fit['k'], z_surf=fit['z_surf'], r2=fit['r2'])
        return res

    def _lift_back(self, z_top):
        """교시 높이까지 올라온다. 정지 요청이 걸려 있어도 이것만은 실행한다."""
        ev = self.r.stop_event
        was_set = ev is not None and ev.is_set()
        if was_set:
            ev.clear()
        try:
            self.r.move_z(z_top - self.r.cur_z())
        except BedError as e:
            self.log(f'누른 뒤 올라오지 못했습니다: {e}')
            if not was_set:
                raise
        finally:
            if was_set:
                ev.set()

    # ── 롤러 ──────────────────────────────────────────────────────────
    def roller(self, direction):
        """direction=+1 팽팽하게, -1 느슨하게. 잡고 돌리고 놓기를 ROLLER_TURNS 번 반복(래칫 방식)."""
        r = self.r
        deg = direction * cfg.ROLLER_TURN_DEG
        r.grip('roller_open')
        self._approach(cfg.P_ROLLER)
        for n in range(cfg.ROLLER_TURNS):
            r.grip('roller_grip')
            r.turn_j6(deg, cfg.ROLLER_VEL, cfg.ROLLER_ACC)
            r.grip('roller_open')
            if n < cfg.ROLLER_TURNS - 1:
                r.turn_j6(-deg, cfg.ROLLER_VEL, cfg.ROLLER_ACC)     # 빈손으로 되돌아가 다시 잡는다
        self._retreat(cfg.P_ROLLER)

    # ── 커버 ──────────────────────────────────────────────────────────
    def cover_open(self):
        self._cover(cfg.P_COVER_GRIP, cfg.P_COVER_OPEN)

    def cover_close(self):
        self._cover(cfg.P_COVER_OPEN, cfg.P_COVER_GRIP)

    def _cover(self, start, end):
        r = self.r
        r.grip('cover_open')
        self._approach(start)
        r.grip('cover_grip')
        r.movec(self.pt[cfg.P_COVER_VIA], self.pt[end], cfg.COVER_VEL, cfg.COVER_ACC)
        r.grip('cover_open')
        self._retreat(end)

    # ── 블록 옮기기 ───────────────────────────────────────────────────
    def _station(self, place):
        """('bed'|'rack'|'discard', 번호) → (교시 위치, 기준 높이 블록을 잡으러 내려가는 양)."""
        kind, idx = place
        if kind == 'bed':
            return self.pt[cfg.P_BED[idx]], cfg.BED_PICK_DZ
        if kind == 'rack':
            return self.pt[cfg.P_RACK[idx]], cfg.RACK_PICK_DZ
        return self.pt[cfg.P_DISCARD], None

    def move_block(self, mv):
        """이동 하나 실행: mv = {'src': (종류, 번호), 'dst': (종류, 번호), 'h': 블록 높이}."""
        r = self.r
        h = mv['h']
        # 블록이 기준 높이보다 높으면 그만큼 덜 내려가야 같은 깊이로 잡힌다
        extra = h - cfg.H_REF

        point, dz = self._station(mv['src'])
        r.grip('block_open')
        r.movej_to(point)
        down = dz + extra
        r.move_z(down, cfg.VELX_SLOW, cfg.ACCX_SLOW)
        r.grip('block_grip')
        r.move_z(-down)

        point, dz = self._station(mv['dst'])
        r.movej_to(point)
        if dz is None:                      # 버리는 통: 위에서 그냥 놓는다
            r.grip('block_open')
            return
        down = dz + extra + cfg.PLACE_CLEAR
        r.move_z(down, cfg.VELX_SLOW, cfg.ACCX_SLOW)
        r.sleep(cfg.PLACE_SETTLE)           # 매달린 블록이 흔들리는 채로 놓으면 칸 안에서 틀어진다
        r.grip('block_open')
        r.move_z(-down)
