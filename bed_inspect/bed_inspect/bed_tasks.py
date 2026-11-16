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

    def press_at(self, point, approach_dz):
        """교시 위치 point 에서 approach_dz 만큼 빠르게 내려온 뒤 누르기 측정을 한다."""
        r = self.r
        res = {'slot': None, 'height': None, 'k': None, 'z_surf': None, 'r2': None,
               'z0': None, 'disp': None, 'f_max': None, 'error': None, 'note': '', 'curve': []}
        r.grip('press')
        r.movej_to(point)
        z_top = r.cur_z()
        tare_top = r.read_fz(cfg.READ_SAMPLES)

        raw = []                    # (높이, 영점을 뺀 힘 - 부호 있음)
        z0 = None
        try:
            r.move_z(approach_dz)
            r.sleep(cfg.PRESS_SETTLE)
            tare = r.read_fz(cfg.TARE_SAMPLES)
            z_start = r.cur_z()
            if abs(tare - tare_top) >= cfg.F_CONTACT:
                # 접근만 했는데 힘이 변했다 = 이미 천에 닿아 있다 → 영점이 틀리므로 측정하지 않는다
                res['error'] = 'touch_at_start'
                res['note'] = (f'접근 높이에서 이미 {abs(tare - tare_top):.1f}N 이 걸립니다. '
                               'PRESS_APPROACH_DZ 를 줄이세요.')
                return res

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
        r.grip('block_open')
        r.move_z(-down)
