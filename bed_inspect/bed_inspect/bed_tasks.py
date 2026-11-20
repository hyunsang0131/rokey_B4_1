# 공정에서 쓰는 동작 단위. 파이프라인의 각 단계가 여기 함수 하나에 대응한다.
#
#   press(i)        i번 칸을 눌러서 강성과 표면 높이를 잰다     (2, 8단계)
#   cover_open()    고리를 위에서 수직으로 잡고 원호 + y·z 직선으로 열어 받침에 걸친다  (4단계)
#   cover_close()   받침에 걸친 고리를 잡고 같은 길을 거꾸로 닫는다                 (7단계)
#   move_block(mv)  블록 하나를 집어서 다른 자리에 놓는다        (5, 6단계)

from bed_inspect import bed_config as cfg
from bed_inspect import bed_math
from bed_inspect.bed_robot import BedError


def make_cover_path():
    """bed_config 의 COVER_* 값으로 커버 좌표를 계산한다 (설정이 이상하면 ValueError)."""
    if cfg.COVER_MODE == 'one_arc':
        lid_len = cfg.COVER_LID_LEN
        if lid_len and cfg.COVER_HOOK_AT_MIDDLE:
            lid_len = max(lid_len, 2.0 * cfg.COVER_TAUGHT_RADIUS)
        return bed_math.cover_path_one_arc(cfg.COVER_ONE_START_POSX, cfg.COVER_ONE_END_POSX, cfg.COVER_TAUGHT_RADIUS,
                                           cfg.COVER_ABOVE_DZ, cfg.COVER_RETREAT,
                                           close_margin_deg=cfg.COVER_CLOSE_MARGIN_DEG,
                                           lid_len=lid_len, exit_clear=cfg.COVER_EXIT_CLEAR,
                                           keep_vertical=cfg.COVER_KEEP_VERTICAL, exit_z_min=cfg.COVER_EXIT_Z_MIN)
    if cfg.COVER_MODE == 'taught':
        return bed_math.cover_path_taught(list(cfg.COVER_GRIP_XYZ) + list(cfg.COVER_GRIP_ORI), cfg.COVER_LIFT_POSX,
                                          cfg.COVER_END_POSX, cfg.COVER_ABOVE_DZ, cfg.COVER_RETREAT,
                                          inset=cfg.COVER_ARC2_INSET, close_margin_deg=cfg.COVER_CLOSE_MARGIN_DEG,
                                          lid_len=cfg.COVER_LID_LEN, exit_clear=cfg.COVER_EXIT_CLEAR,
                                          radius=cfg.COVER_TAUGHT_RADIUS)
    if cfg.COVER_MODE == 'delta':
        return bed_math.cover_path_delta(cfg.COVER_DEMO_START, cfg.COVER_DEMO_VIA, cfg.COVER_DEMO_END,
                                         list(cfg.COVER_GRIP_XYZ) + list(cfg.COVER_GRIP_ORI), cfg.COVER_ARC_RADIUS,
                                         cfg.COVER_ABOVE_DZ, cfg.COVER_RETREAT, rest_deg=cfg.COVER_REST_DEG,
                                         close_margin_deg=cfg.COVER_CLOSE_MARGIN_DEG, keep_x=cfg.COVER_KEEP_X,
                                         min_open_deg=cfg.COVER_MIN_OPEN_DEG, lid_len=cfg.COVER_LID_LEN,
                                         exit_clear=cfg.COVER_EXIT_CLEAR, arc_deg=cfg.COVER_ARC_DEG, via_dir_deg=cfg.COVER_VIA_DIR_DEG,
                                         end_dir_deg=cfg.COVER_END_DIR_DEG, single_arc=cfg.COVER_SINGLE_ARC,
                                         hinge_rel=cfg.COVER_HINGE_REL,
                                         end_deg_on_circle=cfg.COVER_END_DEG)
    return bed_math.cover_path(cfg.COVER_DEMO_START, cfg.COVER_DEMO_VIA, cfg.COVER_DEMO_END,
                               cfg.COVER_GRIP_XYZ, cfg.COVER_GRIP_ORI, cfg.COVER_ABOVE_DZ, cfg.COVER_RETREAT,
                               rest_deg=cfg.COVER_REST_DEG or 110.0, line_segments=cfg.COVER_LINE_SEGMENTS,
                               close_margin_deg=cfg.COVER_CLOSE_MARGIN_DEG,
                               min_open_deg=cfg.COVER_MIN_OPEN_DEG, max_axial_offset=cfg.COVER_MAX_AXIAL_OFFSET,
                               max_grip_angle=cfg.COVER_MAX_GRIP_ANGLE, hinge_at_grip=cfg.COVER_HINGE_AT_GRIP)


class BedTasks:
    def __init__(self, robot, points, log):
        self.r = robot
        self.pt = points            # {이름: 교시 위치}
        self.log = log
        try:
            self.cover = make_cover_path()
        except ValueError as e:
            raise BedError(f'커버 좌표 설정 오류: {e}')
        from bed_inspect import bed_emu
        robot.emu = bed_emu.from_env(points, log)      # 에뮬레이터용 가짜 힘 (환경변수가 있을 때만)

    # ── 공통 ──────────────────────────────────────────────────────────
    def home(self):
        self.r.movej_to(self.pt[cfg.P_HOME])

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
                # 깊이 한계: 접촉 여부와 상관없이 접근 높이에서 이만큼 넘게 내려가지 않는다 (빈칸 바닥에 닿지 않게)
                if cfg.PRESS_DEPTH_MAX and z_start - z >= cfg.PRESS_DEPTH_MAX:
                    if z0 is None:
                        res['error'], res['note'] = 'no_contact', f'{cfg.PRESS_DEPTH_MAX:g}mm(깊이 한계) 까지 닿지 않음 (빈칸)'
                    else:
                        res['note'] = f'깊이 한계 {cfg.PRESS_DEPTH_MAX:g}mm 에서 멈춤 ({f:.1f}N)'
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

    # ── 커버 ──────────────────────────────────────────────────────────
    # 좌표는 bed_config 의 COVER_* 로 계산한 self.cover (bed_math.cover_path 설명 참고). x 는 처음부터 끝까지 그대로.
    #   열기: 홈 → 고리 위(above) → 수직으로 천천히 내려가 잡기(grip) → 시작점 확인
    #         → [순응 제어 + 외력 감시] 원호 movec(mid, via) → 살며시 잡기 → y·z 직선(line, 마지막이 end) → 놓기(뚜껑이 받침에 걸침)
    #         → 툴 -Z 로 빠지기(end_app) → 홈
    #   닫기: 홈 → rest_app → 툴 +Z 로 들어가 받침에 걸친 고리 살며시 잡기(rest) → 시작점 확인
    #         → [순응 제어 + 외력 감시] y·z 직선(line 거꾸로, via 까지) → 꽉 잡기 → 원호 movec(mid, close_end: 닫히기 2도 전) → 놓기
    #         → 수직으로 올라오기(above) → 홈
    # 뚜껑을 잡고 움직이는 동안 멈추면(외력 초과, 정지 버튼, 오류) 그리퍼를 열지 않는다. 놓으면 뚜껑이 떨어진다.
    def cover_open(self):
        r, p = self.r, self.cover
        for w in p.get('warnings', []):
            self.log(f'커버 좌표 주의: {w}')
        r.grip('cover_open')
        self.home()                                     # 어디서 왔든 홈을 거쳐서 직선으로 접근한다
        r.movel_to(p['above'], cfg.VELX, cfg.ACCX, '커버 고리 위로 이동')
        r.movel_to(p['grip'], cfg.COVER_APP_VEL, cfg.COVER_APP_ACC, '커버 고리로 수직 하강')
        r.grip('cover_grip')
        self._check_at(p['grip'], '커버 열기 시작점')
        f0 = self._arc(p['mid'], p['via'], '커버 열기 원호')
        if 'arc2_mid' in p:                             # 교시 방식: lift 점 → 끝점 두 번째 원호 (경첩 원보다 조금 안쪽)
            f0 = self._arc(p['arc2_mid'], p['end'], '커버 열기 원호 2', f0)
        if p['line']:
            self._loosen()                              # 데모처럼 끝점까지는 살며시 잡는다
        for i, pt in enumerate(p['line'], 1):
            f0 = self._line(pt, f'커버 열기 y·z 직선 {i}/{len(p["line"])}', f0)
        r.grip('cover_open')                            # 놓으면 뚜껑이 베드 옆 받침에 걸친다
        self._exit_move(p['end_app'], cfg.COVER_APP_VEL, cfg.COVER_APP_ACC, '커버 고리에서 빠지기')
        if cfg.COVER_EXIT_POSJ:                         # 교시한 길: 바깥으로 빠져 옆으로 돌아 홈으로
            for i, j in enumerate(cfg.COVER_EXIT_POSJ, 1):
                self.log(f'커버 옆으로 돌아 나가기 {i}/{len(cfg.COVER_EXIT_POSJ)}')
                r.movej_to({'name': f'cover_exit_{i}', 'posj': list(j)})
        elif 'exit_up' in p:                              # 열린 뚜껑에 걸리지 않게 위로 올라가 넘어서 홈으로
            if p['exit_out'][:3] != p['end_app'][:3]:
                self._exit_move(p['exit_out'], cfg.VELX, cfg.ACCX, '열린 커버 끝보다 바깥으로 나가기')
            self._exit_move(p['exit_up'], cfg.VELX, cfg.ACCX, '열린 커버 끝보다 위로 올라가기')
            self._exit_move(p['exit_over'], cfg.VELX, cfg.ACCX, '열린 커버 넘어가기')
        else:
            self.log('커버 우회 경로가 없습니다 (COVER_LID_LEN 이 None) → 바로 홈으로 갑니다')
        self.log('홈으로 이동')
        self.home()

    def _exit_move(self, pose, vel, acc, what):
        """커버에서 빠져나오는 이동: 목표와 끝난 위치를 로그에 남긴다 (실물에서 경로 확인용)."""
        self.log(f'{what}: 목표 {[round(v, 1) for v in pose[:3]]}')
        self.r.movel_to(pose, vel, acc, what)
        try:
            self.log(f'  → 도착 {[round(v, 1) for v in self.r.cur_posx()[:3]]}')
        except Exception:
            pass

    def cover_close(self):
        r, p = self.r, self.cover
        r.grip('cover_open')
        self.home()
        if cfg.COVER_EXIT_POSJ:                         # 교시한 길을 거꾸로: 옆으로 돌아 바깥에서 고리 쪽으로
            for i, j in reversed(list(enumerate(cfg.COVER_EXIT_POSJ, 1))):
                self.log(f'커버 옆으로 돌아 들어가기 {i}/{len(cfg.COVER_EXIT_POSJ)}')
                r.movej_to({'name': f'cover_exit_{i}', 'posj': list(j)})
        elif 'exit_over' in p:                          # 열린 뚜껑 위를 넘어 받침 쪽으로
            r.movel_to(p['exit_over'], cfg.VELX, cfg.ACCX, '열린 커버 위로 이동')
            r.movel_to(p['rest_up'], cfg.VELX, cfg.ACCX, '열린 커버 넘어가기')
        if cfg.COVER_EXIT_POSJ:
            # 바깥 점에서 고리(끝점)까지 비스듬한 채로 바로 직선 (수직으로 내려오지 않음). 아직 안 잡았으니 순응 제어 없이 조금 빠르게
            r.movel_to(p['rest'], cfg.COVER_CLOSE_APP_VEL, cfg.COVER_CLOSE_APP_ACC, '커버 고리로 비스듬히 들어가기')
        else:
            r.movel_to(p['rest_app'], cfg.VELX, cfg.ACCX, '받침에 걸친 커버 고리 앞으로 이동')
            r.movel_to(p['rest'], cfg.COVER_APP_VEL, cfg.COVER_APP_ACC, '커버 고리로 들어가기')
        r.grip('cover_grip' if p.get('close_direct') or p.get('close_taught') or not self._loose_ok() else 'cover_loose')
        self._check_at(p['rest'], '커버 닫기 시작점')
        if p.get('close_taught'):                       # 교시 방식: 원호 2 거꾸로 → 원호 1 거꾸로
            f0 = self._arc(p['arc2_mid'], p['via'], '커버 닫기 원호 2')
            self._arc(p['close_mid'], p['close_end'], '커버 닫기 원호 1', f0)
            r.grip('cover_open')
            r.movel_to(p['above'], cfg.COVER_APP_VEL, cfg.COVER_APP_ACC, '커버 고리에서 수직 상승')
            self.home()
            return
        if p.get('close_direct'):                       # delta 방식: 받침에서 같은 원 위로 원호 한 번에 닫기
            if self._loose_ok():
                r.grip('cover_grip')
            self._arc(p['close_mid'], p['close_end'], '커버 닫기 원호')
            r.grip('cover_open')
            r.movel_to(p['above'], cfg.COVER_APP_VEL, cfg.COVER_APP_ACC, '커버 고리에서 수직 상승')
            self.home()
            return
        back = p['line'][::-1] + [p['via']]            # 받침 → 끝점 → (직선 구간 거꾸로) → 경유점
        f0 = None
        for i, pt in enumerate(back, 1):
            f0 = self._line(pt, f'커버 닫기 y·z 직선 {i}/{len(back)}', f0)
        if self._loose_ok():
            r.grip('cover_grip')                        # 원호에서는 뚜껑 무게가 고리에 걸리므로 다시 꽉 잡는다
        self._arc(p['mid'], p['close_end'], '커버 닫기 원호', f0)
        r.grip('cover_open')                            # 닫히기 2도 전에 놓는다 → 뚜껑이 제 무게로 살짝 내려앉는다
        r.movel_to(p['above'], cfg.COVER_APP_VEL, cfg.COVER_APP_ACC, '커버 고리에서 수직 상승')
        self.home()

    @staticmethod
    def _loose_ok():
        return cfg.COVER_LOOSE_ON_LINE and cfg.GRIPPER_MODE != 'io'

    def _loosen(self):
        if self._loose_ok():
            self.r.grip('cover_loose')
        elif cfg.COVER_LOOSE_ON_LINE:
            self.log("그리퍼가 'io' 모드라 폭을 정할 수 없어 끝점까지 꽉 잡은 채로 갑니다.")

    def _arc(self, mid, end, what, f0=None):
        stx = cfg.COVER_STX if cfg.COVER_COMPLIANCE else None
        peak, f0 = self.r.movec_guarded(mid, end, cfg.COVER_VEL, cfg.COVER_ACC, what, cfg.COVER_F_LIMIT,
                                        compliance=stx, poll=cfg.COVER_POLL, end_tol=cfg.COVER_END_TOL, f0=f0)
        self.log(f'{what}: 최대 외력 {peak:.1f}N (한계 {cfg.COVER_F_LIMIT:.0f}N)')
        return f0

    def _line(self, pose, what, f0=None):
        """뚜껑을 잡은 채 직선 이동 (원호와 같은 외력 감시/순응 제어). 외력 기준값은 앞 구간 것을 이어 쓴다."""
        stx = cfg.COVER_STX if cfg.COVER_COMPLIANCE else None
        peak, f0 = self.r.movel_guarded(pose, cfg.COVER_LINE_VEL, cfg.COVER_LINE_ACC, what, cfg.COVER_F_LIMIT,
                                        compliance=stx, poll=cfg.COVER_POLL, end_tol=cfg.COVER_END_TOL, f0=f0)
        self.log(f'{what}: 최대 외력 {peak:.1f}N (한계 {cfg.COVER_F_LIMIT:.0f}N)')
        return f0

    def _check_at(self, pose, what):
        """movec 는 '지금 위치'를 시작점으로 원을 그린다. 시작점이 어긋나 있으면 엉뚱한 원이 되므로 확인한다."""
        cur = self.r.cur_posx()
        err = sum((a - b) ** 2 for a, b in zip(cur[:3], pose[:3])) ** 0.5
        if err > cfg.COVER_POS_TOL:
            raise BedError(f'{what}에서 {err:.1f}mm 벗어나 있습니다 (허용 {cfg.COVER_POS_TOL:.0f}mm). '
                           f'현재 {[round(v, 1) for v in cur[:3]]}, 목표 {pose[:3]}')

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
        if point.get('down'):               # 팀원 교시 파일: 잡는 자리(_down)와 올라올 자리(_up)를 직접 씀
            self._go_down(point, extra)
            r.grip('block_grip')
            self._go_up(point)
        else:
            down = dz + extra
            r.move_z(down, cfg.VELX_SLOW, cfg.ACCX_SLOW)
            r.grip('block_grip')
            r.move_z(-down)

        point, dz = self._station(mv['dst'])
        r.movej_to(point)
        if dz is None:                      # 버리는 통: 위에서 그냥 놓는다
            r.grip('block_open')
            return
        if point.get('down'):
            self._go_down(point, extra + cfg.PLACE_CLEAR_TEACH)
            r.sleep(cfg.PLACE_SETTLE)
            r.grip('block_open')
            self._go_up(point)
            return
        down = dz + extra + cfg.PLACE_CLEAR
        r.move_z(down, cfg.VELX_SLOW, cfg.ACCX_SLOW)
        r.sleep(cfg.PLACE_SETTLE)           # 매달린 블록이 흔들리는 채로 놓으면 칸 안에서 틀어진다
        r.grip('block_open')
        r.move_z(-down)

    def _go_down(self, point, dz_extra):
        """교시한 _down 자리로 직선 하강 (블록이 기준보다 높으면 그만큼 위에서 멈춤)."""
        p = list(point['down'])
        p[2] += dz_extra
        self.r.movel_to(p, cfg.TEACH_VELZ, cfg.TEACH_ACCZ, f'{point["name"]} 잡는 높이로 내려가기')

    def _go_up(self, point):
        """교시한 _up 자리(없으면 위 자리)로 직선 상승."""
        p = point.get('up') or point['posx']
        self.r.movel_to(list(p), cfg.TEACH_VELZ, cfg.TEACH_ACCZ, f'{point["name"]} 위로 올라오기')
