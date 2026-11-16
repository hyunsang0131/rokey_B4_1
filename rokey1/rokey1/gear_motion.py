"""gear_motion.py - 상대좌표 기반 동작 세트(pick / place / force_insert).

절대 좌표로 가는 것은 '집을 자리 위'로 갈 때 한 번뿐이고,
나머지(하강, 상승, 트레이→조립 지그 이동)는 전부 베이스 기준 상대 이동입니다.

참고: 지금 자세(툴이 아래를 봄)에서는 툴 X = 베이스 -X, 툴 Y = 베이스 +Y, 툴 Z = 베이스 -Z 입니다.
헷갈리지 않도록 평행이동은 베이스 기준으로 통일했습니다. move_rel(..., tool=True) 로 툴 기준도 쓸 수 있습니다.
"""
import time

from rokey1 import gear_config as cfg
from rokey1.gear_gripper import GearError


class GearMotion:
    def __init__(self, dr, gripper, logger):
        self.dr = dr
        self.gripper = gripper
        self.log = logger
        self.dz_test = float(cfg.Z_TEST_OFFSET)

    # ── 기본 이동 ─────────────────────────────────────────────────────
    def _check(self, ret, what):
        if ret != 0:
            raise GearError("%s 실패 (로봇 상태/알람을 확인하세요)" % what)

    def move_abs(self, x, y, z, ori=cfg.TOOL_ORI, fine=False):
        pos = self.dr.posx(x, y, z + self.dz_test, ori[0], ori[1], ori[2])
        kw = {"vel": cfg.VEL_FINE, "acc": cfg.ACC_FINE} if fine else {}
        self._check(self.dr.movel(pos, ref=self.dr.DR_BASE, mod=self.dr.DR_MV_MOD_ABS, **kw),
                    "절대 이동 (%.1f, %.1f, %.1f)" % (x, y, z))

    def move_rel(self, dx=0.0, dy=0.0, dz=0.0, tool=False, fine=False):
        """현재 위치에서 (dx, dy, dz)만큼 평행이동. tool=True 면 툴 좌표계 기준."""
        pos = self.dr.posx(dx, dy, dz, 0.0, 0.0, 0.0)
        ref = self.dr.DR_TOOL if tool else self.dr.DR_BASE
        kw = {"vel": cfg.VEL_FINE, "acc": cfg.ACC_FINE} if fine else {}
        self._check(self.dr.movel(pos, ref=ref, mod=self.dr.DR_MV_MOD_REL, **kw),
                    "상대 이동 (%+.1f, %+.1f, %+.1f)" % (dx, dy, dz))

    def go_home(self):
        h = cfg.HOME
        self.move_abs(h[0], h[1], h[2], ori=h[3:6])

    # ── 동작 세트 ─────────────────────────────────────────────────────
    def pick(self, slot):
        """집을 자리 위로 이동 → 수직 하강 → 잡기 → 수직 상승."""
        x = cfg.TRAY_ANCHOR_XY[0] + slot["offset"][0]
        y = cfg.TRAY_ANCHOR_XY[1] + slot["offset"][1]
        self.log.info("[%s] 집기" % slot["name"])
        self.move_abs(x, y, slot["pick_z"] + slot["lift"])
        self.move_rel(dz=-slot["lift"], fine=True)
        self.gripper.grip()
        self.move_rel(dz=+slot["lift"])

    def transfer_to_assembly(self):
        """트레이 위에서 조립 지그 위로 평행이동."""
        self.move_rel(dx=cfg.TRAY_TO_ASM[0], dy=cfg.TRAY_TO_ASM[1])

    def place(self, slot):
        """조립 지그 위로 이동 → 수직 하강 → 놓기 → 수직 상승."""
        self.log.info("[%s] 놓기" % slot["name"])
        down = (slot["pick_z"] + slot["lift"]) - cfg.PLANET_PLACE_Z
        self.transfer_to_assembly()
        self.move_rel(dz=-down, fine=True)
        self.gripper.release()
        self.move_rel(dz=+down)

    def force_insert(self, slot):
        """조립 지그 위로 이동 → 시작 높이까지 하강 → 힘 제어로 흔들며 삽입 → 놓기 → 후퇴."""
        self.log.info("[%s] 힘 제어 삽입" % slot["name"])
        down = (slot["pick_z"] + slot["lift"]) - cfg.SUN_INSERT_START_Z
        self.transfer_to_assembly()
        self.move_rel(dz=-down, fine=True)
        if self.dz_test > 0:
            self.log.warn("테스트 모드(Z +%.0f): 힘 제어 삽입은 건너뜁니다." % self.dz_test)
        else:
            self._compliant_insert()
        self.gripper.release()
        self.move_rel(dz=+cfg.RETREAT_AFTER_INSERT)

    # ── 힘 제어 ───────────────────────────────────────────────────────
    def _force_z(self):
        f = self.dr.get_tool_force(self.dr.DR_TOOL)
        if not isinstance(f, (list, tuple)) or len(f) < 3:
            raise GearError("힘 센서 값을 읽지 못했습니다.")
        return abs(f[2])

    def _pos_z(self):
        res = self.dr.get_current_posx(self.dr.DR_BASE)
        try:
            return float(res[0][2])
        except (TypeError, IndexError):
            raise GearError("현재 위치를 읽지 못했습니다.")

    def _compliant_insert(self):
        dr = self.dr
        engaged = False
        try:
            self._check(dr.task_compliance_ctrl(stx=cfg.INSERT_STIFFNESS, time=0.0), "컴플라이언스 제어 시작")
            time.sleep(0.2)
            self._check(dr.set_desired_force(fd=cfg.INSERT_FORCE, dir=cfg.INSERT_FORCE_DIR, time=0.0,
                                             mod=dr.DR_FC_MOD_ABS), "목표 힘 설정")

            # 1단계: 좌우로 돌리면서 이빨을 맞춤 (원본의 첫 번째 while True)
            deadline = time.monotonic() + cfg.INSERT_WIGGLE_TIMEOUT
            while True:
                self._check(dr.move_periodic(amp=cfg.WIGGLE_AMP, period=cfg.WIGGLE_PERIOD,
                                             atime=cfg.WIGGLE_ATIME, repeat=cfg.WIGGLE_REPEAT,
                                             ref=dr.DR_TOOL), "흔들기 동작")
                fz, z = self._force_z(), self._pos_z()
                self.log.info("  흔들기: Z=%.2f mm, 반력=%.1f N" % (z, fz))
                if cfg.INSERT_Z_MIN <= z <= cfg.INSERT_Z_MAX:
                    engaged = True
                    break
                if fz >= cfg.INSERT_JAM_FORCE:
                    break
                if time.monotonic() > deadline:
                    raise GearError("삽입 실패: %.0f초 동안 맞물리지 않았습니다 (Z=%.2f)."
                                    % (cfg.INSERT_WIGGLE_TIMEOUT, z))

            # 2단계: 바닥에 닿아 반력이 올라올 때까지 대기 (원본의 두 번째 while True)
            deadline = time.monotonic() + cfg.INSERT_SEAT_TIMEOUT
            while self._force_z() < cfg.INSERT_SEAT_FORCE:
                if time.monotonic() > deadline:
                    raise GearError("삽입 실패: %.0f초 안에 안착 반력(%.0fN)이 나오지 않았습니다."
                                    % (cfg.INSERT_SEAT_TIMEOUT, cfg.INSERT_SEAT_FORCE))
                time.sleep(0.05)

            z = self._pos_z()
            if engaged or cfg.INSERT_Z_MIN <= z <= cfg.INSERT_Z_MAX:
                self.log.info("  삽입 완료: 최종 Z=%.2f mm" % z)
            else:
                self.log.warn("  반력은 나왔지만 Z=%.2f 로 기준 구간(%.2f~%.2f) 밖입니다. 이빨 위에 얹혔을 수 있습니다."
                              % (z, cfg.INSERT_Z_MIN, cfg.INSERT_Z_MAX))
        finally:
            # 어떤 경우에도 힘/컴플라이언스 제어는 반드시 해제
            dr.release_force(time=0.0)
            dr.release_compliance_ctrl()

    # ── 전체 순서 ─────────────────────────────────────────────────────
    def run_sequence(self, slots):
        skills = {"place": self.place, "insert": self.force_insert}
        self.go_home()
        self.gripper.release()          # 시작할 때 그리퍼를 확실히 열어 둠
        for slot in slots:
            self.pick(slot)
            skills[slot["task"]](slot)
        self.go_home()
