# 에뮬레이터(sodvir)용 가짜 힘. 에뮬레이터에는 힘 센서가 없어서 누르기 측정이 전부 "닿지 않음"이 된다.
# 환경변수 BED_EMU_FORCE 를 주면, 손끝 위치와 "어느 칸에 어떤 큐브가 있는지"로 힘을 계산해 대신 돌려준다.
#
#   BED_EMU_FORCE="bed=H,H,-,S,H,H;rack=S,H,S,H,S,-"
#     H = 단단한 큐브(40A), S = 무른 큐브(10A), T = 키 큰 10A(TALL_HEIGHT), - = 빈칸.  bed 는 1~6번, rack 은 보관대 1~6번 순서
#
# 그리퍼로 큐브를 집고 놓는 것도 따라가므로, 옮긴 뒤 재측정하면 옮긴 결과가 나온다.
# 실제 로봇에서는 이 환경변수를 주지 않는다 (주지 않으면 이 파일은 아무 일도 하지 않는다).

import math
import os
import random

from bed_inspect import bed_config as cfg

K = {'H': 4.0, 'S': 1.0, 'T': 1.0 * cfg.H_REF / cfg.TALL_HEIGHT}   # 강성 [N/mm]. 40A:10A 약 4배, 키 큰 10A 는 높이에 반비례
DH = {'H': 0.0, 'S': 0.0, 'T': cfg.TALL_HEIGHT - cfg.H_REF}         # 보통 큐브보다 윗면이 높은 양 [mm]
GRIP_DEPTH = 15.0               # 큐브 윗면에서 이만큼 아래를 잡는다고 본다 (bed_sim 과 같음)
NOISE = 0.05                    # 힘 노이즈 [N]
XY_TOL = 10.0                   # 칸 중심에서 이만큼 안이면 그 칸 위로 본다 [mm]


def _parse(text, n):
    out = [None if x.strip() in ('', '-') else x.strip().upper() for x in text.split(',')]
    if len(out) != n or any(v is not None and v not in K for v in out):
        raise ValueError(f'BED_EMU_FORCE 형식 오류: {text!r} (H/S/- 를 {n}개)')
    return out


def from_env(points, log):
    """환경변수가 있으면 EmuForce 를, 없으면 None 을 돌려준다."""
    spec = os.environ.get('BED_EMU_FORCE')
    if not spec:
        return None
    parts = dict(p.split('=', 1) for p in spec.split(';') if '=' in p)
    emu = EmuForce(points, _parse(parts.get('bed', ''), len(cfg.P_BED)),
                   _parse(parts.get('rack', ''), len(cfg.P_RACK)))
    log(f'에뮬레이터 가짜 힘 사용: 침대 {emu.bed}, 보관대 {emu.rack}')
    return emu


class EmuForce:
    def __init__(self, points, bed, rack):
        self.bed, self.rack = list(bed), list(rack)
        self.held = None
        self.rnd = random.Random(0)
        # 칸마다 (x, y, 보통 큐브 윗면 높이): 블록 잡는 높이(교시 높이 + *_PICK_DZ)에서 GRIP_DEPTH 위가 윗면
        self.st = {}
        for kind, names, dz in (('bed', cfg.P_BED, cfg.BED_PICK_DZ), ('rack', cfg.P_RACK, cfg.RACK_PICK_DZ)):
            for i, n in enumerate(names):
                x, y, z = points[n]['posx'][:3]
                self.st[(kind, i)] = (x, y, z + dz + GRIP_DEPTH)

    def _under(self, xyz):
        for key, (x, y, top) in self.st.items():
            if math.hypot(xyz[0] - x, xyz[1] - y) <= XY_TOL:
                return key, top
        return None, None

    def _arr(self, kind):
        return self.bed if kind == 'bed' else self.rack

    def fz(self, xyz):
        """손끝이 큐브를 누르는 힘 [N] (누를수록 +)."""
        f = 0.0
        key, top = self._under(xyz)
        if key is not None and self.held is None:
            cube = self._arr(key[0])[key[1]]
            if cube is not None and top + DH[cube] - xyz[2] > 0:
                f = K[cube] * (top + DH[cube] - xyz[2])
        return f + self.rnd.gauss(0, NOISE)

    def on_grip(self, key, xyz):
        """그리퍼 명령을 따라 큐브를 집거나 놓는다."""
        where, top = self._under(xyz)
        if key == 'block_grip' and self.held is None and where is not None:
            arr = self._arr(where[0])
            if arr[where[1]] is not None and xyz[2] < top + DH[arr[where[1]]]:
                self.held, arr[where[1]] = arr[where[1]], None
        elif key == 'block_open' and self.held is not None:
            if where is not None and self._arr(where[0])[where[1]] is None:
                self._arr(where[0])[where[1]] = self.held
            self.held = None            # 버리는 곳이나 엉뚱한 곳이면 사라진다
