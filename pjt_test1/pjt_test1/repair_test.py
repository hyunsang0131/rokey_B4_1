# 침대 자동 보수 테스트: 측정 → 빈칸/맞지 않는 모듈 찾기 → 트레이에서 맞는 큐브 찾기 → 교체 → 재측정
#
#   ros2 run pjt_test1 repair_test                       # bed_fin.json + exchange_fin.json
#   ros2 run pjt_test1 repair_test --pad                 # 침대에 겉감(네오프렌)이 있을 때 (침대 측정만 30N, 12N부터, 0.25mm)
#   ros2 run pjt_test1 repair_test --use-map             # 트레이를 재지 않고 저장된 강성 지도 사용 (기본은 매번 잰다)
#   ros2 run pjt_test1 repair_test --show                # 저장된 강성 지도만 보기 (로봇 필요 없음)
#   ros2 run pjt_test1 repair_test --plan --sim-bed 12,12,-,7,12,12 --sim-tray 7,12,-,12,7,12
#                                                        # 로봇 없이: 가짜 강성으로 판정·계획만 확인
#                                                        # (--sim-tray 를 빼면 저장된 지도의 트레이로 계획)
#
# 강성 지도 (~/ws_cobot_pjt/repair_test/state.json)
#   침대·트레이 각 칸의 강성. 침대를 잴 때, 트레이를 잴 때, 큐브를 하나 옮길 때마다, 재측정 후 바로 저장한다.
#   옮긴 큐브는 강성이 같이 따라간다 (트레이↔침대는 --rack-to-bed 로 환산한 추정값, 'moved').
#   한 번 실행(시나리오)마다 트레이는 처음에 한 번만 재고, 그 뒤로는 이 지도를 옮길 때마다 고쳐 가며 쓴다.
#   다음 실행은 다시 트레이부터 잰다 (사이에 손으로 큐브를 옮겨도 안전). --use-map 이면 재지 않고 지도를 이어 쓴다.
#   옮기는 도중에 멈추면 그 두 칸을 '확인 필요(unknown)'로 표시한다 → 다음 실행에서 트레이를 다시 재면 풀린다.
#
#   옵션
#     --ratio 0.75        두 무리(단단/무름)로 나눌 최소 비율: 가장 단단한 / 가장 무른 이 1/0.75(1.33배) 이상이면 나눈다
#                         실측 40A ≈ 12.3, 10A ≈ 7.0 N/mm (1.76배) → 10A 를 잡고 40A 끼리 흩어짐(±2%)은 통과
#     --rack-to-bed 1.0   트레이에서 잰 강성 → 침대에서 잴 강성 환산 비율 (겉감이 있으면 1 이 아님. 재측정 후 제안 값 출력)
#     --yes               교체 계획을 묻지 않고 바로 실행
#     --modbus            그리퍼를 /onrobot/sendCommand 로 (기본은 DO)
#
# 순서
#   1) 침대 6칸 강성 측정 (stiffness_test 와 같은 press_at)
#   2) 판정: 빈칸(닿지 않음, 접촉 높이가 낮음) / 맞지 않음(다른 칸 중앙값 대비 --ratio 밖, 너무 무름, 키 큼)
#   3) 문제가 없으면 끝
#   4) 트레이: 6칸을 눌러서 잰다 (--use-map 이고 지도가 있으면 저장된 지도 사용)
#   5) 채울 칸마다 "그 칸의 상하좌우 이웃 칸 평균 강성"과 비율이 가장 가까운 트레이 큐브를 고른다 (bed_math.plan_fill)
#   6) 이동 순서: 빈칸 먼저 채우기 → 맞지 않는 칸은 [그 모듈을 트레이 빈 자리로] → [고른 큐브를 침대로]
#      (bed_math.plan_repair_moves). 트레이에 빈 자리가 하나도 없으면 맞지 않는 모듈을 뺄 곳이 없어 시작하지 않는다.
#   7) 계획을 보여 주고 y 를 받으면 교체 (bring_test 와 같은 잡기/놓기 동작)
#   8) 침대 6칸 재측정 → 빈칸/맞지 않음이 없으면 "보수 완료", 남으면 칸을 알려 준다
#
# 칸 배치 (bed_config.BED_ROWS=3, BED_COLS=2 = 좌표 pin_b1~6 와 같음)
#   1 2        이웃: 1-2, 1-3, 2-4, 3-4, 3-5, 4-6, 5-6
#   3 4
#   5 6
#
# 실행 전 확인: sodreal 연결, 서보 ON(STANDBY), TCP = 좌표 파일의 tcp(GripperDA_v1), 비상정지 손에 들기,
#              WebLogic 프로그램 RUN (DO 그리퍼), 트레이에 빈 자리 1개 이상 (맞지 않는 모듈을 뺄 곳)

import argparse
import json
import os
import statistics
import sys
import time

import rclpy
from rclpy.utilities import remove_ros_args
import DR_init

from bed_inspect import bed_config as cfg
from bed_inspect import bed_math
from bed_inspect.bed_math import PlanError

DR_init.__dsr__id = cfg.ROBOT_ID
DR_init.__dsr__model = cfg.ROBOT_MODEL

TEACH_DIR = os.path.expanduser('~/ws_cobot_pjt/teach_points')
BED_FILE = os.path.join(TEACH_DIR, 'bed_fin.json')
TRAY_FILE = os.path.join(TEACH_DIR, 'exchange_fin.json')
OUT_DIR = os.path.expanduser('~/ws_cobot_pjt/repair_test')
STATE_FILE = os.path.join(OUT_DIR, 'state.json')      # 지금 침대·트레이 강성 지도 (항상 최신)
N = 6
BED, TRAY = 'pin_b', 'tray_b'           # 점 이름: pin_b1, pin_b1_down, pin_b1_up ...
VEL_X, ACC_X = 100, 100                 # 칸 사이 이동 (bring_test 와 같음)
VEL_Z, ACC_Z = 30, 30                   # 칸 안으로 내려가고 올라올 때
SETTLE = 0.3                            # 내려간 뒤 놓기 전 대기 [s]

# 측정 설정. 트레이 결과(--step 0.25 에서 40A/10A 가 깔끔히 나뉨)를 기본으로 한다
BARE = {'force': 20.0, 'from': 5.0, 'step': 0.25}      # 트레이: stiffness_test --step 0.25 와 같음
PAD = {'force': 30.0, 'from': 12.0, 'step': 0.25}      # 침대(겉감): stiffness_test --force 30 --from 12 --pad 와 같음
# 빈칸 판단: 접근 높이(교시점 -40 mm)에서 이만큼 내려가도 안 닿으면 빈칸으로 보고 바로 올라온다 [mm]
#   실측 (2026-11-19): 침대  접근 114.4 → 큐브 윗면 106~110 (4~8 mm), 빈칸 바닥 89 (25 mm)
#                      트레이 접근 109.1 → 큐브 윗면 97~98 (11~12 mm), 빈칸 바닥 78 (31 mm)
#   → '큐브 윗면 + 약 8 mm(EMPTY_DZ)' 에서 멈춘다. 바닥까지는 9~11 mm 여유
SEARCH = {BED: 16.0, TRAY: 20.0}
PROBE_VEL = 5.0                         # 빠른 탐색: 이 속도[mm/s]로 멈추지 않고 내려가며 접촉을 본다 (--no-probe 면 1 mm 계단만)
GRADE_KO = {'ok': '정상', 'mismatch': '맞지 않음', 'empty': '빈칸', 'nodata': '측정 실패', None: '-'}
LINE = '  ' + '-' * 74


def bed_name(i):
    return f'{BED}{i + 1}'


def tray_name(j):
    return f'{TRAY}{j + 1}'


# ── 판정과 계획 (로봇 없이 계산만) ──────────────────────────────────────
def judge_bed(blocks, ratio):
    """침대 블록 판정. blocks[i]['grade'] 를 채우고 (빈칸, 맞지 않는 칸) 번호 목록을 돌려준다 (0부터).

    빈칸/측정 실패/키 큼/너무 무름은 bed_math.find_empties 로 먼저 가른다.
    나머지는 강성을 '가장 크게 벌어진 곳'에서 두 무리로 나눠 무른 무리를 교체 대상으로 한다
    (가장 단단한 것 / 가장 무른 것 이 1/ratio 미만이면 한 종류뿐 = 모두 정상).
    bed_math.judge_auto(다른 칸 중앙값 기준)를 쓰지 않는 이유: 무른 큐브가 절반 이상이면 기준이
    무른 쪽으로 옮겨가 단단한 칸까지 '맞지 않음'이 된다. 이 프로젝트의 불량은 '무른 모듈'이다.
    측정 실패 칸은 판단할 수 없으니 교체 대상에 넣지 않는다 (호출한 쪽에서 따로 알린다).
    """
    empties = bed_math.find_empties(blocks, cfg.EMPTY_DZ, cfg.TALL_DZ)
    live = [i for i, b in enumerate(blocks) if b['grade'] is None]
    special = [i for i in live if blocks[i].get('too_soft') or blocks[i].get('tall')]
    rest = [i for i in live if i not in special and blocks[i].get('k')]
    thr = bed_math.split_classes([blocks[i]['k'] for i in rest], 1.0 / ratio)
    ok = [i for i in rest if thr is None or blocks[i]['k'] >= thr]
    ref = statistics.mean(blocks[i]['k'] for i in ok) if ok else None
    mism = special + [i for i in rest if i not in ok]
    for i in live:
        b = blocks[i]
        b['grade'] = 'mismatch' if i in mism else 'ok'
        if ref and b.get('k'):
            b['ref'], b['ref_src'], b['dev'] = ref, 'hard', b['k'] / ref - 1.0
    return empties, sorted(mism)


def judge_tray(blocks):
    """트레이 블록: 빈 자리와 후보 강성 {자리: k}. 측정 실패한 자리는 후보에서 뺀다."""
    empties = bed_math.find_empties(blocks, cfg.EMPTY_DZ, cfg.TALL_DZ)
    cands = {j: b['k'] for j, b in enumerate(blocks) if b['grade'] is None and b.get('k')}
    return empties, cands


def make_plan(bed_blocks, tray_blocks, empties, mism, rack_to_bed, ratio):
    """채울 칸에 넣을 트레이 큐브와 이동 순서. 계획을 세울 수 없으면 PlanError."""
    tray_empty, cands = judge_tray(tray_blocks)
    cands_bed = {j: k * rack_to_bed for j, k in cands.items()}
    todo = sorted(set(empties) | set(mism))
    keep = {i: b['k'] for i, b in enumerate(bed_blocks) if b['grade'] == 'ok' and b.get('k')}
    fill = bed_math.plan_fill(todo, keep, cands_bed, cfg.BED_ROWS, cfg.BED_COLS)
    # 가장 가까운 큐브라도 기준과 --ratio 밖이면 넣어도 다시 '맞지 않음'이 된다 → 시작하지 않는다
    off = []
    for i, j in fill['assign']:
        ref = nb_ref(i, keep) or fill['target']
        r = cands_bed[j] / ref
        if r < ratio or r > 1.0 / ratio:
            off.append(f'{bed_name(i)}(기준 {ref:.2f}, 고른 {tray_name(j)} {cands_bed[j]:.2f})')
    if off:
        raise PlanError('트레이에 기준에 맞는 큐브가 모자랍니다: ' + ', '.join(off))

    bed_occ = [None if i in empties else 'cube' for i in range(N)]
    tray_occ = [None if j in tray_empty else 'cube' for j in range(N)]
    moves = bed_math.plan_repair_moves(bed_occ, tray_occ, fill['assign'], mism)
    if any(m['dst'][0] == 'discard' for m in moves):
        raise PlanError('맞지 않는 모듈을 뺄 트레이 빈 자리가 없습니다. 트레이에 빈 자리를 1개 이상 만드세요.')
    return {'fill': fill, 'moves': moves, 'tray_empty': tray_empty, 'cands': cands}


def move_names(mv):
    """plan_repair_moves 의 이동 하나 → (출발 점 이름, 도착 점 이름)."""
    def name(side, idx):
        return bed_name(idx) if side == 'bed' else tray_name(idx)
    return name(*mv['src']), name(*mv['dst'])


def nb_ref(i, keep):
    ks = [keep[j] for j in bed_math.grid_neighbors(i, cfg.BED_ROWS, cfg.BED_COLS) if j in keep]
    return statistics.mean(ks) if ks else None


# ── 출력 ──────────────────────────────────────────────────────────────
def print_blocks(title, blocks, name_fn):
    print(LINE)
    print(f'  {title}')
    print(f'  {"칸":9s} {"강성 k":>11s} {"기준 대비":>8s} {"윗면 높이":>10s} {"판정":>10s}  비고')
    for i, b in enumerate(blocks):
        k = f'{b["k"]:.2f} N/mm' if b.get('k') is not None else '-'
        dev = f'{b["dev"] * 100:+.0f}%' if b.get('dev') is not None else '-'
        zs = f'{b["z_surf"]:.1f} mm' if b.get('z_surf') is not None else '-'
        note = ('너무 무름 ' if b.get('too_soft') else '') + ('키 큼 ' if b.get('tall') else '') + (b.get('note') or '')
        print(f'  {name_fn(i):9s} {k:>11s} {dev:>8s} {zs:>10s} {GRADE_KO.get(b.get("grade"), b.get("grade")):>10s}  {note}')
    print(LINE)


def print_plan(plan, bed_blocks):
    keep = {i: b['k'] for i, b in enumerate(bed_blocks) if b['grade'] == 'ok' and b.get('k')}
    f = plan['fill']
    print('  [교체 계획]')
    for i, j in f['assign']:
        ref = nb_ref(i, keep)
        ref_txt = f'이웃 평균 {ref:.2f}' if ref else f'전체 평균 {f["target"]:.2f}'
        print(f'    {bed_name(i)} ← {tray_name(j)} (트레이 k {plan["cands"][j]:.2f} N/mm, {ref_txt})')
    if f.get('mean_after') is not None:
        print(f'    교체 후 예상 평균 {f["mean_after"]:.2f} N/mm (목표 {f["target"]:.2f}, 차이 {f["err"] * 100:.1f}%)')
    print('  [이동 순서]')
    for n, mv in enumerate(plan['moves'], 1):
        s, d = move_names(mv)
        print(f'    {n}. {s} → {d}')
    print(LINE)


def spread_text(blocks):
    ks = [b['k'] for b in blocks if b.get('k') is not None]
    if len(ks) < 2:
        return ''
    m = statistics.mean(ks)
    return (f'평균 {m:.2f} N/mm, 표준편차 {statistics.stdev(ks):.2f} ({statistics.stdev(ks) / m * 100:.1f}%), '
            f'최대/최소 {max(ks) / min(ks):.2f} 배')


# ── 가짜 측정값 (--sim-bed / --sim-tray) ──────────────────────────────
def sim_blocks(text):
    """'12,12,-,7,12,12' → press_at 결과 모양의 dict 6개. '-' 는 빈칸(닿지 않음)."""
    vals = [v.strip() for v in text.split(',')]
    if len(vals) != N:
        raise ValueError(f'값 {N}개가 필요합니다: {text}')
    out = []
    for v in vals:
        if v == '-':
            out.append({'k': None, 'z_surf': None, 'z0': None, 'error': 'no_contact', 'note': '닿지 않음'})
        else:
            out.append({'k': float(v), 'z_surf': 97.5, 'z0': 97.5, 'error': None, 'note': ''})
    return out


def make_emu(spec, pts, log):
    """에뮬레이터(sodvir) 가짜 힘. REPAIR_EMU="bed=H,H,-,S,H,H;tray=S,H,S,H,-,H" (H 40A, S 10A, - 빈칸)

    bed_inspect.bed_emu 는 bed_N / rack_N 이름과 *_PICK_DZ 로 큐브 윗면을 계산하므로, 우리 점(pin_bN, tray_bN)과
    _down 높이로 맞춰서 넘긴다. 실제 로봇에서는 이 환경변수를 주지 않는다.
    """
    from bed_inspect import bed_emu
    parts = dict(p.split('=', 1) for p in spec.split(';') if '=' in p)
    cfg.P_BED = [f'bed_{n}' for n in range(1, N + 1)]
    cfg.P_RACK = [f'rack_{n}' for n in range(1, N + 1)]
    cfg.BED_PICK_DZ = pts[f'{BED}1_down']['posx'][2] - pts[f'{BED}1']['posx'][2]
    cfg.RACK_PICK_DZ = pts[f'{TRAY}1_down']['posx'][2] - pts[f'{TRAY}1']['posx'][2]
    # 큐브 윗면을 실물과 맞춘다 (잡는 높이 + 22 mm → 침대 약 107, 트레이 약 99 mm). 빈칸 탐색 깊이(SEARCH)가 실물 기준이라서
    bed_emu.GRIP_DEPTH = 22.0
    mapped = {}
    for n in range(1, N + 1):
        mapped[f'bed_{n}'], mapped[f'rack_{n}'] = pts[f'{BED}{n}'], pts[f'{TRAY}{n}']
    emu = bed_emu.EmuForce(mapped, bed_emu._parse(parts.get('bed', ''), N), bed_emu._parse(parts.get('tray', ''), N))
    log(f'에뮬레이터 가짜 힘: 침대 {emu.bed}, 트레이 {emu.rack}')
    return emu


# ── 강성 지도 (state.json): 지금 침대·트레이 각 칸에 어떤 강성의 큐브가 있는지 ─────────────
#   {'updated': 시각, 'bed': [칸 6개], 'tray': [칸 6개]}
#   칸 = None(빈칸) 또는 {'k': 강성 또는 None(모름), 'src': 'measured'|'moved'|'unknown', 'from': 옮겨 온 칸,
#                       'time': 시각}
#   k 의 단위는 그 칸에서 잰 기준이다 (침대 칸 = 침대에서 잰 값, 트레이 칸 = 트레이에서 잰 값).
#   큐브를 옮기면 --rack-to-bed 로 환산해서 따라간다 (실제로 재기 전까지는 'moved' = 추정값).
def now():
    return time.strftime('%Y-%m-%d %H:%M:%S')


def load_state():
    try:
        with open(STATE_FILE) as f:
            st = json.load(f)
        if len(st.get('bed', [])) == N and len(st.get('tray', [])) == N:
            return st
    except (OSError, ValueError):
        pass
    return None


def save_state(st):
    st['updated'] = now()
    os.makedirs(OUT_DIR, exist_ok=True)
    tmp = STATE_FILE + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(st, f, ensure_ascii=False, indent=1)
    os.replace(tmp, STATE_FILE)        # 쓰는 도중에 멈춰도 파일이 깨지지 않게


def cells_from_measure(blocks):
    """측정 결과 → 칸 목록. 빈칸은 None, 강성을 못 잰 칸은 k=None."""
    return [None if b.get('grade') == 'empty' else
            {'k': b.get('k'), 'src': 'measured', 'from': None, 'time': now()} for b in blocks]


def blocks_from_cells(cells):
    """칸 목록 → press_at 결과 모양 (judge_tray / make_plan 에 그대로 넣을 수 있게)."""
    out = []
    for c in cells:
        if c is None:
            out.append({'k': None, 'z_surf': None, 'z0': None, 'error': 'no_contact', 'note': '빈칸 (지도)'})
        elif c.get('k') is None:
            out.append({'k': None, 'z_surf': None, 'z0': None, 'error': 'unknown', 'note': '강성 모름 (지도)'})
        else:
            note = '잰 값' if c['src'] == 'measured' else f'{c.get("from")}에서 옮겨 옴 (추정)'
            out.append({'k': c['k'], 'z_surf': None, 'z0': None, 'error': None, 'note': note})
    return out


def side_key(side):
    """bed_math 이동 계획은 트레이를 'rack' 이라고 부른다 → 지도 키 'tray' 로 맞춘다."""
    return 'tray' if side == 'rack' else side


def apply_move(st, src, dst, rack_to_bed):
    """지도에서 src 칸의 큐브를 dst 칸으로 옮긴다. src/dst = ('bed'|'tray'|'rack', 번호)."""
    src, dst = (side_key(src[0]), src[1]), (side_key(dst[0]), dst[1])
    cube = st[src[0]][src[1]]
    st[src[0]][src[1]] = None
    if cube is None:
        st[dst[0]][dst[1]] = {'k': None, 'src': 'unknown', 'from': None, 'time': now()}
        return
    k = cube.get('k')
    if k is not None and src[0] != dst[0]:
        k = k * rack_to_bed if dst[0] == 'bed' else k / rack_to_bed
    name = bed_name(src[1]) if src[0] == 'bed' else tray_name(src[1])
    st[dst[0]][dst[1]] = {'k': k, 'src': 'moved', 'from': name, 'time': now()}


def print_state(st):
    print(LINE)
    print(f'  강성 지도 ({STATE_FILE}, 갱신 {st.get("updated", "-")})')
    for side, name_fn, ko in (('bed', bed_name, '침대'), ('tray', tray_name, '트레이')):
        for i, c in enumerate(st[side]):
            if c is None:
                txt = '빈칸'
            else:
                k = f'{c["k"]:.2f} N/mm' if c.get('k') is not None else '강성 모름'
                how = {'measured': '잰 값', 'moved': f'{c.get("from")}에서 옮겨 옴(추정)',
                       'unknown': '이동 중 멈춤 - 확인 필요'}.get(c['src'], c['src'])
                txt = f'{k:>12s}  {how}  ({c.get("time", "")})'
            print(f'  {ko:3s} {name_fn(i):9s} {txt}')
    print(LINE)


def save(record):
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, time.strftime('repair_%Y%m%d_%H%M%S.json'))
    with open(path, 'w') as f:
        json.dump(record, f, ensure_ascii=False, indent=1, default=str)
    return path


def strip_curve(blocks):
    return [{k: v for k, v in b.items() if k != 'curve'} for b in blocks]


def main(args=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--bed', default=BED_FILE, help='침대 좌표 파일 (pin_b1~6 + _down/_up)')
    ap.add_argument('--tray', default=TRAY_FILE, help='트레이 좌표 파일 (tray_b1~6 + _down/_up)')
    ap.add_argument('--pad', '--mat', dest='pad', action='store_true', help='침대에 겉감(네오프렌)이 있을 때')
    ap.add_argument('--ratio', type=float, default=0.75, help='맞지 않음 판정 비율 (기본 0.75)')
    ap.add_argument('--rack-to-bed', type=float, default=cfg.RACK_TO_BED, help='트레이 k → 침대 k 환산 비율')
    ap.add_argument('--yes', action='store_true', help='교체 계획을 묻지 않고 실행')
    ap.add_argument('--modbus', action='store_true', help='그리퍼를 /onrobot/sendCommand 로 제어')
    ap.add_argument('--use-map', action='store_true',
                    help='트레이를 재지 않고 저장된 강성 지도 사용 (기본: 실행마다 처음에 한 번 잰다)')
    ap.add_argument('--measure-tray', action='store_true', help=argparse.SUPPRESS)  # 예전 옵션. 지금은 기본
    ap.add_argument('--no-probe', action='store_true',
                    help='빠른 탐색을 끄고 예전처럼 1 mm 계단으로만 접촉을 찾는다')
    ap.add_argument('--show', action='store_true', help='저장된 강성 지도만 보여 주고 끝 (로봇 필요 없음)')
    ap.add_argument('--plan', action='store_true',
                    help='로봇 없이 판정·계획만 (--sim-bed 필요. --sim-tray 가 없으면 저장된 지도의 트레이 사용)')
    ap.add_argument('--sim-bed', help="가짜 침대 강성 6개, 예: 12,12,-,7,12,12 ('-' 빈칸)")
    ap.add_argument('--sim-tray', help='가짜 트레이 강성 6개')
    opt = ap.parse_args(remove_ros_args(sys.argv)[1:])

    if not 0.3 <= opt.ratio < 1.0:
        print('--ratio 는 0.3 ~ 1.0 사이로 주세요.')
        return

    state = load_state()
    if opt.show:
        if state:
            print_state(state)
        else:
            print(f'저장된 강성 지도가 없습니다: {STATE_FILE}')
        return

    # ── 로봇 없이 계획만 ──
    if opt.plan:
        if not opt.sim_bed or not (opt.sim_tray or state):
            print('--plan 에는 --sim-bed 가 필요하고, --sim-tray 가 없으면 저장된 강성 지도가 있어야 합니다.')
            return
        bed = sim_blocks(opt.sim_bed)
        empties, mism = judge_bed(bed, opt.ratio)
        print_blocks('침대 판정 (가짜 값)', bed, bed_name)
        if not empties and not mism:
            print('  모든 칸이 정상입니다. 교체할 것이 없습니다.')
            return
        tray = sim_blocks(opt.sim_tray) if opt.sim_tray else blocks_from_cells(state['tray'])
        bed_math.find_empties(tray, cfg.EMPTY_DZ, cfg.TALL_DZ)
        print_blocks('트레이 (가짜 값)' if opt.sim_tray else '트레이 (저장된 지도)', tray, tray_name)
        try:
            print_plan(make_plan(bed, tray, empties, mism, opt.rack_to_bed, opt.ratio), bed)
        except PlanError as e:
            print(f'  계획을 세울 수 없습니다: {e}')
        return

    # ── 좌표 파일 ──
    pts = {}
    for path in (opt.bed, opt.tray):
        try:
            with open(os.path.expanduser(path)) as f:
                pts.update({p['name']: p for p in json.load(f)})
        except (OSError, ValueError, KeyError) as e:
            print(f'좌표 파일을 읽을 수 없습니다: {path} ({e})')
            return
    need = [f'{s}{n}{t}' for s in (BED, TRAY) for n in range(1, N + 1) for t in ('', '_down', '_up')]
    missing = [n for n in need if n not in pts]
    if missing:
        print(f'좌표 파일에 없는 점: {missing}')
        return

    # 실물은 DO(WebLogic). 에뮬레이터의 RViz 그리퍼는 DO 를 보지 않고 /onrobot/sendCommand(가상 그리퍼)로만
    # 움직이므로, REPAIR_EMU 로 에뮬레이터 시험을 할 때는 서비스로 보내서 닫기/열기가 화면에 보이게 한다
    cfg.GRIPPER_MODE = 'service' if opt.modbus or os.environ.get('REPAIR_EMU') else 'io'
    bed_set, tray_set = (PAD if opt.pad else BARE), BARE
    if opt.pad and opt.rack_to_bed == cfg.RACK_TO_BED:
        print(f'주의: 겉감 모드인데 --rack-to-bed 가 기본값({cfg.RACK_TO_BED:g})입니다. 트레이와 침대의 강성이 다르게 '
              '나오므로 재측정 후 나오는 "제안 값"으로 맞추세요.')

    rclpy.init(args=args)
    node = rclpy.create_node('pjt_repair_test', namespace=cfg.ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        import DSR_ROBOT2 as dr
    except ImportError as e:
        node.get_logger().error(f'DSR_ROBOT2 를 불러오지 못했습니다: {e}')
        rclpy.shutdown()
        return
    from bed_inspect.bed_robot import BedError, BedRobot
    from bed_inspect.bed_tasks import BedTasks

    class Robot(BedRobot):
        """좌표 파일의 점은 posx 만 있으므로 직선 이동(movel)한다."""

        def movej_to(self, point, slow=False):
            if point.get('posj'):
                return super().movej_to(point, slow)
            self.move_to(point, cfg.VELX, cfg.ACCX)

        def move_to(self, point, vel, acc):
            self.require_standby(point['name'])
            ret = self.dr.movel(self.posx(list(point['posx'])), vel=vel, acc=acc, ref=self.dr.DR_BASE)
            self._done(ret, f'[{point["name"]}] 직선 이동')

    log = node.get_logger().info
    robot = Robot(dr, node, log)
    tasks = BedTasks(robot, pts, log)
    if os.environ.get('REPAIR_EMU'):
        robot.emu = make_emu(os.environ['REPAIR_EMU'], pts, log)
    record ={'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'pad': opt.pad, 'ratio': opt.ratio,
              'rack_to_bed': opt.rack_to_bed}
    unknown = {'k': None, 'src': 'unknown', 'from': None, 'time': now()}
    st = state or {'bed': [dict(unknown) for _ in range(N)], 'tray': [dict(unknown) for _ in range(N)]}
    measure_tray = not opt.use_map or state is None
    if state is None and opt.use_map:
        log(f'저장된 강성 지도가 없어서 이번에는 트레이도 잽니다 ({STATE_FILE}).')

    def use(setting):
        cfg.F_TARGET, cfg.F_FIT_MIN, cfg.PRESS_STEP = setting['force'], setting['from'], setting['step']

    def measure(prefix, setting, title):
        use(setting)
        cfg.SEARCH_MAX = SEARCH[prefix]
        cfg.PROBE_VEL = 0.0 if opt.no_probe else PROBE_VEL
        log(f'=== {title} 측정 (목표 {setting["force"]:g}N, 계산 {setting["from"]:g}N~, 스텝 {setting["step"]:g}mm, '
            f'{cfg.SEARCH_MAX:g}mm 안 닿으면 빈칸, 빠른 탐색 {"끔" if opt.no_probe else f"{PROBE_VEL:g}mm/s"}) ===')
        out = []
        for n in range(1, N + 1):
            res = tasks.press_at(pts[f'{prefix}{n}'], cfg.PRESS_APPROACH_DZ)
            k = f'{res["k"]:.2f} N/mm' if res.get('k') is not None else (res.get('note') or res.get('error'))
            pr = res.get('probe')
            if pr:   # 하드웨어 확인용: 빠른 탐색이 어디서 멈췄고 내려가는 동안 힘이 얼마나 보였는지
                k += f'  [탐색 {"닿음" if pr["hit"] else "안 닿음"} z {pr["z"]:.1f}, 최대 {pr["f_peak"]:.1f}N]'
            log(f'  {prefix}{n}: {k}')
            out.append(res)
        return out

    def transfer(src, dst):
        """src 칸의 큐브를 dst 칸으로 (bring_test 와 같은 순서)."""
        log(f'--- {src} → {dst} ---')
        robot.move_to(pts[src], VEL_X, ACC_X)
        robot.grip('block_open')
        robot.move_to(pts[src + '_down'], VEL_Z, ACC_Z)
        robot.grip('block_grip')
        robot.move_to(pts[src + '_up'], VEL_Z, ACC_Z)
        robot.move_to(pts[dst], VEL_X, ACC_X)
        robot.move_to(pts[dst + '_down'], VEL_Z, ACC_Z)
        robot.sleep(SETTLE)
        robot.grip('block_open')
        robot.move_to(pts[dst + '_up'], VEL_Z, ACC_Z)

    try:
        robot.wait_services()
        tcp = robot.prepare()
        tcps = {p.get('tcp', '') for p in pts.values()}
        if tcps != {tcp}:
            raise BedError(f'현재 TCP({tcp or "없음"})와 좌표 파일의 TCP{sorted(tcps)} 가 다릅니다.')
        log(f'TCP {tcp}, 그리퍼 {cfg.GRIPPER_MODE}, 판정 비율 {opt.ratio:g}, 환산 {opt.rack_to_bed:g}'
            + (' (겉감 모드)' if opt.pad else ''))

        # 1~3) 침대 측정 + 판정 → 지도의 침대 칸을 잰 값으로
        bed = measure(BED, bed_set, '침대')
        empties, mism = judge_bed(bed, opt.ratio)
        st['bed'] = cells_from_measure(bed)
        save_state(st)
        print_blocks('침대 판정', bed, bed_name)
        print('  ' + spread_text(bed))
        record['bed_before'] = strip_curve(bed)
        nodata = [bed_name(i) for i, b in enumerate(bed) if b['grade'] == 'nodata']
        if nodata:
            print(f'  측정 실패 칸 {nodata} 은 판단하지 않습니다 (stiffness_test 로 따로 확인).')
        if not empties and not mism:
            print('  모든 칸이 정상입니다. 교체할 것이 없습니다.')
            return

        # 4~6) 트레이: 이번 실행에서 한 번 잰다 (--use-map 이면 저장된 지도) → 계획
        if measure_tray:
            tray = measure(TRAY, tray_set, '트레이')
            bed_math.find_empties(tray, cfg.EMPTY_DZ, cfg.TALL_DZ)
            st['tray'] = cells_from_measure(tray)
            save_state(st)
            print_blocks('트레이 (측정)', tray, tray_name)
        else:
            tray = blocks_from_cells(st['tray'])
            bed_math.find_empties(tray, cfg.EMPTY_DZ, cfg.TALL_DZ)
            print_blocks(f'트레이 (저장된 지도, {state.get("updated", "-")})', tray, tray_name)
            print('  큐브를 손으로 옮겼다면 --use-map 없이 실행해서 다시 재세요.')
        record['tray'] = strip_curve(tray)
        record['tray_measured'] = measure_tray
        plan = make_plan(bed, tray, empties, mism, opt.rack_to_bed, opt.ratio)
        print_plan(plan, bed)
        record['assign'] = [(bed_name(i), tray_name(j)) for i, j in plan['fill']['assign']]
        record['moves'] = [move_names(m) for m in plan['moves']]

        # 7) 교체
        if not opt.yes and input('  이 계획대로 교체할까요? (y: 실행 / 그 밖: 취소) ').strip().lower() != 'y':
            print('  취소했습니다.')
            return
        for mv in plan['moves']:
            try:
                transfer(*move_names(mv))
            except BaseException:
                # 옮기는 도중에 멈춤: 큐브가 출발 칸/그리퍼/도착 칸 어디에 있는지 모른다 → 두 칸 다 '확인 필요'
                for side, idx in (mv['src'], mv['dst']):
                    st[side_key(side)][idx] = {'k': None, 'src': 'unknown', 'from': None, 'time': now()}
                save_state(st)
                log(f'{"→".join(move_names(mv))} 이동 중 멈춤. 지도의 두 칸을 "확인 필요"로 표시했습니다 '
                    '(큐브 위치를 확인하고 다시 실행하면 트레이를 새로 잽니다).')
                raise
            apply_move(st, mv['src'], mv['dst'], opt.rack_to_bed)     # 이동이 끝날 때마다 지도 갱신
            save_state(st)
            record['done_moves'] = record.get('done_moves', []) + [move_names(mv)]

        # 8) 재측정 → 지도의 침대 칸을 다시 잰 값으로
        after = measure(BED, bed_set, '침대 재측정')
        e2, m2 = judge_bed(after, opt.ratio)
        st['bed'] = cells_from_measure(after)
        save_state(st)
        print_blocks('재측정 판정', after, bed_name)
        print('  ' + spread_text(after))
        record['bed_after'] = strip_curve(after)
        if not e2 and not m2:
            print('  보수 완료: 빈칸과 맞지 않는 모듈이 없습니다.')
            record['result'] = 'ok'
        else:
            left = [bed_name(i) for i in sorted(set(e2) | set(m2))]
            print(f'  ⚠ 보수 후에도 문제가 남은 칸: {left} → 작업자 확인이 필요합니다.')
            record['result'] = f'left {left}'

        # 환산 비율 제안: 옮겨 넣은 큐브의 (침대 k / 트레이 k)
        ratios = [after[i]['k'] / tray[j]['k'] for i, j in plan['fill']['assign']
                  if after[i].get('k') and tray[j].get('k')]
        if ratios:
            print(f'  환산 비율 제안: --rack-to-bed ≈ {statistics.mean(ratios):.2f} '
                  f'(옮긴 큐브 {len(ratios)}개 기준, 지금 {opt.rack_to_bed:g})')
            record['rack_to_bed_suggest'] = statistics.mean(ratios)
        print_state(st)

    except PlanError as e:
        node.get_logger().error(f'계획을 세울 수 없습니다: {e}')
    except BedError as e:
        node.get_logger().error(str(e))
    except (KeyboardInterrupt, EOFError):
        print()
        node.get_logger().warn('중단됨 (로봇이 아직 움직이면 펜던트에서 정지하세요)')
    finally:
        if len(record) > 4:
            print('기록:', save(record))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
