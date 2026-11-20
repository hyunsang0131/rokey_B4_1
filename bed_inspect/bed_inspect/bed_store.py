# 파일로 저장하는 것들: 교시 위치, 블록 재고, 기준 강성, 측정 기록
#
#   DATA_DIR/inventory.json   침대 6칸과 보관대에 지금 어떤 높이의 블록이 있는지
#   DATA_DIR/calib.json       높이별 기준 강성 (기준값 저장을 했을 때)
#   DATA_DIR/logs/*.csv       측정할 때마다 결과와 힘-변위 곡선

import csv
import glob
import json
import os
import time

from bed_inspect import bed_config as cfg


# ── 교시 위치 ─────────────────────────────────────────────────────────
def teach_path(args=()):
    """교시 파일 경로: 실행 인자(.json) > 환경변수 BED_TEACH_FILE > 설정값."""
    files = [a for a in args if a.endswith('.json')]
    path = files[0] if files else os.environ.get('BED_TEACH_FILE', cfg.TEACH_FILE)
    return os.path.expanduser(path)


# 팀원(pjt_test1 repair_test) 교시 파일 이름 → 공정 이름: pin_b{n} → bed_{n}, tray_b{n} → rack_{n}
#   pin_b{n}_down / _up 은 잡는 높이 / 올라올 자리로 붙여 둔다 (bed_tasks.move_block 이 그대로 씀)
TEAM_PREFIX = {'pin_b': 'bed_', 'tray_b': 'rack_'}


def load_teach(path):
    """교시 JSON → {이름: {'posj': [...], 'posx': [...], 'tcp': ...}}.

    path 에 쉼표로 여러 파일을 줄 수 있다 (예: 'bed_fin.json,exchange_fin.json').
    팀원 파일(pin_b1~6, tray_b1~6 + _down/_up)도 그대로 읽어서 bed_1~6, rack_1~6 으로 바꾼다.
    """
    points = []
    for one in str(path).split(','):
        one = os.path.expanduser(one.strip())
        if not one:
            continue
        with open(one) as f:
            points += json.load(f)
    raw = {p['name']: p for p in points}
    out = dict(raw)
    for old, new in TEAM_PREFIX.items():
        for n in range(1, 7):
            base = raw.get(f'{old}{n}')
            if base is None or f'{new}{n}' in raw:
                continue
            p = dict(base, name=f'{new}{n}')
            if f'{old}{n}_down' in raw:
                p['down'] = list(raw[f'{old}{n}_down']['posx'])
            if f'{old}{n}_up' in raw:
                p['up'] = list(raw[f'{old}{n}_up']['posx'])
            out[f'{new}{n}'] = p
    return out


def required_names():
    # 꼭 필요한 것: 침대 6칸 + 교체 트레이 6칸 (12개).
    # home 은 없으면 기본 관절각(cfg.HOME_POSJ), discard(버리는 통)는 불량 교체(replace) 모드에서만 필요.
    # 커버는 교시하지 않고 bed_config 의 좌표(COVER_*)로 움직인다
    return list(cfg.P_BED) + list(cfg.P_RACK)


def fill_defaults(points):
    """교시 파일에 home 이 없으면 기본 관절각으로 채운다."""
    if cfg.P_HOME not in points:
        points[cfg.P_HOME] = {'name': cfg.P_HOME, 'posj': list(cfg.HOME_POSJ), 'posx': None, 'tcp': None}
    return points


def missing_names(points):
    return [n for n in required_names() if n not in points]


def choose_teach_file(args=()):
    """press_test 용: 인자로 받은 파일, 없으면 교시 폴더 목록에서 고른다. 취소하면 None."""
    files = [a for a in args if a.endswith('.json')]
    if files:
        return os.path.expanduser(files[0])
    folder = os.path.dirname(cfg.TEACH_FILE)
    paths = sorted(glob.glob(os.path.join(folder, '*.json')), key=os.path.getmtime, reverse=True)
    if not paths:
        print(f'교시 파일이 없습니다: {folder}')
        return None
    print(f'\n=== 교시 파일 ({folder}) ===')
    for i, p in enumerate(paths, 1):
        print(f'  {i}. {os.path.basename(p)}')
    while True:
        sel = input('파일 번호 (q: 종료): ').strip()
        if sel == 'q':
            return None
        if sel.isdigit() and 1 <= int(sel) <= len(paths):
            return paths[int(sel) - 1]


def use_sim_data_if_sim(path):
    """가짜 로봇용 교시 파일이면 재고/기록 폴더를 따로 쓴다 (실제 재고 기록을 건드리지 않게)."""
    if os.path.abspath(path) == os.path.abspath(cfg.SIM_TEACH_FILE):
        cfg.DATA_DIR = cfg.SIM_DATA_DIR
        return True
    return False


# ── JSON 읽고 쓰기 ────────────────────────────────────────────────────
def _path(name):
    os.makedirs(cfg.DATA_DIR, exist_ok=True)
    return os.path.join(cfg.DATA_DIR, name)


def _save_json(name, data):
    path = _path(name)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    os.replace(tmp, path)          # 쓰는 도중 꺼져도 파일이 깨지지 않게


def _load_json(name, default):
    try:
        with open(_path(name)) as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


# ── 재고 ──────────────────────────────────────────────────────────────
def load_inventory():
    """{'bed': [높이 × 6], 'rack': [높이 또는 None × 보관대 수]}. 파일이 없으면 설정의 처음 값."""
    inv = _load_json('inventory.json', None)
    if not inv or len(inv.get('bed', [])) != len(cfg.P_BED) or len(inv.get('rack', [])) != len(cfg.P_RACK):
        inv = {'bed': list(cfg.BED_INIT), 'rack': list(cfg.RACK_INIT)}
        save_inventory(inv)
    return inv


def save_inventory(inv):
    _save_json('inventory.json', {'bed': inv['bed'], 'rack': inv['rack']})


def check_inventory(bed, rack):
    """재고 값이 말이 되는지 확인. 문제가 있으면 설명 문자열, 없으면 None."""
    if len(bed) != len(cfg.P_BED):
        return f'침대 칸은 {len(cfg.P_BED)}개여야 합니다.'
    if len(rack) != len(cfg.P_RACK):
        return f'보관대 자리는 {len(cfg.P_RACK)}개여야 합니다.'
    for h in bed:
        if h is not None and h not in cfg.INVENTORY_HEIGHTS:
            return f'침대 칸의 높이는 {cfg.INVENTORY_HEIGHTS} 중 하나이거나 비어 있어야 합니다 (입력: {h}).'
    for h in rack:
        if h is not None and h not in cfg.INVENTORY_HEIGHTS:
            return f'보관대의 높이는 {cfg.INVENTORY_HEIGHTS} 중 하나이거나 비어 있어야 합니다 (입력: {h}).'
    return None


# ── 기준 강성 ─────────────────────────────────────────────────────────
def load_calib():
    return _load_json('calib.json', {})


def save_calib(calib):
    _save_json('calib.json', calib)


# ── 측정 기록 ─────────────────────────────────────────────────────────
def save_measure_log(tag, blocks, tilt):
    """측정 결과를 CSV 2개로 저장(요약, 곡선). 요약 파일 경로를 돌려준다."""
    folder = _path('logs')
    os.makedirs(folder, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    fname = os.path.join(folder, f'{stamp}_{tag}.csv')

    def num(v, fmt='{:.3f}'):
        return '' if v is None else fmt.format(v)

    with open(fname, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['slot', 'height', 'k', 'ref', 'dev_pct', 'grade', 'z_surf', 'disp', 'f_max',
                    'r2', 'error', 'note'])
        for b in blocks:
            dev = None if b.get('dev') is None else b['dev'] * 100
            w.writerow([b['slot'], b['height'], num(b.get('k'), '{:.4f}'), num(b.get('ref'), '{:.4f}'),
                        num(dev, '{:.1f}'), b.get('grade', ''), num(b.get('z_surf')), num(b.get('disp')),
                        num(b.get('f_max')), num(b.get('r2')), b.get('error') or '', b.get('note', '')])
        if tilt:
            w.writerow([])
            w.writerow(['tilt_col_deg', 'tilt_row_deg', 'plane_rms_mm'])
            w.writerow([num(tilt['col_deg']), num(tilt['row_deg']), num(tilt['rms'])])
    with open(fname.replace('.csv', '_curve.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['slot', 'z', 'force'])
        for b in blocks:
            for z, fo in b.get('curve', []):
                w.writerow([b['slot'], f'{z:.3f}', f'{fo:.3f}'])
    return fname
