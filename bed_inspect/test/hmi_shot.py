# 화면 모양 확인용: ROS 없이 가짜 상태를 넣어 화면을 PNG 로 저장한다.
#   QT_QPA_PLATFORM=offscreen python3 test/hmi_shot.py 출력폴더 [상태.json ...]
import json
import os
import sys

from PyQt5 import QtWidgets

from bed_inspect import bed_config as cfg
from bed_inspect.bed_hmi import BedWindow


def block(slot, k, grade, h=30.0, dev=None, note='', error=None):
    return {'slot': slot, 'height': h, 'k': k, 'grade': grade, 'dev': dev, 'ref': 2.0, 'ref_src': 'median',
            'z_surf': 119.5, 'disp': 10.2, 'f_max': 20.3, 'r2': 0.998, 'note': note, 'error': error}


def base(**kw):
    s = {'state': 'idle', 'mode': None, 'step': 0, 'step_name': '', 'detail': '', 'message': '', 'active_slot': None,
         'first': None, 'second': None, 'plan': None,
         'layout': {'rows': cfg.BED_ROWS, 'cols': cfg.BED_COLS, 'heights': cfg.BLOCK_HEIGHTS, 'warn': 0.15, 'fail': 0.30},
         'step_names': cfg.STEP_NAMES, 'inventory': {'bed': [30.0] * 6, 'rack': list(cfg.RACK_INIT)},
         'world': {'cover_open': False, 'tight': True, 'uncertain': False}, 'calib': {},
         'achievable': {'col': [0.0], 'row': [-3.7, 0.0, 3.7]}, 'stamp': 0}
    s.update(kw)
    return s


FIRST = [block(1, 2.01, 'ok', dev=0.005), block(2, 1.02, 'fail', dev=-0.49), block(3, 1.99, 'ok', dev=-0.005),
         block(4, 1.68, 'warn', dev=-0.16), block(5, None, 'nodata', note='힘-변위가 직선이 아님 (다시 측정 필요)', error='bad_fit'),
         block(6, 2.00, 'ok', dev=0.0)]
TILT = {'col_deg': 0.1, 'row_deg': -0.1, 'rms': 0.1, 'n': 6}
SAMPLES = {
    'wait': None,
    'measuring': base(state='running', mode='replace', step=3, step_name='실리콘 측정', detail='3번 블록 (3/6)', active_slot=3,
                      first={'blocks': [{k: v for k, v in b.items() if k != 'grade'} for b in FIRST[:2]], 'tilt': None, 'done': False}),
    'confirm': base(state='confirm', mode='replace', step=4, step_name='판정 · 교체 계획',
                    message='교체 계획을 확인하고 "진행" 또는 "취소"를 누르세요.',
                    first={'blocks': FIRST, 'tilt': TILT, 'done': True},
                    plan={'type': 'replace', 'faulty': [2], 'skipped': [], 'done': 0,
                          'moves': ['침대 2번 (30mm) → 버림', '보관대 1번 (30mm) → 침대 2번']}),
    'done': base(state='done', mode='replace', step=12, step_name='정상 확인',
                 message='[2]번 블록을 교체했고, 재측정 결과 전부 정상입니다.',
                 first={'blocks': FIRST, 'tilt': TILT, 'done': True},
                 second={'blocks': [block(i, 2.0 + 0.01 * (i - 3), 'ok', dev=0.005 * (i - 3)) for i in range(1, 7)],
                         'tilt': TILT, 'done': True},
                 plan={'type': 'replace', 'faulty': [2], 'skipped': [], 'done': 2,
                       'moves': ['침대 2번 (30mm) → 버림', '보관대 1번 (30mm) → 침대 2번']},
                 calib={'30.0': 2.0}),
    'repair_confirm': base(
        state='confirm', mode='repair', step=4, step_name='판정 · 교체 계획',
        message='교체 계획을 확인하고 "진행" 또는 "취소"를 누르세요.',
        inventory={'bed': [30.0, 30.0, None, 30.0, 30.0, 30.0], 'rack': [30.0] * 5 + [None]},
        first={'blocks': [dict(block(1, 4.1, 'ok'), cls='hard', want='hard', ref_src='profile'),
                          dict(block(2, 3.9, 'ok'), cls='hard', want='hard', ref_src='profile'),
                          dict(block(3, None, 'empty', error='no_contact', note='30mm 내려가도 닿지 않음 (블록 없음?)')),
                          dict(block(4, 4.0, 'mismatch'), cls='hard', want='soft', ref_src='profile'),
                          dict(block(5, 4.2, 'ok'), cls='hard', want='hard', ref_src='profile'),
                          dict(block(6, 3.8, 'ok'), cls='hard', want='hard', ref_src='profile')],
               'tilt': TILT, 'done': True},
        plan={'type': 'repair', 'target_mode': 'user', 'profile': ['hard', 'hard', 'soft', 'soft', 'hard', 'hard'],
              'thr': 2.23, 'empties': [3], 'mismatch': [4], 'done': 0,
              'rack': [{'slot': 1, 'k_bed': 1.25}, {'slot': 2, 'k_bed': 5.0}, {'slot': 3, 'k_bed': 1.25},
                       {'slot': 4, 'k_bed': 5.0}, {'slot': 5, 'k_bed': 1.25}],
              'choice': [{'slot': 3, 'rack': 1, 'why': 'empty'}, {'slot': 4, 'rack': 5, 'why': 'mismatch'}],
              'moves': ['보관대 1번 (30mm) → 침대 3번', '침대 4번 (30mm) → 보관대 1번', '보관대 5번 (30mm) → 침대 4번']}),
}


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    app = QtWidgets.QApplication(sys.argv)
    samples = dict(SAMPLES)
    for path in sys.argv[2:]:
        samples[os.path.splitext(os.path.basename(path))[0]] = json.load(open(path))
    for name, status in samples.items():
        win = BedWindow(lambda cmd: None)
        if name == 'measuring':
            win.rb_tilt.setChecked(True)
            win.angle.setValue(3.0)
        if name == 'repair_confirm':
            win.rb_user.setChecked(True)
            win.set_profile(0, 'tall')
            win.profile[0].set_open(True)
            win.set_profile(2, 'soft')
            win.set_profile(3, 'soft')
        win.apply_status(status)
        for i in range(8):
            win.add_log(f'03:27:{10 + i} [3단계] 실리콘 측정 - {i % 6 + 1}번 블록')
        win.show()
        app.processEvents()
        win.grab().save(os.path.join(out, name + '.png'))
        print('saved', name, win.width(), win.height())
        win.close()


if __name__ == '__main__':
    main()
