# cushion_bridge 변환 함수 테스트 (ROS 없이)
import pytest

from bed_inspect import cushion_bridge as cb

ORDER = {'order_id': '20261119-1', 'slots': {'Slot 1': '부드러움', 'Slot 2': '단단함', 'Slot 3': '단단함',
                                             'Slot 4': '부드러움', 'Slot 5': '단단함', 'Slot 6': '단단함'}}


def test_order_to_profile():
    assert cb.order_to_profile(ORDER, [1, 2, 3, 4, 5, 6]) == ['soft', 'hard', 'hard', 'soft', 'hard', 'hard']
    # Slot 1 → 공정 2번 칸 처럼 바꿔 끼우기
    assert cb.order_to_profile(ORDER, [2, 1, 4, 3, 6, 5]) == ['hard', 'soft', 'soft', 'hard', 'hard', 'hard']
    with pytest.raises(ValueError):
        cb.order_to_profile({'slots': {'Slot 1': '중간'}}, [1, 2, 3, 4, 5, 6])


def test_cmd_for():
    assert cb.cmd_for('diagnose', None) == {'cmd': 'start', 'mode': 'measure'}
    assert cb.cmd_for('full', None) == {'cmd': 'start', 'mode': 'replace'}
    assert cb.cmd_for('calibrate', None) == {'cmd': 'calibrate'}
    assert cb.cmd_for('order', ['soft'] * 6) == {'cmd': 'start', 'mode': 'repair', 'target': 'user',
                                                 'profile': ['soft'] * 6}
    with pytest.raises(ValueError):
        cb.cmd_for('order', None)


def test_status_mapping():
    bed = {'state': 'running', 'mode': 'repair', 'step': 8, 'active_slot': 3,
           'first': {'blocks': [{'slot': i, 'k': 5.0, 'grade': 'ok'} for i in range(1, 7)], 'done': True},
           'second': {'blocks': [{'slot': 1, 'k': 2.0, 'grade': 'mismatch', 'z_surf': 101.0},
                                 {'slot': 2, 'k': None, 'grade': 'empty'}], 'done': False},
           'plan': {'target_mode': 'user'}, 'inventory': {'rack': [30.0, None, 30.0]}, 'message': ''}
    s = cb.to_cushion_status(bed, [1, 2, 3, 4, 5, 6], ORDER, False)
    assert s['state'] == 'REMEASURE' and s['running'] and s['mode'] == 'order' and s['spares'] == 2
    st = [m['status'] for m in s['modules']]
    assert st[:3] == ['mismatch', 'empty', 'measuring'] and st[3] == 'unknown'
    assert s['modules'][0]['z0'] == 101.0 and s['current_module'] == 3
    s = cb.to_cushion_status(dict(bed, state='confirm', message='확인하세요'), [1, 2, 3, 4, 5, 6])
    assert s['state'] == 'CONFIRM' and s['wait_operator'] and s['prompt'] == '확인하세요'
    s = cb.to_cushion_status(dict(bed, state='error', message='오류'), [1, 2, 3, 4, 5, 6])
    assert s['state'] == 'FAULT' and s['alarm'] == '오류' and not s['running']


def test_order_tall_from_web():
    """웹 HIGH(키 큰 큐브) → '높게' → 공정 profile 'tall'."""
    order = {'order_id': 'TW-1', 'slots': {'Slot 1': '높게', 'Slot 2': '높게', 'Slot 3': '단단함',
                                           'Slot 4': '단단함', 'Slot 5': '부드러움', 'Slot 6': '단단함'}}
    assert cb.order_to_profile(order, [1, 2, 3, 4, 5, 6]) == ['tall', 'tall', 'hard', 'hard', 'soft', 'hard']
