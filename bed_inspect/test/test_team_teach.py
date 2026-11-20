# 팀원 교시 파일(pin_b / tray_b + _down/_up) 읽기 테스트
import json

from bed_inspect import bed_store


def _pt(name, x, y, z):
    return {'name': name, 'posj': [0.0] * 6, 'posx': [x, y, z, 90.0, 180.0, 78.23], 'tcp': 'GripperDA_v1'}


def test_team_files(tmp_path):
    bed = [p for n in range(1, 7) for p in (_pt(f'pin_b{n}', 500 + n, -170, 200), _pt(f'pin_b{n}_down', 500 + n, -170, 130),
                                             _pt(f'pin_b{n}_up', 500 + n, -170, 210))]
    tray = [p for n in range(1, 7) for p in (_pt(f'tray_b{n}', 450 + n, 120, 200), _pt(f'tray_b{n}_down', 450 + n, 120, 125))]
    (tmp_path / 'bed_fin.json').write_text(json.dumps(bed))
    (tmp_path / 'exchange_fin.json').write_text(json.dumps(tray))
    pts = bed_store.load_teach(f'{tmp_path}/bed_fin.json,{tmp_path}/exchange_fin.json')
    assert bed_store.missing_names(pts) == []
    assert pts['bed_3']['posx'][0] == 503 and pts['bed_3']['down'][2] == 130 and pts['bed_3']['up'][2] == 210
    assert pts['rack_2']['down'][2] == 125 and 'up' not in pts['rack_2']
    pts = bed_store.fill_defaults(pts)
    assert pts['home']['posj'] == [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
