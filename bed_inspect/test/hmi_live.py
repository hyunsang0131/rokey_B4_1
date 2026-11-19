# 화면 연동 검증: 가짜 로봇 + 공정 노드를 띄우고, 실제 화면의 버튼을 프로그램으로 눌러 끝까지 돌린다.
# 단계마다 화면을 PNG 로 저장한다.
#
#   QT_QPA_PLATFORM=offscreen python3 test/hmi_live.py 출력폴더           # 정지, 불량 교체, 기울기
#   QT_QPA_PLATFORM=offscreen python3 test/hmi_live.py 출력폴더 repair    # 자동 보수 (사용자 지정)
import os
import signal
import subprocess
import sys
import time

from bed_inspect import bed_config as cfg
from bed_inspect.bed_hmi import InventoryDialog, make_app

OUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/hmi_live'
REPAIR = len(sys.argv) > 2 and sys.argv[2] == 'repair'
os.makedirs(OUT, exist_ok=True)
SIM_ARGS = (['--bed', '30,30,-,30,30,30', '--bed-kind', 'H,H,-,H,H,H',
             '--rack', f'{cfg.TALL_HEIGHT:g},30,30,30,{cfg.TALL_HEIGHT:g},-', '--rack-kind', 'S,H,S,H,S,-'] if REPAIR
            else ['--fault', '2', '--rack', '30,25,25,35,35,-'])


def run_repair(app, win, wait, state, shot):
    """고객 맞춤: 1·2번 높게, 3번 부드럽게 → 1·2번(단단함)을 키 큰 큐브로, 3번(빈칸)에 부드러운 큐브."""
    win.rb_user.setChecked(True)
    app.processEvents()
    assert win.repair_box.isVisible() and not win.profile[0].body.isVisible()
    for i, v in ((0, 'tall'), (1, 'tall'), (2, 'soft')):
        win.profile[i].head.click()                     # ▸ 토글 펼치기
        app.processEvents()
        assert win.profile[i].body.isVisible()
        win.profile[i].opts[v].click()
    assert win.profile_values() == ['tall', 'tall', 'soft', 'hard', 'hard', 'hard'], win.profile_values()
    assert '높게' in win.profile[0].head.text()
    win.confirm.setChecked(True)
    win.b_start.click()
    wait(lambda: state() == 'confirm', 200, '확인 대기')
    assert '○ 빈칸' in win.map1.cells[2].grade.text(), win.map1.cells[2].grade.text()
    assert '≠ 맞지 않음' in win.map1.cells[0].grade.text(), win.map1.cells[0].grade.text()
    assert '목표 높게' in win.map1.cells[0].line2.text(), win.map1.cells[0].line2.text()
    assert '(교체)' in win.plan_list.text() and '(빈칸)' in win.plan_list.text(), win.plan_list.text()
    shot('r1_confirm')
    win.b_go.click()
    wait(lambda: state() in ('done', 'warning', 'error', 'stopped'), 300, '보수 완료')
    assert state() == 'done', win.status['message']
    assert all('✓ 정상' in c.grade.text() for c in win.map2.cells)
    shot('r2_done')


def main():
    logs = [open(os.path.join(OUT, n), 'w') for n in ('sim.log', 'process.log')]
    sim = subprocess.Popen(['ros2', 'run', 'bed_inspect', 'bed_sim', *SIM_ARGS, '--speed', '40', '--seed', '3'],
                           stdout=logs[0], stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(2.0)
    proc = subprocess.Popen(['ros2', 'run', 'bed_inspect', 'bed_process', cfg.SIM_TEACH_FILE],
                            stdout=logs[1], stderr=subprocess.STDOUT, start_new_session=True)
    app, win, node, timer = make_app()
    win.show()

    def wait(cond, timeout, what):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            app.processEvents()
            time.sleep(0.01)
            if cond():
                return
        st = win.status
        raise AssertionError(f'{what}: 시간 초과. 상태={st and st["state"]} 단계={st and st["step"]} {st and st["message"]}')

    def state():
        return win.status['state'] if win.status else None

    def shot(name):
        app.processEvents()
        win.grab().save(os.path.join(OUT, name + '.png'))
        print('화면 저장:', name, flush=True)

    try:
        wait(lambda: state() == 'idle', 40, '준비')
        assert win.b_start.isEnabled() and not win.b_stop.isEnabled()
        wait(lambda: node.count_subscribers('/bed/cmd') > 0, 10, '명령 연결')
        if REPAIR:
            run_repair(app, win, wait, state, shot)
        else:
            run_main(app, win, wait, state, shot)
        print('화면 연동 검증: 통과', flush=True)
        code = 0
    except AssertionError as e:
        print('화면 연동 검증: 실패 -', e, flush=True)
        shot('fail')
        code = 1
    finally:
        for p in (proc, sim):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
    sys.exit(code)


def run_main(app, win, wait, state, shot):
    """정지, 불량 교체(확인 포함), 기울기, 재고 수정 창."""

    # 1) 정지 버튼: 측정 중에 누르면 멈춘다
    win.rb_measure.setChecked(True)
    win.b_start.click()
    wait(lambda: state() == 'running' and win.status['step'] == 3 and win.status['active_slot'] == 2, 60, '측정 시작')
    assert win.b_stop.isEnabled() and not win.b_start.isEnabled()
    win.b_stop.click()
    wait(lambda: state() == 'stopped', 20, '정지')

    # 2) 불량 교체 모드 + 교체 전 확인
    win.rb_replace.setChecked(True)
    win.confirm.setChecked(True)
    win.b_start.click()
    wait(lambda: state() == 'running' and win.status['step'] == 3 and win.status['active_slot'] == 4, 60, '측정')
    shot('1_measuring')
    wait(lambda: state() == 'confirm', 120, '확인 대기')
    assert win.b_go.isVisible() and win.b_cancel.isVisible()
    assert '✕ 불량' in win.map1.cells[1].grade.text()
    shot('2_confirm')
    win.b_go.click()
    wait(lambda: state() == 'running' and win.status['step'] == 8, 120, '블록 넣기')
    shot('3_swapping')
    wait(lambda: state() in ('done', 'warning', 'error', 'stopped'), 300, '교체 완료')
    assert state() == 'done', win.status['message']
    assert all('✓ 정상' in c.grade.text() for c in win.map2.cells)
    assert not win.b_go.isVisible()
    shot('4_replace_done')

    # 3) 기울기 맞춤 모드: 1→3번 방향 3도 (행 간격 77.5mm, 5mm 차이 → 3.7도)
    win.rb_tilt.setChecked(True)
    win.confirm.setChecked(False)
    win.axis.setCurrentIndex(win.axis.findData('row'))
    win.angle.setValue(3.0)
    app.processEvents()
    assert '+3.7°' in win.hint.text(), win.hint.text()
    win.b_start.click()
    wait(lambda: state() == 'running', 10, '기울기 시작')
    wait(lambda: state() in ('done', 'warning', 'error', 'stopped'), 400, '기울기 완료')
    assert state() == 'done', win.status['message']
    assert win.status['inventory']['bed'] == [25.0, 25.0, 30.0, 30.0, 35.0, 35.0]
    assert '+3.7°' in win.map2.tilt.text() or '+3.6°' in win.map2.tilt.text(), win.map2.tilt.text()
    shot('5_tilt_done')

    # 4) 재고 수정 창: 지금 재고가 그대로 들어 있는지
    dlg = InventoryDialog(win, win.status)
    cmd = dlg.command()
    assert cmd['bed'] == win.status['inventory']['bed'] and cmd['rack'] == win.status['inventory']['rack'], cmd
    assert cmd['tight'] is True and cmd['cover_open'] is False
    dlg.show()
    app.processEvents()
    dlg.grab().save(os.path.join(OUT, '6_inventory_dialog.png'))
    dlg.close()
    assert len(win.log.toPlainText().splitlines()) > 50


if __name__ == '__main__':
    main()
