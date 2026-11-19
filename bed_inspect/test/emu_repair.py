# 에뮬레이터(sodvir)에서 자동 보수를 끝까지 돌려 보는 스크립트. 힘은 bed_emu 의 가짜 힘을 쓴다.
#
#   1) sodvir                                   (다른 터미널, RViz 에서 움직임을 본다)
#   2) ros2 run bed_inspect emu_teach           (교시 파일 bed_emu.json, 한 번만)
#   3) python3 test/emu_repair.py [auto|user|all] [로그폴더]
#
# 실행마다 에뮬레이터용 재고 기록(~/ws_cobot_pjt/bed_data_emu)을 처음 상태로 되돌리고 bed_process 를 새로 띄운다.
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

TEACH = os.path.expanduser('~/ws_cobot_pjt/teach_points/bed_emu.json')
DATA = os.path.expanduser('~/ws_cobot_pjt/bed_data_emu')
LATCH = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

CASES = {
    # 실제 준비물 구성: 40A 6개, 10A 2개, 키 큰 10A 2개. 침대 3번 빈칸, 4번 10A / 보관대 T,H,S,H,T,빈자리
    # 자동 보수: 3번 빈칸, 4번(10A)이 주변과 맞지 않음 → 단단한 큐브 2개
    'auto': ('bed=H,H,-,S,H,H;rack=T,H,S,H,T,-', {'target': 'auto'}, [3], [4]),
    # 고객 맞춤: 1·2번 높게, 3·4번 부드럽게 → 1·2번을 키 큰 큐브로, 3번 빈칸에 부드러운 큐브 (4번은 이미 부드러움)
    'user': ('bed=H,H,-,S,H,H;rack=T,H,S,H,T,-',
             {'target': 'user', 'profile': ['tall', 'tall', 'soft', 'soft', 'hard', 'hard']}, [3], [1, 2]),
}


def reset_data():
    os.makedirs(DATA, exist_ok=True)
    for name, data in (('inventory.json', {'bed': [30.0] * 6, 'rack': [30.0] * 5 + [None]}),
                       ('world.json', {'cover_open': False, 'tight': True, 'uncertain': False}),
                       ('calib.json', {})):
        with open(os.path.join(DATA, name), 'w') as f:
            json.dump(data, f)


def servo_on(node):
    """에뮬레이터가 SAFE_OFF 면 서보를 켠다."""
    from dsr_msgs2.srv import GetRobotState, SetRobotControl
    get = node.create_client(GetRobotState, '/dsr01/dsr_controller2/system/get_robot_state')
    ctl = node.create_client(SetRobotControl, '/dsr01/dsr_controller2/system/set_robot_control')
    for c in (get, ctl):
        if not c.wait_for_service(timeout_sec=20.0):
            raise SystemExit(f'서비스 없음: {c.srv_name} (sodvir 가 떠 있나요?)')
    for _ in range(20):
        f = get.call_async(GetRobotState.Request())
        rclpy.spin_until_future_complete(node, f, timeout_sec=3.0)
        st = f.result().robot_state if f.result() else -1
        if st == 1:
            return
        if st in (3, 10):                       # SAFE_OFF
            req = SetRobotControl.Request()
            req.robot_control = 3               # CONTROL_SERVO_ON
            f = ctl.call_async(req)
            rclpy.spin_until_future_complete(node, f, timeout_sec=3.0)
            print('서보 켜는 중...', flush=True)
        time.sleep(1.0)
    raise SystemExit(f'로봇이 STANDBY 가 아닙니다 (상태 {st})')


def run_case(node, name, logdir):
    spec, cmd, want_empty, want_mis = CASES[name]
    reset_data()
    env = dict(os.environ, BED_EMU_FORCE=spec, BED_QUICK='1', BED_DATA_DIR=DATA)
    log = open(os.path.join(logdir, f'emu_{name}_process.log'), 'w')
    proc = subprocess.Popen(['ros2', 'run', 'bed_inspect', 'bed_process', TEACH], env=env,
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    box = {'st': None, 'logs': []}
    s1 = node.create_subscription(String, '/bed/status', lambda m: box.__setitem__('st', json.loads(m.data)), LATCH)
    s2 = node.create_subscription(String, '/bed/log', lambda m: (box['logs'].append(m.data), print('  ' + m.data, flush=True)), 50)
    pub = node.create_publisher(String, '/bed/cmd', 10)

    def wait(cond, timeout, what):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if cond():
                return
            if proc.poll() is not None:
                raise AssertionError(f'{what}: bed_process 가 죽었습니다')
        raise AssertionError(f'{what}: 시간 초과 ({box["st"] and box["st"]["state"]})')

    t0 = time.monotonic()
    ok = False
    try:
        wait(lambda: box['st'] and box['st']['state'] in ('idle', 'error'), 60, '준비')
        assert box['st']['state'] == 'idle', box['st']['message']
        wait(lambda: pub.get_subscription_count() > 0, 60, '명령 연결')
        stamp = box['st']['stamp']
        pub.publish(String(data=json.dumps(dict(cmd='start', mode='repair', **cmd))))
        wait(lambda: box['st']['stamp'] > stamp and box['st']['state'] == 'running', 15, '시작')
        wait(lambda: box['st']['state'] != 'running', 1800, '실행')
        st = box['st']
        plan = st['plan'] or {}
        g1 = [b['grade'] for b in (st['first'] or {}).get('blocks', [])]
        g2 = [b['grade'] for b in (st['second'] or {}).get('blocks', [])]
        print(f'\n[{name}] 결과: {st["state"]} - {st["message"]}')
        print(f'  1차 판정: {g1}\n  재측정  : {g2}')
        print(f'  빈칸 {plan.get("empties")}, 맞지 않음 {plan.get("mismatch")}, 이동 {plan.get("done")}/{len(plan.get("moves") or [])}')
        for m in plan.get('moves') or []:
            print('   ', m)
        print(f'  재고: {st["inventory"]}  소요 {time.monotonic() - t0:.0f}s', flush=True)
        ok = (st['state'] == 'done' and plan.get('empties') == want_empty and plan.get('mismatch') == want_mis
              and g2 == ['ok'] * 6)
        with open(os.path.join(logdir, f'emu_{name}_status.json'), 'w') as f:
            json.dump(st, f, ensure_ascii=False, indent=1)
    finally:
        node.destroy_subscription(s1)
        node.destroy_subscription(s2)
        node.destroy_publisher(pub)
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        log.close()
    return ok


def main():
    which = sys.argv[1] if len(sys.argv) > 1 else 'all'
    logdir = sys.argv[2] if len(sys.argv) > 2 else '/tmp/bed_emu'
    os.makedirs(logdir, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('emu_repair_driver')
    servo_on(node)
    results = {}
    for name in (CASES if which == 'all' else [which]):
        print(f'\n===== {name} =====', flush=True)
        try:
            results[name] = run_case(node, name, logdir)
        except AssertionError as e:
            print(f'[{name}] 실패: {e}', flush=True)
            results[name] = False
    print('\n요약:', {k: ('통과' if v else '실패') for k, v in results.items()}, flush=True)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all(results.values()) else 1)


if __name__ == '__main__':
    main()
