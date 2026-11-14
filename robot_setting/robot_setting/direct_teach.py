# 직접 교시 (콕핏 버튼으로 로봇을 손으로 움직이고, 노트북에서 위치를 기록)
#
#   ros2 run robot_setting direct_teach
#
# 동작 방식
#   1) 로봇을 MANUAL(수동) 모드로 바꾼다. 콕핏 직접 교시는 수동 모드에서만 동작한다.
#   2) 사용자가 로봇 손목의 콕핏 버튼을 누른 채 팔을 손으로 움직인다.
#      (버튼을 누르는 동안 로봇 상태가 TEACHING(4) 이 되고, 화면에 표시된다)
#   3) 원하는 자세에서 터미널에 Enter → 현재 관절각(posj)과 TCP 위치(posx) 저장
#   4) q 입력 → JSON 파일로 저장하고, 원래 모드로 되돌린 뒤 종료
#
# 터미널 명령
#   Enter        현재 위치를 p1, p2, ... 이름으로 저장
#   이름 + Enter  그 이름으로 저장 (예: bed_c1)
#   l            저장한 목록 보기
#   u            마지막 저장 취소
#   q            파일로 저장하고 종료
#
# 한계 (두산 ROS 2 드라이버 기준)
#   - 콕핏 버튼 기능 설정(직접 교시 / TCP-Z / 자세만 등)은 드라이버에 함수가 없어서
#     펜던트 설정을 그대로 쓴다.
#   - 버튼 눌림 자체는 읽을 수 없어서, 로봇 상태가 TEACHING 인지로 판단한다.
#   - posx 는 현재 설정된 TCP 기준이다. 저장할 때 TCP 이름도 같이 기록한다.

import json
import os
import select
import sys
import time

from robot_setting.common import MODE_NAMES, STATE_STANDBY, Robot

STATE_TEACHING = 4
POLL = 0.2  # 상태 확인 주기 [s]
OUT_DIR = os.path.expanduser('~/ws_cobot_pjt/teach_points')


def fmt(v):
    return '[' + ', '.join(f'{x:.2f}' for x in v) + ']'


def main():
    robot = Robot('robot_setting_direct_teach')
    d = robot.dr
    clients = robot.base_clients() + [
        d._ros2_set_robot_mode, d._ros2_get_current_posj,
        d._ros2_get_current_posx, d._ros2_get_current_tcp]
    if not robot.wait_services(clients):
        robot.shutdown()
        return

    points = []
    orig_mode = robot.mode()
    try:
        s = robot.state()
        if s != STATE_STANDBY:
            robot.log.error(f'STANDBY 가 아닙니다 ({robot.state_name(s)}). '
                            '먼저 servo_on 을 실행하세요.')
            return

        if orig_mode != d.ROBOT_MODE_MANUAL:
            robot.log.info(f'{MODE_NAMES.get(orig_mode)} → MANUAL 모드로 전환')
            if d.set_robot_mode(d.ROBOT_MODE_MANUAL) != 0:
                robot.log.error('MANUAL 모드 전환 실패')
                return

        tcp = d.get_tcp()
        print('\n=== 직접 교시 ===')
        print('콕핏 버튼을 누른 채 팔을 움직이고, 원하는 자세에서 Enter.')
        print('Enter: 저장 | 이름+Enter: 이름으로 저장 | l: 목록 | u: 취소 | q: 종료')
        print(f'TCP: {tcp or "(없음 → 플랜지 기준)"}\n')

        teaching = False
        while True:
            # 상태 확인: 직접 교시 시작/끝을 화면에 알려준다
            s = robot.state()
            if s == STATE_TEACHING and not teaching:
                print('  >> 직접 교시 중 (TEACHING)')
                teaching = True
            elif s != STATE_TEACHING and teaching:
                print('  >> 직접 교시 끝')
                teaching = False
            elif s not in (STATE_STANDBY, STATE_TEACHING):
                print(f'  !! 상태: {robot.state_name(s)}')
                print(f'  !! 알람: {robot.alarm_text()}')
                time.sleep(1.0)

            # 키 입력 대기 (POLL 초 동안만, 없으면 다시 상태 확인)
            ready, _, _ = select.select([sys.stdin], [], [], POLL)
            if not ready:
                continue
            cmd = sys.stdin.readline().strip()

            if cmd == 'q':
                break
            if cmd == 'l':
                for p in points:
                    print(f'  {p["name"]:10s} posj={fmt(p["posj"])}')
                    print(f'  {"":10s} posx={fmt(p["posx"])}')
                if not points:
                    print('  (저장한 위치 없음)')
                continue
            if cmd == 'u':
                if points:
                    print(f'  취소: {points.pop()["name"]}')
                continue

            if teaching:
                print('  콕핏 버튼을 놓고 로봇이 멈춘 뒤 저장하세요.')
                continue
            name = cmd or f'p{len(points) + 1}'
            posj = list(d.get_current_posj())
            posx, _ = d.get_current_posx(ref=d.DR_BASE)
            points.append({'name': name, 'posj': posj, 'posx': list(posx),
                           'tcp': tcp, 'time': time.strftime('%H:%M:%S')})
            print(f'  저장 {name}: posj={fmt(posj)}')
            print(f'  {"":{len(name) + 5}s}posx={fmt(posx)}')

    except KeyboardInterrupt:
        print()
    finally:
        if points:
            os.makedirs(OUT_DIR, exist_ok=True)
            fname = os.path.join(OUT_DIR, time.strftime('teach_%Y%m%d_%H%M%S.json'))
            with open(fname, 'w') as f:
                json.dump(points, f, indent=2, ensure_ascii=False)
            print(f'\n{len(points)}개 저장: {fname}')
            print('코드에 붙여 쓸 형식:')
            for p in points:
                print(f'  {p["name"]} = posj({fmt(p["posj"])})')
                print(f'  {p["name"]}_x = posx({fmt(p["posx"])})')

        if robot.mode() != orig_mode:
            robot.log.info(f'원래 모드({MODE_NAMES.get(orig_mode)})로 복귀')
            d.set_robot_mode(orig_mode)
        robot.shutdown()


if __name__ == '__main__':
    main()
