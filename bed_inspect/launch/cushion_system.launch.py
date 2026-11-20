# 웹 주문 → 자동 구성 한 번에 띄우기 (로봇 bringup 은 따로 먼저 켠다)
#
# 실물:
#   ros2 launch bed_inspect cushion_system.launch.py teach_file:=$HOME/ws_cobot_pjt/teach_points/bed.json
# 가상 (에뮬레이터 + RViz, 로봇 없이 전 과정):
#   ros2 launch m0609_rg2_bringup bringup.launch.py mode:=virtual      ← 먼저 (실물 bringup 은 끈다)
#   ros2 launch bed_inspect cushion_system.launch.py virtual:=true
#     - 교시 파일: teach_points/bed_emu.json (없으면 emu_teach 로 에뮬레이터에서 자동으로 만든다)
#     - 힘: 에뮬레이터에는 힘 센서가 없어서 bed_emu 의 가짜 힘 (emu_force 로 침대/보관대 큐브 종류 지정)
#     - 기록: ~/ws_cobot_pjt/bed_data_emu (실물 기록과 따로, 시작할 때마다 처음 상태로)
#     - quick:=true 면 누르기를 빠르게 (동작 확인용, 강성 값은 대충)
#
# 띄운 뒤 작업은 명령으로 (다른 터미널):
#   ros2 run bed_inspect bed_cmd inspect      ① 첫 공정 전체 (측정 → 판정 → 커버 → 교체 → 재측정)
#   ros2 run bed_inspect bed_cmd order        ② 주문대로 구성 (웹 주문 또는 JSON 파일)
#
# 띄우는 것: bed_process(9단계 공정), cushion_bridge(연결), order_server(웹 주문, host:port), cushion_hmi hmi(화면)
import json
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

EMU_TEACH = os.path.expanduser('~/ws_cobot_pjt/teach_points/bed_emu.json')
EMU_DATA = os.path.expanduser('~/ws_cobot_pjt/bed_data_emu')


def _kinds_to_inventory(spec, tall=40.0, normal=30.0):
    """BED_EMU_FORCE 문자열 → 재고 기록 (H/S = 보통 높이, T = 키 큰 큐브, - = 빈칸)."""
    parts = dict(p.split('=', 1) for p in spec.split(';') if '=' in p)

    def conv(text, n):
        out = [None if k.strip() == '-' else (tall if k.strip().upper() == 'T' else normal) for k in text.split(',')]
        return (out + [None] * n)[:n]
    return {'bed': conv(parts.get('bed', ''), 6), 'rack': conv(parts.get('rack', ''), 6)}


def _setup(context):
    get = lambda k: context.launch_configurations.get(k, '')           # noqa: E731
    virtual = get('virtual').lower() in ('1', 'true', 'yes')
    teach = os.path.expanduser(get('teach_file') or (EMU_TEACH if virtual else ''))
    if not teach:
        raise RuntimeError('teach_file:=교시파일 을 주세요 (가상 모드는 virtual:=true)')
    env = {}
    if virtual:
        env = {'BED_VIRTUAL': '1', 'BED_EMU_FORCE': get('emu_force'), 'BED_DATA_DIR': EMU_DATA,
               'BED_QUICK': '1' if get('quick').lower() in ('1', 'true', 'yes') else '0'}
        os.makedirs(EMU_DATA, exist_ok=True)
        for name, data in (('inventory.json', _kinds_to_inventory(get('emu_force'))),
                           ('world.json', {'cover_open': False, 'uncertain': False})):
            with open(os.path.join(EMU_DATA, name), 'w') as f:
                json.dump(data, f)
    nodes = [
        Node(package='bed_inspect', executable='bed_process', arguments=[teach], output='screen',
             additional_env=env),
        Node(package='bed_inspect', executable='cushion_bridge', output='screen',
             parameters=[{'auto_start': get('auto_start').lower() in ('1', 'true', 'yes')}]),
        Node(package='cushion_hmi', executable='order_server', output='screen',
             parameters=[{'host': get('host'), 'port': int(get('port'))}]),
        Node(package='cushion_hmi', executable='hmi', output='screen'),
    ]
    if virtual and not os.path.exists(teach):
        # 에뮬레이터에서 가짜 교시 파일을 먼저 만들고, 끝나면 나머지를 띄운다
        maker = ExecuteProcess(cmd=['ros2', 'run', 'bed_inspect', 'emu_teach', teach], output='screen')
        return [maker, RegisterEventHandler(OnProcessExit(target_action=maker, on_exit=nodes))]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('teach_file', default_value='', description='교시 파일 (가상 모드는 비워 두면 bed_emu.json)'),
        DeclareLaunchArgument('virtual', default_value='false', description='에뮬레이터 + RViz 가상 모드'),
        DeclareLaunchArgument('emu_force', default_value='bed=H,H,H,H,H,H;rack=S,S,S,H,H,-',
                              description='가상 모드 큐브 종류 (H 단단 / S 무름 / T 키 큰 무름 / - 빈칸)'),
        DeclareLaunchArgument('quick', default_value='true', description='가상 모드에서 누르기를 빠르게'),
        DeclareLaunchArgument('auto_start', default_value='false',
                              description='주문이 오면 바로 시작 (기본 false: bed_cmd order 로 시작)'),
        DeclareLaunchArgument('port', default_value='5000', description='주문 서버 포트'),
        DeclareLaunchArgument('host', default_value='0.0.0.0', description='주문 서버 주소 (0.0.0.0 = 모든 네트워크)'),
        OpaqueFunction(function=_setup),
    ])
