# 명령 하나로 전부: 로봇 bringup → 공정·연결·주문 서버·화면 → 고른 작업을 9단계 파이프라인으로 실행
#
#   작업 job:=inspect   공정 전체: 측정 → 판정 → 커버 열기 → 교체 → 커버 닫기 → 재측정 → 확인 (자동 보수)
#   작업 job:=order     주문대로 구성: 웹 주문(http://<이 PC IP>:5000/)을 기다렸다가 → 같은 9단계로 주문대로 교체
#                       (order_file:=주문.json 을 주면 웹 대신 그 파일로 바로)
#   로봇 robot:=virtual (에뮬레이터 + RViz)  /  robot:=real (실물, teach_file 필요)
#
#   ros2 launch bed_inspect bed_all.launch.py job:=inspect robot:=virtual
#   ros2 launch bed_inspect bed_all.launch.py job:=order   robot:=virtual
#   ros2 launch bed_inspect bed_all.launch.py job:=inspect robot:=real     # 교시: teach_points/bed_fin.json + exchange_fin.json (팀원 파일)
#   ros2 launch bed_inspect bed_all.launch.py job:=order   robot:=real
#
#   ※ 실물 bringup 이 이미 켜져 있는데 가상을 같이 띄우려면 먼저 export ROS_DOMAIN_ID=77 (이름 dsr01 이 겹치지 않게)
#
# 인자
#   job:=none                 띄우기만 (작업은 나중에 bed_cmd 로)
#   replace:=true             첫 공정을 '기준값 대비 불량 교체'로 (기본: 자동 보수)
#   robot_ip:=192.168.1.100   실물 컨트롤러 IP,  robot_port:=12345
#   emu_force:=... quick:=true   가상 모드 가짜 힘 / 빠른 누르기 (cushion_system.launch.py 참고)
#   order_port:=5000          웹 주문 포트
#   bringup:=false            bringup 을 이미 따로 켜 두었으면
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource


TEACH_DIR = os.path.expanduser('~/ws_cobot_pjt/teach_points')


def _robot_running():
    import rclpy
    import time
    rclpy.init()
    node = rclpy.create_node('bed_all_probe')
    try:
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            names = [n for n, _ in node.get_service_names_and_types()]
            if '/dsr01/dsr_controller2/system/get_robot_state' in names:
                return True
            time.sleep(0.3)
        return False
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _yes(v):
    return str(v).strip().lower() in ('1', 'true', 'yes', 'on')


def _setup(context):
    get = lambda k: context.launch_configurations.get(k, '')           # noqa: E731
    mode = get('robot') or get('mode')              # mode:= 는 예전 이름 (robot:= 과 같음)
    if mode not in ('virtual', 'real'):
        raise RuntimeError('robot:=virtual 또는 robot:=real 을 주세요')
    job = get('job')
    if job not in ('inspect', 'order', 'none'):
        raise RuntimeError('job:=inspect (공정 전체) 또는 job:=order (주문대로 구성) 을 주세요')
    virtual = mode == 'virtual'
    teach = get('teach_file')
    if not virtual and not teach:
        # 기본: 팀원이 실물에서 찍은 교시 파일 두 개 (침대 pin_b1~6, 교체 트레이 tray_b1~6)
        teach = ','.join(os.path.join(TEACH_DIR, f) for f in ('bed_fin.json', 'exchange_fin.json'))
    if not virtual:
        missing = [f for f in teach.split(',') if not os.path.exists(os.path.expanduser(f.strip()))]
        if missing:
            raise RuntimeError(f'교시 파일이 없습니다: {missing} (팀원 bed_fin.json, exchange_fin.json 을 '
                               f'{TEACH_DIR} 에 넣거나 teach_file:= 로 주세요)')
    actions = []
    bringup = get('bringup')
    if bringup == 'auto':
        # 로봇 bringup 이 이미 떠 있으면(같은 ROS_DOMAIN_ID) 또 띄우지 않는다 (두 개면 컨트롤러가 충돌함)
        bringup = 'false' if _robot_running() else 'true'
        print(f'[bed_all] bringup 이 이미 {"떠 있어 그대로 씀" if bringup == "false" else "없어 새로 띄움"}')
    if _yes(bringup):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('m0609_rg2_bringup'), 'launch', 'bringup.launch.py')),
            launch_arguments={'mode': mode, 'host': '127.0.0.1' if virtual else get('robot_ip'),
                              'port': get('robot_port')}.items()))
    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('bed_inspect'), 'launch', 'cushion_system.launch.py')),
        launch_arguments={'virtual': 'true' if virtual else 'false', 'teach_file': teach,
                          'emu_force': get('emu_force'), 'quick': get('quick'),
                          'port': get('order_port'), 'auto_start': 'false'}.items())
    after = [system]
    if job == 'inspect':
        cmd = ['ros2', 'run', 'bed_inspect', 'bed_cmd', 'inspect', '--ready-timeout', '180']
        if _yes(get('replace')):
            cmd.append('--replace')
        after.append(ExecuteProcess(cmd=cmd, output='screen', name='job_inspect'))
    elif job == 'order':
        cmd = ['ros2', 'run', 'bed_inspect', 'bed_cmd', 'order', '--ready-timeout', '180']
        if get('order_file'):
            cmd.insert(5, os.path.expanduser(get('order_file')))
        else:
            cmd.append('--wait')                        # 켠 뒤에 들어오는 웹 주문을 기다린다
        after.append(ExecuteProcess(cmd=cmd, output='screen', name='job_order'))
    wait = ExecuteProcess(cmd=['ros2', 'run', 'bed_inspect', 'bed_cmd', 'wait_robot', '--ready-timeout', '180'],
                          output='screen', name='wait_robot')

    def on_ready(event, ctx):
        if event.returncode != 0:
            return [LogInfo(msg='로봇이 준비되지 않아 공정을 띄우지 않습니다. Ctrl+C 로 끄고 다시 실행하세요.')]
        return after
    actions += [wait, RegisterEventHandler(OnProcessExit(target_action=wait, on_exit=on_ready))]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('job', default_value='inspect', description='inspect (공정 전체, 기본) / order (주문대로 구성) / none'),
        DeclareLaunchArgument('robot', default_value='', description='virtual (에뮬레이터 + RViz) 또는 real (실물)'),
        DeclareLaunchArgument('mode', default_value='', description='robot 과 같음 (예전 이름)'),
        DeclareLaunchArgument('order_file', default_value='', description='job:=order 에서 웹 대신 쓸 주문 JSON'),
        DeclareLaunchArgument('teach_file', default_value='', description='실물 교시 파일 (가상은 비워 두면 bed_emu.json)'),
        DeclareLaunchArgument('replace', default_value='false', description='첫 공정을 기준값 대비 불량 교체로'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.100'),
        DeclareLaunchArgument('robot_port', default_value='12345'),
        DeclareLaunchArgument('bringup', default_value='auto',
                              description='auto = 이미 떠 있으면 그대로 씀, true = 새로 띄움, false = 따로 켠 것으로 봄'),
        DeclareLaunchArgument('emu_force', default_value='bed=H,H,H,S,H,H;rack=S,S,H,H,H,-',
                              description='가상 모드 큐브 종류 (H 단단 / S 무름 / T 키 큰 무름 / - 빈칸)'),
        DeclareLaunchArgument('quick', default_value='true', description='가상 모드에서 누르기를 빠르게'),
        DeclareLaunchArgument('order_port', default_value='5000', description='웹 주문 서버 포트'),
        OpaqueFunction(function=_setup),
    ])
