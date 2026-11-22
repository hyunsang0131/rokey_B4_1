from setuptools import find_packages, setup

package_name = 'bed_inspect'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cushion_system.launch.py', 'launch/bed_all.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='실리콘 블록 침대 검사 공정',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'bed_process = bed_inspect.bed_process:main',   # 공정 노드 (로봇을 움직임)
            'bed_hmi = bed_inspect.bed_hmi:main',           # 히트맵 화면
            'bed_sim = bed_inspect.bed_sim:main',           # 가짜 로봇 + 가짜 침대 (연습/검증용)
            'press_test = bed_inspect.press_test:main',     # 블록 1개씩 누르기 테스트
            'cover_demo = bed_inspect.cover_demo:main',     # 커버 열기/닫기만 따로 (RViz 확인용)
            'emu_teach = bed_inspect.emu_teach:main',
            'cushion_bridge = bed_inspect.cushion_bridge:main',
            'string_v1 = bed_inspect.string_v1_demo:main', # 실 고정 v1: 영상 전 (30/20, 180도)
            'bed_cmd = bed_inspect.bed_cmd:main',             # 터미널에서 공정 명령 (inspect / order / ...)   # cushion_hmi 화면/웹 주문 ↔ 공정 연결       # 에뮬레이터(sodvir)용 가짜 교시 파일 만들기
        ],
    },
)
