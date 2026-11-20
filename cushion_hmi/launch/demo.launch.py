# 로봇 없이 HMI 를 시험: 모의 미션 노드 + HMI 를 같이 띄운다
#
#   ros2 launch cushion_hmi demo.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='cushion_hmi', executable='mock_mission', output='screen',
             parameters=[{'defects': [4], 'spares': 2, 'bad_spares': 0}]),
        Node(package='cushion_hmi', executable='hmi', output='screen'),
    ])
