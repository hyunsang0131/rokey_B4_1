from glob import glob

from setuptools import find_packages, setup

package_name = 'cushion_hmi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BL',
    maintainer_email='mail@mail.com',
    description='협동1 쿠션 진단 HMI (PyQt5) 와 모의 미션 노드',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hmi = cushion_hmi.hmi:main',
            'mock_mission = cushion_hmi.mock_mission:main',
            'order_server = cushion_hmi.order_server:main',
        ],
    },
)
