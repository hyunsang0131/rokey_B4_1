from setuptools import find_packages, setup

package_name = 'robot_setting'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BL',
    maintainer_email='mail@mail.com',
    description='Teach pendant equivalent functions for Doosan M0609 (home, align, servo on/off, direct teaching, tool/TCP setup, tool weight)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'status = robot_setting.status:main',
            'home = robot_setting.home:main',
            'servo_on = robot_setting.servo:servo_on',
            'servo_off = robot_setting.servo:servo_off',
            'direct_teach = robot_setting.direct_teach:main',
            'tool_tcp = robot_setting.tool_tcp:main',
        ],
    },
)
