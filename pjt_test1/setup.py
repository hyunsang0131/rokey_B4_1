from setuptools import find_packages, setup

package_name = 'pjt_test1'

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
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'read_sensor = pjt_test1.read_sensor:main',
            'move_to_teach = pjt_test1.move_to_teach:main',
            'measure_teach_force = pjt_test1.measure_teach_force:main',
            'gripper = pjt_test1.gripper:main',
            'stiffness_test = pjt_test1.stiffness_test:main',
        ],
    },
)
