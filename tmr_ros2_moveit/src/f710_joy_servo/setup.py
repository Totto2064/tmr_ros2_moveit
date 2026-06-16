from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'f710_joy_servo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moto',
    maintainer_email='moto@todo.todo',
    description='F710 joystick to MoveIt Servo bridge',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'joy_to_servo = f710_joy_servo.joy_to_servo:main',
            'servo_trajectory_bridge = f710_joy_servo.servo_trajectory_bridge:main',
        ],
    },
)
