from setuptools import setup
from glob import glob
import os

package_name = 'rover_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Moonrockers Team',
    maintainer_email='team@moonrockers.com',
    description='ROS2 package for rover robot control and teleoperation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = rover_control.motor_controller:main',
            'teleop_keyboard = rover_control.teleop_keyboard:main',
            'sensor_manager = rover_control.sensor_manager:main',
            'telemetry = rover_control.telemetry:main',
            'safety_monitor = rover_control.safety_monitor:main',
        ],
    },
)
