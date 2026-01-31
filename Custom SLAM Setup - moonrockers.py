# setup.py
from setuptools import setup
import os
from glob import glob

package_name = 'custom_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Custom SLAM implementation from scratch for Intel RealSense D435i',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'custom_slam_node = custom_slam.custom_slam_node:main',
        ],
    },
)

# ============================================
# setup.cfg
# ============================================
# [develop]
# script_dir=$base/lib/custom_slam
# [install]
# install_scripts=$base/lib/custom_slam

# ============================================
# requirements.txt (optional, for pip dependencies)
# ============================================
# numpy>=1.21.0
# opencv-python>=4.5.0
# scipy>=1.7.0