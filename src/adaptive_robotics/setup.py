from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'adaptive_robotics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chapter Author',
    maintainer_email='author@example.com',
    description='Chapter 5: Adaptive Robotics - Behavior switching, logging, and adaptation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_switcher = adaptive_robotics.behavior_switcher:main',
            'decision_logger = adaptive_robotics.decision_logger:main',
            'heuristic_selector = adaptive_robotics.heuristic_selector:main',
            'log_viewer = adaptive_robotics.log_viewer:main',
        ],
    },
)
