from setuptools import setup
import os
from glob import glob

package_name = 'mapless_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='your-email@example.com',
    description='Mapless Navigation with Multi-LiDAR Fusion and YOLO Tracking',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_tracker_node = mapless_nav.target_tracker_node:main',
            'following_controller_node = mapless_nav.following_controller_node:main',
            'yolo_detector_node = mapless_nav.yolo_detector_node:main',
            'bytetrack_tracker_node = mapless_nav.bytetrack_tracker_node:main',
        ],
    },
)
