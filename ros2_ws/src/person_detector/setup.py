from setuptools import setup
import os
from glob import glob

package_name = 'person_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_data={
        package_name: ['*.so'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhuo-skadi',
    maintainer_email='your.email@example.com',
    description='ROS2 node for person detection using YOLOv8',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = person_detector.detector_node:main',
            'person_detector_node = person_detector.person_detector_node:main',
            'performance_detector_node = person_detector.performance_detector_node:main',
            'test_listener = person_detector.test_listener:main'
        ],
    },
)