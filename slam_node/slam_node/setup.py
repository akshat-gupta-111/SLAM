from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'slam_node'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS 2 package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Checkpoints (seed file)
        (os.path.join('share', package_name, 'checkpoints'),
            glob('checkpoints/*.json')),
        # Maps directory placeholder
        (os.path.join('share', package_name, 'maps'), []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kushal',
    maintainer_email='kushalsoni9876@gmail.com',
    description='Industry-standard SLAM mapping and checkpoint management',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Interactive checkpoint recorder
            'checkpoint_saver = slam_node.checkpoint_saver:main',
        ],
    },
)
