from setuptools import setup
import os
from glob import glob

package_name = 'slam_node'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Akshat Gupta',
    maintainer_email='akshat@dakshdynamics.com',
    description='All-in-one Mapping (.pgm), Localization, and Checkpoint Navigation node fed by /odometry/filtered',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mapper_utility = slam_node.mapper_utility:main',
            'save_map = slam_node.map_saver_cli:main',
            'nav_utility = slam_node.nav_utility:main',
            'checkpoint_cli = slam_node.checkpoint_manager:main',
        ],
    },
)
