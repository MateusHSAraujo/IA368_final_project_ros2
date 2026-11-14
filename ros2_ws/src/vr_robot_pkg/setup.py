import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'vr_robot_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mateus Henrique Silva Araujo',
    maintainer_email='mateusaraujopro2001@gmail.com',
    description=('ROS 2 side of the IA368 mixed-reality HMI: CoppeliaSim bridge '
                 'for the Pioneer P3-DX, SLAM, Nav2 with keepout filter'),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'coppelia_bridge_node = vr_robot_pkg.coppelia_bridge_node:main',
            'keepout_mask_node = vr_robot_pkg.keepout_mask_node:main',
        ],
    },
)
