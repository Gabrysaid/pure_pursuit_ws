from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'pure_pursuit_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Said Vera Carballo',
    maintainer_email='A00839097@tec.mx',
    description='Pure Pursuit path tracking for the Prius in Gazebo (M4 Activity 2)',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'path_recorder = pure_pursuit_controller.path_recorder:main',
            'pure_pursuit_node = pure_pursuit_controller.pure_pursuit_node:main',
        ],
    },
)
