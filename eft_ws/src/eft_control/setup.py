import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'eft_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AmeyaB2005',
    maintainer_email='ameyabhagwat2005@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_bridge = eft_control.teleop_bridge:main',
            'trajectory_visualizer = eft_control.trajectory_visualizer:main',
            'waypoint_controller = eft_control.waypoint_controller:main',
            'waypoint_controller_mujoco = eft_control.waypoint_controller_mujoco:main',
            'keyboard_teleop = eft_control.keyboard_teleop:main',
            'joystick_teleop = eft_control.joystick_teleop:main'
        ],
    },
)
