import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'overrack_mission'
launch_files = glob('overrack_mission/launch/**/*.py', recursive=True)
param_files = glob('overrack_mission/param/*.yaml')

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), launch_files),
        (os.path.join('share', package_name, 'param'), param_files),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Overrack Robotics',
    maintainer_email='support@overrack.ai',
    description='Mission runner built on top of px4_msgs offboard interfaces.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_runner = overrack_mission.nodes.mission_control_node:main',
            'inspection_node = overrack_mission.nodes.inspection_node:main',
            'mission_metrics = overrack_mission.nodes.metrics_node:main',
            'torch_controller = overrack_mission.px4io.actuators.torch_controller:main',
        ],
    },
)
