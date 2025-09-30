from setuptools import find_packages, setup

package_name = 'overrack_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'mission_runner = overrack_mission.mission_node:main',
        ],
    },
)
