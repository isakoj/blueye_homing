from setuptools import find_packages, setup

package_name = 'action_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='isak',
    maintainer_email='isak.o.jordal@ntnu.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigate_to_pose_action_server = action_server.navigate_to_pose_action_server:main',
            'adjusting_position_action_server = action_server.adjusting_position_action_server:main',
            'initialize_docking_action_server = action_server.initialize_docking_action_server:main',
            'homing_action_server = action_server.homing_action_server:main',
            'navigate_waypoints_action_server = action_server.navigate_waypoints_action_server:main',
            'maneuvering_action_server = action_server.maneuvering_action_server:main'
        ],
    },
)
