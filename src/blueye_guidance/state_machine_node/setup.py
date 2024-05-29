from setuptools import find_packages, setup

package_name = 'state_machine_node'

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
            'monitor_node = state_machine_node.monitor_node:main',
            'transit2 = state_machine_node.transit2:main',
            'blueye_fsm = state_machine_node.blueye_fsm:main',
            'homing = state_machine_node.homing:main',
            'homing_mk_ii = state_machine_node.homing_mk_ii:main',
        ],
    },
)
