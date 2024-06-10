from setuptools import find_packages, setup

package_name = 'blueye_converters'

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
    maintainer='isakoj',
    maintainer_email='isakoj@stud.ntnu.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'IMU_to_ros2 = blueye_converters.IMU_to_ros2:main',
            'Video_to_ros2 = blueye_converters.Video_to_ros2:main',
            'laptop_camera = blueye_converters.laptop_camera:main',
            'blueye_image_simple = blueye_converters.blueye_image_simple:main',
            'BluEye_Pose = blueye_converters.BluEye_Pose:main',
            'BluEye_Force = blueye_converters.BluEye_Force:main',
            'dvl_to_ros2 = blueye_converters.dvl_to_ros2:main',
            
        ],
    },
)
