import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.launch_description_sources import PythonLaunchDescriptionSource

import lifecycle_msgs.msg

def generate_launch_description():



    controller = Node(
        package = "blueye_controller",
        executable = "controller_node",
        name = "PID",

    )

    camera_blueye = Node(
            package="blueye_image",
            executable="blueye_image",
            name="blueye_image",

            )
    
    Blueye_camera = Node(
        package='blueye_converters',
        executable='Video_to_ros2',
        name='video_node'
    )

    Blueye_IMU = Node(
        package='blueye_converters',
        executable='IMU_to_ros2',
        name='imu_node'
    )
    
    blueye_dvl = Node(
        package='blueye_converters',
        executable='dvl_to_ros2',
        name='dvl_node'
    )
    
    action_server_executables = [
        "navigate_to_pose_action_server",
        "adjusting_position_action_server",
        "homing_action_server",
        "initialize_docking_action_server",
        "navigate_waypoints_action_server",
        "maneuvering_action_server"
    ]

    # Create action server nodes
    action_servers = [
        Node(
            package="action_server",
            executable=exe,
            name="blueye_actions",
        )
        for exe in action_server_executables
    ]



    return LaunchDescription([
        camera_blueye,
        controller,
        Blueye_camera,
        Blueye_IMU,
        blueye_dvl,
        #blueye_usbl,
    ] + action_servers)


