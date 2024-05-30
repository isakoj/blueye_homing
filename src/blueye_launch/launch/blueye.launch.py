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
        executable = "blueye_controller",
        name = "PID",

    )
    
 
    # Define the charging station's USBL transceiver node
    camera_blueye = Node(
            package="image",
            executable="blueye_image",
            name="image",

            )


    actions_server = Node(
        package="action_server",
        executable="navigate_to_pose_action_server",
        name="blueye_actions",

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



    return LaunchDescription([
        # camera_blueye,
        controller,
        actions_server,
        Blueye_camera,
        Blueye_IMU    
    ])


