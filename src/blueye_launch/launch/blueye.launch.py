import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import LogInfo
from launch.actions import EmitEvent, ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.actions import IncludeLaunchDescription
from launch.events import matches_action
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.launch_description_sources import PythonLaunchDescriptionSource

import lifecycle_msgs.msg

def generate_launch_description():
    
    robot_name = "blueye"
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    model_path = "src/gz_worlds/worlds/blueye_world.sdf"
    use_sim_time = {'use_sim_time': False}
    #tam_config = os.path.join(get_package_share_directory("blueye_thrust_allocation"), "config", "config.yaml"))
    
    # observer = Node(
    #     package = "blueye_observer",
    #     executable = "blueye_observer",
    #     name = 'ekf'
    # )

    controller = Node(
        package = "blueye_controller",
        executable = "blueye_controller",
        name = "PID",
        parameters=[use_sim_time]
    )
    
 
    # Define the charging station's USBL transceiver node
    usbl_transceiver = Node(
        package="usbl_transceiver",  # Replace with your actual package name
        executable="usbl_transceiver",
        name="usbl_charging_station_transceiver",
        parameters=[use_sim_time
        ]
    )

    camera_blueye = Node(
            package="blueye_image",
            executable="blueye_image",
            name="blueye_image",
            parameters=[use_sim_time
        ]
            )


    actions_server = Node(
        package="action_server",
        executable="navigate_to_pose_action_server",
        name="blueye_actions",
        parameters=[use_sim_time]
    )



    return LaunchDescription([
        gz_sim,
        usbl_transceiver,
        usbl_transponder,
        # homing,
        camera_blueye,
        utility,
        joystick_node,
        joystick_parser,
        thrust_allocation,
        controller,
        ros_bridge,
        tf_transforms,
        actions_server
    ])


