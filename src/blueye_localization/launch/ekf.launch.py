from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('blueye_localization'), 'config')

    original_ekf_params = os.path.join(config_dir, 'ekf_config.yaml')
    ekf_dvl_params = os.path.join(config_dir, 'ekf_config_dvl.yaml')
    ekf_imu_params = os.path.join(config_dir, 'ekf_config_imu.yaml')
    ekf_usbl_params = os.path.join(config_dir, 'ekf_config_usbl.yaml')
    ekf_marker_params = os.path.join(config_dir, 'ekf_config_marker.yaml')

    return LaunchDescription([
        # Original EKF Node
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[original_ekf_params, {"use_sim_time": False}]
        ),

        ## EKF Node for DVL
        #launch_ros.actions.Node(
        #    package='robot_localization',
        #    executable='ekf_node',
        #    name='ekf_filter_node',
        #    output='screen',
        #    parameters=[ekf_dvl_params, {"use_sim_time": True}],
        #    remappings=[
        #        ('odometry/filtered', '/odometry/filtered/dvl'),
        #        ('accel/filtered', '/accel/filtered/dvl')
        #    ]
        #),
        ## EKF Node for USBL
        #launch_ros.actions.Node(
        #    package='robot_localization',
        #    executable='ekf_node',
        #    name='ekf_filter_node',
        #    output='screen',
        #    parameters=[ekf_usbl_params, {"use_sim_time": True}],
        #    remappings=[
        #        ('odometry/filtered', '/odometry/filtered/usbl'),
        #        ('accel/filtered', '/accel/filtered/usbl')
        #    ]
        #)
        # EKF Node for Marker
        #launch_ros.actions.Node(
        #    package='robot_localization',
        #    executable='ekf_node',
        #    name='ekf_filter_node',
        #    output='screen',
        #    parameters=[ekf_marker_params, {"use_sim_time": False}],
        #    remappings=[
        #        ('odometry/filtered', '/odometry/filtered/marker'),
        #        ('accel/filtered', '/accel/filtered/marker')
        #    ]
        #)
    ])

if __name__ == '__main__':
    generate_launch_description()
