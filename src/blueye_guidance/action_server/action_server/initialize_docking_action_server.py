import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from blueye_interfaces.action import InitializeDocking
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Vector3, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from rclpy.qos import qos_profile_sensor_data
import math
import numpy as np
from transforms3d.euler import euler2quat, quat2euler

class InitializeDockingActionServer(Node):

    def __init__(self):
        super().__init__('initialize_docking_action_server')
        self._action_server = ActionServer(
            self,
            InitializeDocking,
            '/initialize_docking',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self._ref_pub = self.create_publisher(PoseStamped, '/FSM/homing_reference', 10)
        self._ref_pub_2 = self.create_publisher(PoseStamped, '/FSM/reference', 10)
        self._init_ref_pub = self.create_publisher(Vector3, '/FSM/initialize_reference', 10)
        self._error_pub = self.create_publisher(Vector3Stamped, '/FSM/errors', 10)
        self._stop_pub = self.create_publisher(Empty, '/blueye/stop', 10)
        self._aruco_sub = self.create_subscription(PoseWithCovarianceStamped, 'blueye/aruco_pose_board', self.aruco_callback, qos_profile_sensor_data)
        self._odom_sub = self.create_subscription(Odometry, 'odometry/filtered', self.odom_callback, qos_profile_sensor_data)
        self.current_pose = None
        self.aruco_pose = Pose()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received initialize docking goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request for initialize docking')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing initialize docking...')
        feedback_msg = InitializeDocking.Feedback()
        
        desired_pose = goal_handle.request.aruco_pose


        while True:
            if self.current_pose is None:
                continue
            roll, pitch, yaw = quat2euler([self.current_pose.orientation.w, self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z])
            psi = normalize_angle(yaw)

            error_x = desired_pose.position.x - self.current_pose.position.x
            error_y = desired_pose.position.y - self.current_pose.position.y
            aruco_position = self.aruco_pose.position

            desired_heading = np.pi/3
            e_psi = normalize_angle(desired_heading - psi)

            error = Vector3Stamped()
            error.header.frame_id = 'map'
            error.header.stamp = self.get_clock().now().to_msg()
            error.vector.x = error_x
            error.vector.y = error_y
            error.vector.z = e_psi
            self._error_pub.publish(error)
            print('Error: ', error)

            eta_ref = PoseStamped()
            eta_ref.header.frame_id = 'map'
            eta_ref.header.stamp = self.get_clock().now().to_msg()
            eta_ref.pose.position.x = desired_pose.position.x
            eta_ref.pose.position.y = desired_pose.position.y
            eta_ref.pose.position.z = 0.0
            self._ref_pub_2.publish(eta_ref)

            feedback_msg.current_pose.pose = self.current_pose
            feedback_msg.distance_remaining = 0.0  # Calculate actual distance
            goal_handle.publish_feedback(feedback_msg)

            if abs(error_x) < 0.1 and abs(e_psi) < 0.87:

                self._stop_pub.publish(Empty())
                goal_handle.succeed()
                result = InitializeDocking.Result()
                result.result_code = InitializeDocking.Result.NONE
                self._stop_pub.publish(Empty())
                return result
            rclpy.spin_once(self, timeout_sec=1)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Initialize docking goal canceled')
                result = InitializeDocking.Result()
                result.result_code = InitializeDocking.Result.ERROR
                self._stop_pub.publish(Empty())
                return result

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def aruco_callback(self, msg):
        self.aruco_pose = msg.pose.pose

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi   

def main(args=None):
    rclpy.init(args=args)
    node = InitializeDockingActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
