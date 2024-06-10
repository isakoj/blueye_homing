import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from blueye_interfaces.action import AdjustingPosition
from geometry_msgs.msg import PoseStamped, Vector3Stamped, TwistStamped
from std_msgs.msg import Empty, Float32, Int32
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from transforms3d.euler import quat2euler
import math
import numpy as np


class AdjustingPositionActionServer(Node):

    def __init__(self):
        super().__init__('adjusting_position_action_server')
        self._action_server = ActionServer(
            self,
            AdjustingPosition,
            '/adjusting_position',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self._ref_pub = self.create_publisher(PoseStamped, '/FSM/homing_reference', 10)
        self._twist_pub = self.create_publisher(TwistStamped, '/FSM/twist', 10)
        self._stop_pub = self.create_publisher(Empty, '/blueye/stop', 10)
        self._odom_sub = self.create_subscription(Odometry, 'odometry/filtered', self.odom_callback, qos_profile_sensor_data)
        self._radius_pub = self.create_publisher(Float32, '/FSM/radius', 10)
        self._marker_sub = self.create_subscription(Int32, '/blueye/marker_id', self.marker_callback, 10)
        self.current_position = None
        self.current_velocity = None
        self.u0 = 0.3
        self.k_slope = 1.0
        self.u_d = 0.15
        self.delta = 2.0
        self.nu_d = np.zeros(3)
        self.p = np.zeros(3)
        self.waypoints = None
        self.waypoint_index = 0
        self.marker = None

    def goal_callback(self, goal_request):
        self.get_logger().info('Received adjusting position goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request for adjusting position')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):

        self.get_logger().info('Executing adjusting position...')
        feedback_msg = AdjustingPosition.Feedback()

        target_pose = goal_handle.request.pose
        print(target_pose)
        target_point = 1

        while True:
            if self.current_position is None:
                continue

            current_x = self.current_position.position.x
            current_y = self.current_position.position.y
            current_z = self.current_position.position.z

            roll, pitch, yaw = quat2euler([self.current_position.orientation.w, self.current_position.orientation.x, self.current_position.orientation.y, self.current_position.orientation.z])
            psi = normalize_angle(yaw)

            self.p = np.array([current_x, current_y, psi])

            distance_to_target = np.linalg.norm(self.p[:2] - np.array([target_pose.pose.position.x, target_pose.pose.position.y]))

            self._ref_pub.publish(target_pose)
            feedback_msg.distance_remaining = distance_to_target
            goal_handle.publish_feedback(feedback_msg)

            if self.marker == target_point:
                self.marker_count += 1
            else:
                self.marker_count = 0

            if self.marker_count >= 100 or distance_to_target < 0.1:
                result = AdjustingPosition.Result()
                result.result_code = AdjustingPosition.Result.NONE
                goal_handle.succeed()
                self.get_logger().info('Adjusting position goal succeeded')
                self._stop_pub.publish(Empty())
                return result

            rclpy.spin_once(self, timeout_sec=1)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Adjusting position goal canceled')
                result = AdjustingPosition.Result()
                result.result_code = AdjustingPosition.Result.CANCELED
                self._stop_pub.publish(Empty())
                return result

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose
        self.current_velocity = msg.twist.twist

    def marker_callback(self, msg):
        self.marker = msg.data

def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def main(args=None):
    rclpy.init(args=args)
    node = AdjustingPositionActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
