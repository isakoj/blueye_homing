import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from blueye_interfaces.action import Maneuvering
from geometry_msgs.msg import PoseStamped, Vector3Stamped, TwistStamped
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from transforms3d.euler import quat2euler
import math
import numpy as np

class ManeuveringActionServer(Node):

    def __init__(self):
        super().__init__('maneuvering_action_action_server')
        self._action_server = ActionServer(
            self,
            Maneuvering,
            '/maneuvering',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self._ref_pub = self.create_publisher(PoseStamped, '/FSM/reference', 10)
        self._error_pub = self.create_publisher(Vector3Stamped, '/FSM/errors', 10)
        self._twist_pub = self.create_publisher(TwistStamped, '/FSM/twist', 10)
        self._stop_pub = self.create_publisher(Empty, '/blueye/stop', 10)
        self._odom_sub = self.create_subscription(Odometry, 'odometry/filtered', self.odom_callback, qos_profile_sensor_data)
        self.current_position = None
        self.current_velocity = None
        self.ref_point = None
        self.u0 = 0.3
        self.k_slope = 1.0
        self.u_d = 0.15
        self.delta = 2.0
        self.nu_d = np.zeros(3)

        self.timer_period = 0.01  # 100 Hz update rate
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.waypoints = []
        self.target_point = None
        self.goal_handle = None

    def goal_callback(self, goal_request):
        self.get_logger().info('Received adjusting position goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request for adjusting position')
        return CancelResponse.ACCEPT

    def desired_velocity(self, wp_src, wp_dst, theta, eta_err):
        if np.array_equal(wp_src, wp_dst):
            return np.array([0.0, 0.0])
        psd = wp_dst - wp_src
        psd_n = np.linalg.norm(psd)
        d = theta * psd_n
        speed_along_path = (self.sat((psd_n - d) / self.k_slope) + self.u0) / (1 + self.u0)
        speed_assignment = (speed_along_path * self.u_d) / psd_n
        p1 = psd * speed_assignment
        p2 = eta_err / np.sqrt(eta_err @ eta_err + self.delta ** 2)
        return p1 - p2

    def get_desired_position(self, wp_src, wp_dst, theta, offset=0):
        return (1 - theta) * wp_src + theta * wp_dst

    def theta_from_pos(self, wp_src, wp_dst, eta):
        n = wp_dst - wp_src
        n_norm = np.linalg.norm(n)
        if n_norm == 0.0:
            return 1.0
        n /= n_norm
        ap = eta - wp_src
        return np.dot(ap, n) / n_norm

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing maneuvering goal...')
        feedback_msg = Maneuvering.Feedback()

        self.waypoints = goal_handle.request.poses
        self.goal_handle = goal_handle
        self.target_point = self.waypoints.pop(0)
        self.goal_completed = False

        self.ref_point = None

        while self.target_point is not None:
            rclpy.spin_once(self)
        result = Maneuvering.Result()

        self._stop_pub.publish(Empty())


        return result

    def timer_callback(self):
        if self.goal_handle is None or self.current_position is None or self.target_point is None:
            return
        
        if self.ref_point is None:
            self.ref_point = self.current_position

        current_x = self.current_position.position.x
        current_y = self.current_position.position.y
        current_z = self.current_position.position.z
        p = np.array([current_x, current_y])

        roll, pitch, yaw = quat2euler([self.current_position.orientation.w, self.current_position.orientation.x, self.current_position.orientation.y, self.current_position.orientation.z])
        psi = normalize_angle(yaw)

        ref_x = self.ref_point.position.x
        ref_y = self.ref_point.position.y
        ref = np.array([ref_x, ref_y])

        target_point_x = self.target_point.pose.position.x
        target_point_y = self.target_point.pose.position.y
        next_waypoint = np.array([target_point_x, target_point_y])

        distance_to_target = np.linalg.norm(p - next_waypoint)


        theta = self.theta_from_pos(ref, next_waypoint, p)
        theta = np.clip(theta, 0.0, 1.0)

        p_d = self.get_desired_position(ref, next_waypoint, theta)
        eta_ref = np.array([*p_d, math.atan2(target_point_y - p[1], target_point_x - p[0])])

        nu_d = self.desired_velocity(ref, next_waypoint, theta, p - eta_ref[:2])

        heading_d = math.atan2(-current_y, -current_x)
        e_psi = normalize_angle(heading_d - psi)

        nu_desired = TwistStamped()
        nu_desired.header.frame_id = 'base_link'
        nu_desired.header.stamp = self.get_clock().now().to_msg()
        nu_desired.twist.linear.x = nu_d[0]
        nu_desired.twist.linear.y = nu_d[1]
        nu_desired.twist.angular.z = 0.0
        self._twist_pub.publish(nu_desired)

        eta_desired = PoseStamped()
        eta_desired.header.frame_id = 'odom'
        eta_desired.header.stamp = self.get_clock().now().to_msg()
        eta_desired.pose.position.x = eta_ref[0]
        eta_desired.pose.position.y = eta_ref[1]
        eta_desired.pose.position.z = self.target_point.pose.position.z
        self._ref_pub.publish(eta_desired)

        cte = np.cross(p - ref, next_waypoint - ref) / np.linalg.norm(next_waypoint - ref)

        error_msg = Vector3Stamped()
        error_msg.header.frame_id = 'odom'
        error_msg.header.stamp = self.get_clock().now().to_msg()
        error_msg.vector.x = p[0] - eta_ref[0]
        error_msg.vector.y = p[1] - eta_ref[1]
        error_msg.vector.z = e_psi
        self._error_pub.publish(error_msg)

        feedback_msg = Maneuvering.Feedback()
        feedback_msg.distance_remaining = distance_to_target
        self.goal_handle.publish_feedback(feedback_msg)

        if distance_to_target < 0.5:
            self.get_logger().info(f'Reached waypoint, finding next waypoint...')
            if self.waypoints:
                self.ref_point = self.target_point.pose
                self.get_logger().info(f'New reference point: {self.ref_point}')
                self.target_point = self.waypoints.pop(0)
            else:
                self.target_point = None
                self.goal_handle.succeed()
                self.goal_completed = True
                self.get_logger().info('Goal completed')

                self._stop_pub.publish(Empty())

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose
        self.current_velocity = msg.twist.twist

    def sat(self, value):
        return max(min(value, 1), -1)


def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def main(args=None):
    rclpy.init(args=args)
    node = ManeuveringActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
