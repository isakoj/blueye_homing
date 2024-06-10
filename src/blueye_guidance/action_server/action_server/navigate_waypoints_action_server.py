import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from blueye_interfaces.action import NavigateWaypoints
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, TwistStamped, Vector3Stamped
from std_msgs.msg import Empty
from rclpy.qos import qos_profile_sensor_data
import math
import numpy as np
from transforms3d.euler import euler2quat, quat2euler

class NavigateWaypointsActionServer(Node):

    def __init__(self):
        super().__init__('navigate_waypoints_action_server')
        self._action_server = ActionServer(
            self,
            NavigateWaypoints,
            '/navigate_waypoints',
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
        self.U = 1.0
        self.delta = 1.2

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = NavigateWaypoints.Feedback()
        
        waypoints = goal_handle.request.poses
        result = NavigateWaypoints.Result()
    
        for target_point in waypoints:
            self.ref_point = self.current_position if self.current_position else target_point.pose
    
            while True:
                if self.current_position is None:
                    continue
                
                current_x = self.current_position.position.x
                current_y = self.current_position.position.y
                current_z = self.current_position.position.z
    
                roll, pitch, yaw = quat2euler([self.current_position.orientation.w, self.current_position.orientation.x, self.current_position.orientation.y, self.current_position.orientation.z])
                psi = normalize_angle(yaw)
    
                ref_x = self.ref_point.position.x
                ref_y = self.ref_point.position.y
                ref_z = self.ref_point.position.z
    
                target_point_x = target_point.pose.position.x
                target_point_y = target_point.pose.position.y
                target_point_z = target_point.pose.position.z
    
                distance = math.sqrt((target_point_x - current_x)**2 + (target_point_y - current_y)**2)
                
                dx = target_point_x - ref_x
                dy = target_point_y - ref_y
    
                pi_p = math.atan2(dy, dx)
                pi_p = normalize_angle(pi_p)
    
                R = np.array([[math.cos(pi_p), -math.sin(pi_p)], 
                              [math.sin(pi_p), math.cos(pi_p)]])
                
                eps = R.T @ np.array([current_x - ref_x, current_y - ref_y])
    
                e_x = eps[0]  # Along-track error
                e_y = eps[1]  # Cross-track error
    
                course_d = pi_p - math.atan(e_y / self.delta)
                course_d = normalize_angle(course_d)
    
                nu_d = np.array([self.U * math.cos(course_d), self.U * math.sin(course_d)])
    
                heading_d = course_d - math.asin(self.current_velocity.linear.y / math.sqrt(self.current_velocity.linear.x**2 + self.current_velocity.linear.y**2))
                heading_d = normalize_angle(heading_d)
                print('Heading', heading_d)
    
                e_psi = normalize_angle(heading_d - psi)
    
                pose = PoseStamped()
                pose.header.frame_id = "odom"
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = target_point_x
                pose.pose.position.y = target_point_y   
                pose.pose.position.z = target_point_z
                self._ref_pub.publish(pose)
    
                error_msg = Vector3Stamped()
                error_msg.header.frame_id = "odom"
                error_msg.header.stamp = self.get_clock().now().to_msg()
                error_msg.vector.x = e_x
                error_msg.vector.y = e_y
                error_msg.vector.z = e_psi
                self._error_pub.publish(error_msg)
    
                twist_d = TwistStamped()
                twist_d.header.frame_id = 'base_link'
                twist_d.header.stamp = self.get_clock().now().to_msg()
                twist_d.twist.linear.x = nu_d[0]
                twist_d.twist.linear.y = nu_d[1]
                self._twist_pub.publish(twist_d)
    
                # Convert current_position (Pose) to PoseStamped
                current_pose_stamped = PoseStamped()
                current_pose_stamped.header.frame_id = "odom"
                current_pose_stamped.header.stamp = self.get_clock().now().to_msg()
                current_pose_stamped.pose = self.current_position
    
                feedback_msg.current_pose = current_pose_stamped
                feedback_msg.number_of_recoveries = 0  # Placeholder, should be updated based on actual recoveries
                feedback_msg.distance_remaining = distance
                goal_handle.publish_feedback(feedback_msg)
    
                if distance < 0.5:
                    self.get_logger().info('Reached waypoint, finding next waypoint...')
                    break
                
                rclpy.spin_once(self, timeout_sec=1)
    
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    result.error_code = NavigateWaypoints.Result.CANCELED  # Indicating cancellation
                    
                    # Publish stop command
                    self._stop_pub.publish(Empty())
    
                    return result
    
            # Update the reference point to the current target point after reaching it
            self.ref_point = target_point.pose
    
        goal_handle.succeed()
        result.error_code = NavigateWaypoints.Result.NONE  # Indicating success
        # Publish stop command
        self._stop_pub.publish(Empty())
    
        return result


    def odom_callback(self, msg):
        self.current_position = msg.pose.pose
        self.current_velocity = msg.twist.twist

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def main(args=None):
    rclpy.init(args=args)
    node = NavigateWaypointsActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
