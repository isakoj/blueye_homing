import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from robot_interfaces.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Duration
import math
import time

class NavigateToPoseActionServer(Node):

    def __init__(self):
        super().__init__('navigate_to_pose_action_server')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self._desired_pose_pub = self.create_publisher(PoseStamped, '/desired_pose', 10)
        self._odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.current_position = None
        self.start_time = None

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = NavigateToPose.Feedback()
        self.start_time = time.time()
        
        goal_pose = goal_handle.request.pose

        while True:
            if self.current_position is None:
                continue

            dx = goal_pose.pose.position.x - self.current_position.x
            dy = goal_pose.pose.position.y - self.current_position.y
            distance = math.sqrt(dx**2 + dy**2)

            current_time = time.time()
            feedback_msg.current_pose = PoseStamped()
            feedback_msg.current_pose.pose = self.current_position
            feedback_msg.navigation_time = Duration(sec=int(current_time - self.start_time))
            feedback_msg.estimated_time_remaining = Duration(sec=int(distance / 0.1))  # Assuming constant speed
            feedback_msg.number_of_recoveries = 0
            feedback_msg.distance_remaining = distance

            if distance < 0.1:
                goal_handle.succeed()
                result = NavigateToPose.Result()
                result.error_code = NavigateToPose.Result.NONE
                self.get_logger().info('Goal succeeded')
                return result

            # Publish the desired pose
            self._desired_pose_pub.publish(goal_pose)

            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.distance_remaining} meters remaining')

            rclpy.spin_once(self, timeout_sec=1)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return NavigateToPose.Result(error_code=NavigateToPose.Result.NONE)

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position

def main(args=None):
    rclpy.init(args=args)
    node = NavigateToPoseActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
