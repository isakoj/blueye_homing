import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import numpy as np
class SensorManager(Node):
    def __init__(self):
        super().__init__('sensor_manager')
        self.station_position = np.array([0.0,0.0,0.0])  # Define the station position
        self.threshold_distance = 5.0  # Define the distance threshold
        self.usbl_active = True

        # Subscribers
        self.pose_sub = self.create_subscription(Odometry, '/odometry/filtered', self.pose_callback, 10)
        self.aruco_sub = self.create_subscription(PoseStamped, '/aruco_pose', self.aruco_callback, 10)
        self.usbl_sub = self.create_subscription(PoseStamped, '/usbl_pose', self.usbl_callback, 10)
        
        # Publisher for the filter
        self.filtered_pose_pub = self.create_publisher(PoseStamped, '/filtered_pose', 10)

    def pose_callback(self, msg):
        current_position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        distance = self.calculate_distance(current_position, self.station_position)

        if distance < self.threshold_distance:
            self.usbl_active = False
        else:
            self.usbl_active = True

    def calculate_distance(self, pos1, pos2):
        return ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5

    def aruco_callback(self, msg):
        if not self.usbl_active:
            self.publish_filtered_pose(msg.pose)

    def usbl_callback(self, msg):
        if self.usbl_active:
            self.publish_filtered_pose(msg.pose)

    def publish_filtered_pose(self, pose):
        filtered_pose = PoseStamped()
        filtered_pose.pose = pose
        self.filtered_pose_pub.publish(filtered_pose)

def main(args=None):
    rclpy.init(args=args)
    sensor_manager = SensorManager()
    rclpy.spin(sensor_manager)
    sensor_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
