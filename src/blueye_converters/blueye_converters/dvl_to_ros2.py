import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3Stamped, TwistWithCovarianceStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import time
import numpy as np
import blueye.protocol as bp
from blueye.sdk import Drone

class DVLPublisher(Node):
    def __init__(self):
        super().__init__('dvl_publisher')
        self.dvl_publisher = self.create_publisher(TwistWithCovarianceStamped, '/blueye/dvl_twist', 10)
        self.br = TransformBroadcaster(self)  # TF2 Transform Broadcaster
        self.initialize_drone()

    def initialize_drone(self):
        self.drone = Drone()
        self.drone.telemetry.set_msg_publish_frequency(bp.PositionEstimateTel, 40)
        self.drone.telemetry.add_msg_callback([bp.PositionEstimateTel], self.callback_position_estimate)
        
    def callback_position_estimate(self, msg_type, msg):
        
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'
        twist_msg.twist.twist.linear.x = msg.position_estimate.surge_rate
        twist_msg.twist.twist.linear.y = -msg.position_estimate.sway_rate
        twist_msg.twist.twist.angular.z = -msg.position_estimate.yaw_rate
        
        linear_std_dev = 0.101
        angular_std_dev = 0.01  # Adjust as needed

        twist_covariance = np.zeros((6, 6))
        twist_covariance[0, 0] = linear_std_dev ** 2
        twist_covariance[1, 1] = linear_std_dev ** 2
        twist_covariance[2, 2] = linear_std_dev ** 2
        twist_covariance[3, 3] = angular_std_dev ** 2
        twist_covariance[4, 4] = angular_std_dev ** 2
        twist_covariance[5, 5] = angular_std_dev ** 2
        twist_msg.twist.covariance = twist_covariance.flatten().tolist()
        
        self.dvl_publisher.publish(twist_msg)  
        
def main(args=None):
    rclpy.init(args=args)
    imu_publisher = DVLPublisher()

    try:
        print("Press CTRL+C to stop the node.")
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        print("Stopping IMU publisher node.")
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()