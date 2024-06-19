import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3Stamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import time
import blueye.protocol as bp
from blueye.sdk import Drone

class DepthPublisher(Node):
    def __init__(self):
        super().__init__('dvl_publisher')
        self.dvl_publisher = self.create_publisher(PoseWithCovarianceStamped, '/blueye/depth', 100)
        self.br = TransformBroadcaster(self)  # TF2 Transform Broadcaster
        self.initialize_drone()

    def initialize_drone(self):
        self.drone = Drone()
        self.drone.telemetry.set_msg_publish_frequency(bp.DepthTel, 40)
        self.drone.telemetry.add_msg_callback([bp.DepthTel], self.callback_position_estimate)
        
    def callback_position_estimate(self, msg_type, msg):
        
        depth_msg = PoseWithCovarianceStamped()
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.header.frame_id = 'odom'
        depth_msg.pose.pose.position.z = -msg.depth.value
        depth_msg.pose.covariance = [0.0] * 36
        
        self.dvl_publisher.publish(depth_msg)  
        
def main(args=None):
    rclpy.init(args=args)
    depth_publisher = DepthPublisher()

    try:
        print("Press CTRL+C to stop the node.")
        rclpy.spin(depth_publisher)
    except KeyboardInterrupt:
        print("Stopping IMU publisher node.")
    finally:
        depth_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()