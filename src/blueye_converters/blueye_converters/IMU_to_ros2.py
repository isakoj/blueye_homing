import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3Stamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import time
import blueye.protocol as bp
from blueye.sdk import Drone

from transforms3d.euler import euler2quat
import numpy as np

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.imu_publisher = self.create_publisher(Imu, '/blueye/imu', 10)
        self.magnetometer_publisher = self.create_publisher(MagneticField, 'magnetometer/data', 10)  # Magnetometer publisher
        self.attitude_publisher = self.create_publisher(Vector3Stamped, '/blueye/attitude', 10)  # Attitude publisher
        self.yaw_quat_publisher = self.create_publisher(Imu, '/blueye/quaternion/imu', 10)
        self.imu_to_enu = self.create_publisher(Imu, '/blueye/imu_enu', 10)
        self.br = TransformBroadcaster(self)  # TF2 Transform Broadcaster
        self.initialize_drone()

    def initialize_drone(self):
        self.drone = Drone()
        self.drone.telemetry.set_msg_publish_frequency(bp.CalibratedImuTel, 100)
        self.drone.telemetry.add_msg_callback([bp.CalibratedImuTel], self.callback_imu_calibrated)
        self.drone.telemetry.set_msg_publish_frequency(bp.AttitudeTel, 100)
        self.drone.telemetry.add_msg_callback([bp.AttitudeTel], self.callback_attitude)

    def callback_imu_calibrated(self, msg_type, msg):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'
        imu_msg.angular_velocity.x = msg.imu.gyroscope.x
        imu_msg.angular_velocity.y = msg.imu.gyroscope.y
        imu_msg.angular_velocity.z = -msg.imu.gyroscope.z
        imu_msg.linear_acceleration.x = msg.imu.accelerometer.x
        imu_msg.linear_acceleration.y = msg.imu.accelerometer.y
        imu_msg.linear_acceleration.z = msg.imu.accelerometer.z
        self.imu_publisher.publish(imu_msg)

        # Assuming msg.imu also contains magnetometer data (x, y, z)
        magnetometer_msg = MagneticField()
        magnetometer_msg.header.stamp = imu_msg.header.stamp
        magnetometer_msg.header.frame_id = 'imu_link'
        # Populate magnetometer_msg.magnetic_field with magnetometer data
        magnetometer_msg.magnetic_field.x = msg.imu.magnetometer.x  # Placeholder, adjust according to actual data structure
        magnetometer_msg.magnetic_field.y = msg.imu.magnetometer.y
        magnetometer_msg.magnetic_field.z = msg.imu.magnetometer.z
        self.magnetometer_publisher.publish(magnetometer_msg)
        
        

        # Now broadcast the transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        t.transform.translation.x = 1.0  # Adjust these values based on the actual physical location of the IMU
        t.transform.translation.y = 1.0
        t.transform.translation.z = 1.0
        # Assuming no rotation from base_link to imu_link; adjust as necessary
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

        # Your existing TF broadcasting code here...
        
    def callback_attitude(self, msg_type, msg):
        attitude_msg = Vector3Stamped()
        attitude_msg.header.stamp = self.get_clock().now().to_msg()
        attitude_msg.header.frame_id = 'base_link'
        
        # Assuming msg contains roll, pitch, and yaw data
        attitude_msg.vector.x = np.radians(msg.attitude.roll)
        attitude_msg.vector.y = np.radians(msg.attitude.pitch)
        attitude_msg.vector.z = np.radians(msg.attitude.yaw)
        
        
        
        # Publish the attitude message
        self.attitude_publisher.publish(attitude_msg)
        
        w, x, y, z = euler2quat(np.radians(msg.attitude.roll),
                                np.radians(msg.attitude.pitch),
                                np.radians(msg.attitude.yaw))
        
        
        heading_msg = Imu()
        heading_msg.header.stamp = self.get_clock().now().to_msg()
        heading_msg.header.frame_id = 'base_link'
        heading_msg.orientation.w = w
        heading_msg.orientation.x = x
        heading_msg.orientation.y = y
        heading_msg.orientation.z = z
        self.yaw_quat_publisher.publish(heading_msg)
        
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        t.transform.translation.x = 1.0
        t.transform.translation.y = 1.0
        t.transform.translation.z = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)
        
        heading_msg_enu = Imu()
        heading_msg_enu.header.stamp = self.get_clock().now().to_msg()
        heading_msg_enu.header.frame_id = 'base_link'
        heading_msg_enu.orientation.w = w
        heading_msg_enu.orientation.x = y
        heading_msg_enu.orientation.y = x
        heading_msg_enu.orientation.z = -z
        self.imu_to_enu.publish(heading_msg_enu)
        


def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()

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