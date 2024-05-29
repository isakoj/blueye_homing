import rclpy
from rclpy.node import Node

from transforms3d.euler import quat2euler

from blueye.sdk import Drone

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32, Empty
from rclpy.qos import QoSProfile
import numpy as np

from blueye_controller.PID_controller import PIDController
from blueye_controller.pid_tuning_gui import PIDTuningGUI  # Import the GUI class

class Controller(Node):
    def __init__(self):
        super().__init__('blueye_controller')
        self.declare_parameter('robot_name', 'blueye')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        
        self.drone = Drone()

        # Initialize PID controllers with default values
        self.transit_surge_pid = PIDController(kp=0.6, ki=0.0, kd=0.2)
        self.transit_sway_pid = PIDController(kp=0.16, ki=0.0, kd=0.002)
        self.transit_yaw_pid = PIDController(kp=0.75, ki=0.01, kd=2.0)

        # Homing PID gains
        self.homing_surge_pid = PIDController(kp=0.8, ki=0.0, kd=0.25)
        self.homing_sway_pid = PIDController(kp=0.2, ki=0.0, kd=0.01)
        self.homing_yaw_pid = PIDController(kp=1.0, ki=0.01, kd=2.5)
        
        self.controller_active = False
        self.yaw_control = False
        self.direction = None
        self.homing_control = False

        qos = QoSProfile(depth=10)

        self.eta = np.zeros(3)
        self.nu = np.zeros(3)
        self.bias = np.zeros(3)
        self.reference = np.zeros(3)

        self.dt = 0.01
        self.radius_d = 3.0
        self.angle_step = -0.2
        self.angle_d = np.pi/2

        # Subscribers and publishers
        self.obs_sub = self.create_subscription(Odometry, '/odometry/filtered', self.observer_callback, 10)
        self.reference_sub = self.create_subscription(PoseStamped, '/FSM/reference', self.reference_callback, 10)
        self.yaw_reference_sub = self.create_subscription(PoseStamped, '/FSM/yaw_reference', self.yaw_reference_callback, 10)
        self.homing_sub = self.create_subscription(PoseStamped, '/FSM/homing_reference', self.homing_reference_callback, 10)
        self.radius_sub = self.create_subscription(Float32, '/FSM/radius', self.radius_callback, 10)
        # Initialize the GUI

        self.velocity_stop = self.create_subscription(Empty, '/blueye/stop', self.stop_velocity_commands, 1)
        self.gui = PIDTuningGUI(self)
    
    def control_movement(self, radius, angle, radius_d, angle_d):
        
        surge_pid = self.homing_surge_pid
        sway_pid = self.homing_sway_pid
        

        error_radius = radius - radius_d
        surge_cmd = surge_pid.update(error_radius, self.nu[0], self.dt)

        error_angle = normalize_angle(angle - angle_d)
        sway_cmd = sway_pid.update(error_angle, self.nu[1], self.dt)
        return surge_cmd, sway_cmd
    
    def smooth_transition(self, error, threshold, scale):
        """Calculate a smooth transition value between 0 and 1 based on the error."""
        if abs(error) < threshold:
            return 1.0
        else:
            return 1.0 / (1.0 + scale * (abs(error) - threshold))
    
    def stop_velocity_commands(self, msg=None):
        self.reference = None
        self.controller_callback()

    def controller_callback(self):
        if self.reference is None:
            self.drone.motion.surge = 0.0
            self.drone.motion.sway = 0.0
            self.drone.motion.yaw = 0.0
            
            
            
        else:
            self.eta[2] = normalize_angle(self.eta[2])
            R = rotation(self.eta[2])

            error_n = self.reference - self.eta
            error_b = R.T @ error_n
            error_b[2] = normalize_angle(error_b[2])

            self.get_logger().info(f'Error: {error_b}, Error_n: {error_n}')

            if self.homing_control:
                surge_pid = self.homing_surge_pid
                sway_pid = self.homing_sway_pid
                yaw_pid = self.homing_yaw_pid

                dock_pose = np.array([0,0])
                radius, angle = calculate_polar_coordinates(self.eta[:2], dock_pose)

                surge_vel_cmd, sway_vel_cmd = self.control_movement(radius, angle, self.radius_d, self.angle_d)

                dx = -self.eta[0]
                dy = -self.eta[1]

                desired_yaw = np.arctan2(dy, dx)
                error_b[2] = normalize_angle(desired_yaw - self.eta[2])
                yaw_vel_cmd = yaw_pid.update(error_b[2], self.nu[2], self.dt)
                yaw_error_threshold = 0.2
                
            else:
                surge_pid = self.transit_surge_pid
                sway_pid = self.transit_sway_pid
                yaw_pid = self.transit_yaw_pid

                surge_vel_cmd = surge_pid.update(error_b[0], self.nu[0], self.dt)
                sway_vel_cmd = sway_pid.update(error_b[1], self.nu[1], self.dt)
                yaw_vel_cmd = yaw_pid.update(error_b[2], self.nu[2], self.dt)
                yaw_error_threshold = 0.1 

            max_force = 0.5  # Adjusted limit for force
            max_torque = 1.0 # Adjusted limit for torque
            # Apply limits to surge and sway force outputs
            surge_vel_cmd = max(-max_force, min(surge_vel_cmd, max_force))
            sway_vel_cmd = max(-max_force, min(sway_vel_cmd, max_force))
            yaw_vel_cmd = max(-max_torque, min(yaw_vel_cmd, max_torque))

            transition_scale = self.smooth_transition(error_b[2], yaw_error_threshold, 10.0)
            surge_vel_cmd *= transition_scale
            sway_vel_cmd *= transition_scale

            self.drone.motion.surge = surge_vel_cmd
            self.drone.motion.sway = sway_vel_cmd
            self.drone.motion.yaw = yaw_vel_cmd
            
    def observer_callback(self, msg):
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.w, orientation.x, orientation.y, orientation.z)
        _, _, yaw = quat2euler(quaternion, axes='sxyz')
        self.eta = np.array([position[0], position[1], normalize_angle(yaw)])

        self.nu = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z])

    def reference_callback(self, msg):
        try:
            position = msg.pose.position
            orientation = msg.pose.orientation
            position_array = np.array([position.x, position.y, position.z])
            quaternion = (orientation.w, orientation.x, orientation.y, orientation.z)
            _, _, reference_yaw = quat2euler(quaternion, axes='sxyz')
            self.reference = np.array([position_array[0], position_array[1], normalize_angle(reference_yaw)])
        except Exception as e:
            self.get_logger().error(f'Failed in reference_callback: {str(e)}')

        self.yaw_control = False
        self.homing_control = False
        self.controller_callback()

    def yaw_reference_callback(self, msg):
        try:
            orientation = msg.pose.orientation
            quaternion = (orientation.w, orientation.x, orientation.y, orientation.z)
            _, _, reference_yaw = quat2euler(quaternion, axes='sxyz')
            self.reference[2] = normalize_angle(reference_yaw)
            self.yaw_control = True
            self.homing_control = False
        except Exception as e:
            self.get_logger().error(f'Failed in yaw_reference_callback: {str(e)}')

        self.controller_callback()

    def homing_reference_callback(self, msg):
        self.reference = np.array([msg.pose.position.x, msg.pose.position.y, 0])
        self.yaw_control = False
        self.homing_control = True
        self.controller_callback()

    def direction_callback(self, msg):
        self.direction = msg.data

    def radius_callback(self, msg):
        self.radius_d = msg.data


def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi
            
def rotation(angle):
    return np.array([[np.cos(angle), -np.sin(angle), 0],
                     [np.sin(angle), np.cos(angle), 0],
                     [0, 0, 1]])

def calculate_polar_coordinates(eta, dock_pose):
    dx = eta[0] - dock_pose[0]
    dy = eta[1] - dock_pose[1]
    radius = np.sqrt(dx**2 + dy**2)
    angle = np.arctan2(dy, dx)
    return radius, angle

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    
    # Run the ROS2 node in a separate thread
    import threading
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,))
    ros_thread.start()
    
    # Run the Tkinter GUI in the main thread
    node.gui.run()

    # Shutdown ROS2
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
