#!/usr/bin/env python3

# Required imports
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32, Empty, String, Float32, Bool, Header
from blueye_interfaces.msg import DesiredVelocity
from blueye_interfaces.action import AdjustingPosition, InitializeDocking, Homing, Maneuvering
import math
import numpy as np
from transforms3d.euler import euler2quat, quat2euler
import time
import rclpy
from rclpy.node import Node
from yasmin import CbState, State, Blackboard, StateMachine
from yasmin_ros import MonitorState, ActionState
from yasmin_viewer import YasminViewerPub
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from std_srvs.srv import SetBool

END = "end"
HAS_NEXT = "has_next"


def log_transition(node: Node, blackboard: Blackboard, state_name: str) -> None:
    if 'transitions' not in blackboard.__dict__:
        blackboard.transitions = []
    timestamp = node.get_clock().now().to_msg()
    blackboard.transitions.append({'state': state_name, 'timestamp': timestamp})
    transition_publisher = node.create_publisher(Header, '/FSM/transitions', 10)


    # Publish the transition
    transition_msg = Header()
    transition_msg.stamp = timestamp  # Use the timestamp from the clock
    transition_msg.frame_id = state_name
    node.get_logger().info(f"Transitioning to {state_name}")
    transition_publisher.publish(transition_msg)


def create_waypoint(blackboard: Blackboard) -> str:
    docking_station = PoseStamped()
    docking_station.header.frame_id = "odom"
    docking_station.pose.position.x = 0.0
    docking_station.pose.position.y = 0.0
    docking_station.pose.position.z = 0.0
    docking_station.pose.orientation.w = 1.0

    blackboard.waypoint = {
        "docking_station": docking_station
    }

    radius = 3.0
    num_waypoints = 8
    waypoints = []

    for i in range(num_waypoints):
        angle = 2 * math.pi * i / num_waypoints
        waypoint = PoseStamped()
        waypoint.header.frame_id = "odom"
        waypoint.pose.position.x = radius * math.cos(angle)
        waypoint.pose.position.y = radius * math.sin(angle)
        waypoint.pose.position.z = 0.0
        waypoint.pose.orientation.w = 1.0
        waypoints.append(waypoint)

    blackboard.circle_waypoints = waypoints

    # Simulate some processing time
    time.sleep(3)
    return SUCCEED

class GetPath(MonitorState):
    def __init__(self, node: Node) -> None:
        super().__init__(Odometry,  # msg type
                         "odometry/filtered",  # topic name
                         [SUCCEED, ABORT],  # outcomes
                         self.monitor_handler,
                         qos=qos_profile_sensor_data,  # monitor handler callback
                         msg_queue=10,  # queue of the monitor handler callback
                         timeout=10  # timeout to wait for msgs in seconds
                                     # if not None, CANCEL outcome is added
                         )
        self.node = node
        self._guidance_ref = node.create_publisher(PoseStamped, '/guidance/ref', 10)
        self.guidance_ref = None

    def path_length(self, path):
        distance = 0.0
        for i in range(len(path) - 1):
            dx = path[i + 1].pose.position.x - path[i].pose.position.x
            dy = path[i + 1].pose.position.y - path[i].pose.position.y
            distance += math.sqrt(dx**2 + dy**2)
        return distance    

    def monitor_handler(self, blackboard: Blackboard, msg: Odometry) -> str:
        waypoints = blackboard.circle_waypoints
        if not waypoints or len(waypoints) == 0:
            print("No waypoints found.")
            return ABORT

        final_waypoint_index = 2

        current_position = msg.pose.pose.position
        current_x = current_position.x
        current_y = current_position.y

        closest_waypoint = None
        closest_index = -1
        min_distance = float("inf")
        
        for i, waypoint in enumerate(waypoints):
            waypoint_x = waypoint.pose.position.x
            waypoint_y = waypoint.pose.position.y
            distance = math.sqrt((waypoint_x - current_x)**2 + (waypoint_y - current_y)**2)

            if distance < min_distance:
                min_distance = distance
                closest_waypoint = waypoint
                closest_index = i

        if closest_index == -1:
            print("No closest waypoint found.")
            return ABORT

        blackboard.closest_waypoint = closest_waypoint
        print('closest_waypoint:', (closest_waypoint.pose.position.x, closest_waypoint.pose.position.y))

        # Clockwise path
        if closest_index <= final_waypoint_index:
            clockwise_path = waypoints[closest_index:final_waypoint_index + 1]
        else:
            clockwise_path = waypoints[closest_index:] + waypoints[:final_waypoint_index + 1]
        
        # Counterclockwise path
        if closest_index > final_waypoint_index:
            counterclockwise_path = waypoints[final_waypoint_index:closest_index + 1]
        else:
            counterclockwise_path = waypoints[final_waypoint_index:] + waypoints[:closest_index + 1]

        # Ensure paths are ordered correctly (start at closest waypoint and end at final waypoint)
        clockwise_path = self.order_path(clockwise_path, closest_index, final_waypoint_index, waypoints)
        counterclockwise_path = self.order_path(counterclockwise_path, closest_index, final_waypoint_index, waypoints)

        clockwise_length = self.path_length(clockwise_path)
        counterclockwise_length = self.path_length(counterclockwise_path)

        if clockwise_length < counterclockwise_length:
            path = clockwise_path
        else:
            path = counterclockwise_path

        blackboard.path = path
        blackboard.ref_point = current_position
        print('path:', [(p.pose.position.x, p.pose.position.y) for p in path])
        
        ref = PoseStamped()
        ref.header.frame_id = "odom"
        ref.pose.position.x = current_position.x
        ref.pose.position.y = current_position.y
        ref.pose.position.z = current_position.z
        self._guidance_ref.publish(ref)

        aruco_pose = Pose()
        aruco_pose.position.x = 0.5
        aruco_pose.position.y = 2.0
        aruco_pose.position.z = 0.0

        blackboard.aruco_pose = aruco_pose

        docking_station = blackboard.waypoint["docking_station"]
        dx = docking_station.pose.position.x - current_x
        dy = docking_station.pose.position.y - current_y
        self.distance = math.sqrt(dx**2 + dy**2)

        log_transition(self.node, blackboard, "GetPath")
        
        if self.distance > 5.0:
            
            return SUCCEED
        else:
            return ABORT

    def order_path(self, path, start_index, end_index, waypoints):
        if start_index <= end_index:
            return path
        else:
            return path[::-1]  # Reverse the path to ensure it starts at the closest and ends at the final waypoint

        


class ExecutingPathHoming(ActionState):
    def __init__(self, node: Node) -> None:
        super().__init__(
            Homing,  # action type
            "/homing",  # action name
            self.create_goal_handler,  # cb to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            None  # cb to process the response
        )
        self.node = node
    
    def create_goal_handler(self, blackboard: Blackboard) -> Homing.Goal:
        log_transition(self.node, blackboard, "ExecutingPath")
        goal = Homing.Goal()
        goal.pose = blackboard.waypoint["docking_station"]
        goal.pose.header.frame_id = "odom"
        return goal
    
class StartScanning(State):
    def __init__(self, node: Node) -> None:
        super().__init__([SUCCEED])

        self.node = node

        self.usbl_client = node.create_client(SetBool, "/sensor/set_usbl")
        self.marker_detection_client = node.create_client(SetBool, "/sensor/set_marker_detection")
        self._stop_pub = node.create_publisher(Empty, "/blueye/stop", 10)

    def execute(self, blackboard: Blackboard) -> str:
        print("Start scanning")
        log_transition(self.node, blackboard, "StartScanning")


        for _ in range(5):
            self._stop_pub.publish(Empty())
            time.sleep(0.1)  # Short delay between messages

        self.set_service(self.marker_detection_client, True)
        self.set_service(self.usbl_client, False)
        
        time.sleep(10)
        return SUCCEED
    
    def set_service(self, client, value):
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f'{client.srv_name} service not available, waiting again...')
        
        request = SetBool.Request()
        request.data = value
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        
        if future.result().success:
            self.node.get_logger().info(f'Service {client.srv_name} set to {"enabled" if value else "disabled"}')
        else:
            self.node.get_logger().error(f'Failed to set service {client.srv_name}')

        

class Scanning(MonitorState):
    def __init__(self, node: Node) -> None:
        super().__init__(Int32,  # msg type
                         "blueye/marker_id",  # topic name
                         [SUCCEED, ABORT],  # outcomes
                         self.monitor_handler,
                         qos=qos_profile_sensor_data,  # monitor handler callback
                         msg_queue=10,  # queue of the monitor handler callback
                         timeout=10  # timeout to wait for msgs in seconds
                         )
        self.node = node
        self.ref_pub = node.create_publisher(DesiredVelocity, "/blueye/desired_velocity", 10)
        self.last_seen_tag = None
        self.processed_markers = set()  # Set to track processed markers

    def monitor_handler(self, blackboard: Blackboard, msg: Int32) -> str:
        print("Scanning for object")
        marker_id = msg.data

        log_transition(self.node, blackboard, "Scanning")

        # Check if the marker has already been processed
        if marker_id in self.processed_markers:
            print("Already processed marker: ", marker_id)
            # No new marker detected or it is already processed, continue scanning
            velocity_cmd = DesiredVelocity()
            velocity_cmd.surge = 0.0
            velocity_cmd.sway = 0.0
            velocity_cmd.heave = 0.0
            velocity_cmd.yaw = 0.5
            self.ref_pub.publish(velocity_cmd)

            return ABORT
        else:
            if marker_id is not None and marker_id != self.last_seen_tag:
                self.last_seen_tag = marker_id
                blackboard.marker_id = marker_id
                self.processed_markers.add(marker_id)  # Add to the set of processed markers
                print("New tag detected: ", blackboard.marker_id)
                return SUCCEED

class ObjectDetected(State):
    """State to detect a specific object (docking station)."""
    def __init__(self, node: Node) -> None:
        super().__init__([END, HAS_NEXT])

        self.node = node

        self.ref_pub = node.create_publisher(DesiredVelocity, "/blueye/desired_velocity", 10)
        self.ref_pub_fsm = node.create_publisher(PoseStamped, "/FSM/reference", 10)
        self.dock_init_pub = node.create_publisher(Bool, "/blueye/docking_init", 10)
        self.already_targeted = False
        

    def execute(self, blackboard: Blackboard) -> str:
        print("Object detected")

        log_transition(self.node, blackboard, "ObjectDetected")

        marker_id = blackboard.marker_id

        # Stop the robot
        self.ref_pub_fsm.publish(PoseStamped())
        

        velocity_cmd = DesiredVelocity()
        velocity_cmd.surge = 0.0
        velocity_cmd.sway = 0.0
        velocity_cmd.heave = 0.0
        velocity_cmd.yaw = 0.0
        self.ref_pub.publish(velocity_cmd)

        if marker_id == 1:
            blackboard.heading = "dock"
            self.dock_init_pub.publish(Bool(data=True))

            return END
        elif marker_id in [16, 17, 18, 19, 20]:
            blackboard.target_marker_id = 1
            blackboard.home_pose = "back"
            return HAS_NEXT
        else:
            blackboard.target_marker_id = 1
            return HAS_NEXT
    
'''        
class AdjustingPositionState(ActionState):
    def __init__(self, node: Node) -> None:
        super().__init__(
            AdjustingPosition,  # action type
            "/adjusting_position",  # action name
            self.create_goal_handler,  # cb to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            None  # cb to process the response
        )
        self.node = node

    
    def create_goal_handler(self, blackboard: Blackboard) -> AdjustingPosition.Goal:
        print("Adjusting position")
        goal = AdjustingPosition.Goal()
        goal.pose = blackboard.front_pose
        
        return goal
'''

class ManeuveringState(ActionState):
    def __init__(self, node: Node) -> None:
        super().__init__(
            Maneuvering,  # action type
            "/maneuvering",  # action name
            self.create_goal_handler,  # cb to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            None  # cb to process the response
        )
        self.node = node
        

    
    def create_goal_handler(self, blackboard: Blackboard) -> Maneuvering.Goal:
        print("Maneuvering")
        log_transition(self.node, blackboard, "AdjustingPosition")
        goal = Maneuvering.Goal()
        goal.poses = blackboard.path
        
        return goal
    

class InitializeDockingState(ActionState):
    def __init__(self, node: Node) -> None:
        super().__init__(
            InitializeDocking,  # action type
            "/initialize_docking",  # action name
            self.create_goal_handler,  # cb to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            None  # cb to process the response
        )
        self.node = node
        self._dock_init_pub = node.create_publisher(Bool, "/blueye/docking_init", 10)
    def create_goal_handler(self, blackboard: Blackboard) -> InitializeDocking.Goal:
        print("Initializing docking")

        log_transition(self.node, blackboard, "InitializeDocking")

        self._dock_init_pub.publish(Bool(data=True))
        goal = InitializeDocking.Goal()
        goal.aruco_pose = blackboard.aruco_pose
        
        return goal
    
class CancelState(State):
    def __init__(self, node: Node) -> None:
        super().__init__([CANCEL])

        self.node = node

        self._stop_pub = node.create_publisher(Empty, "/blueye/stop", 10)

    def execute(self, blackboard: Blackboard) -> str:
        print("Cancelling the action")
        
        log_transition(self.node, blackboard, "Cancel")
        self._stop_pub.publish(Empty())

        time.sleep(3)

        return CANCEL

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def main():
    print("Homing initiated")
    
    # init ROS 2
    rclpy.init()
    node = rclpy.create_node("state_machine_node")

    # create a FSM
    sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])
    nav_sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])
    
    # add states
    sm.add_state(
        "WaitForWp",
        CbState([SUCCEED], create_waypoint),
        transitions={
            SUCCEED: "GetPath",
        }
    )

    sm.add_state(
        "GetPath",
        GetPath(node),
        transitions={
            SUCCEED: "ExecutingPath",
            ABORT: "WaitForWp",
            CANCEL: "Cancel"
        }
    )
    
    sm.add_state(
        "ExecutingPath",
        ExecutingPathHoming(node),
        transitions={
            SUCCEED: "StartScanning",
            CANCEL: "Cancel"
        }
    )

    sm.add_state(
        "StartScanning",
        StartScanning(node),
        transitions={
            SUCCEED: "Scanning"
        }
    )

    sm.add_state(
        "Scanning",
        Scanning(node),
        transitions={
            SUCCEED: "ObjectDetected",
            ABORT: "Scanning",
            CANCEL: "Cancel"
        }
    )
    
    sm.add_state(
        "ObjectDetected",
        ObjectDetected(node),
        transitions={
            END: "InitializeDocking",
            HAS_NEXT: "AdjustingPosition"
        }
    )

    sm.add_state(
        "AdjustingPosition",
        ManeuveringState(node),
        transitions={
            SUCCEED: "InitializeDocking",
            CANCEL: "Cancel"
        }
    )
    
    sm.add_state(
        "InitializeDocking",
        InitializeDockingState(node),
        transitions={
            SUCCEED: SUCCEED,
            CANCEL: "Cancel"
        }
    )

    sm.add_state(
        "Cancel",
        CancelState(node),
        transitions={
            CANCEL: CANCEL
        }
    )

    # pub FSM info
    YasminViewerPub("HOMING", sm)

    # execute FSM
    blackboard = Blackboard()
    blackboard.marker_lock = False
    blackboard.priority_marker_id = False
    blackboard.target_marker_id = None
    blackboard.marker_id_targets = [1, 16, 17, 18, 19, 20]
    
    outcome = sm(blackboard)
    print(outcome)

    # shutdown ROS 2
    rclpy.shutdown()
    node.destroy_node()

if __name__ == "__main__":
    main()
