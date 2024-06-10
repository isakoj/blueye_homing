from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import Header
from blueye_interfaces.action import NavigateToPose, NavigateWaypoints
from rclpy.qos import qos_profile_sensor_data

import math
import numpy as np
from transforms3d.euler import euler2quat, quat2euler


import time
import rclpy
from rclpy.node import Node
from yasmin import CbState
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import MonitorState, ActionState
from yasmin_viewer import YasminViewerPub
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL

END = "end"


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
    # Create the docking station waypoint
    docking_station = PoseStamped()
    docking_station.header.frame_id = "odom"
    docking_station.pose.position.x = 0.0
    docking_station.pose.position.y = 0.0
    docking_station.pose.position.z = 0.0
    docking_station.pose.orientation.w = 1.0

    # Store the docking station in the blackboard
    blackboard.waypoint = {
        "docking_station": docking_station
    }

    # Define the mission waypoints for the lawnmower pattern
    waypoints = [
            [3, 0],
            [3, 3],
            [0, 3],
            [0, 0],
            [30, 40],
            [30, 15],
    ]

    # Create a list of PoseStamped for the mission waypoints
    mission_waypoints = []
    for wp in waypoints:
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.pose.position.x = float(wp[0])
        pose.pose.position.y = float(wp[1])
        pose.pose.position.z = 0.0  # Desired depth pose
        pose.pose.orientation.w = 1.0
        mission_waypoints.append(pose)
    
    # Store the mission waypoints in the blackboard
    blackboard.mission_waypoints = mission_waypoints

    # Wait for 3 seconds before returning SUCCEED
    time.sleep(3)
    return SUCCEED



class Mission(ActionState):
    def __init__(self, node: Node) -> None:
        super().__init__(
            NavigateWaypoints,  # action type
            "/navigate_waypoints",  # action name
            self.create_goal_handler,  # cb to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            None  # cb to process the response
        )
        self.node = node
    
    def create_goal_handler(self, blackboard: Blackboard) -> NavigateWaypoints.Goal:
        self.node.get_logger().info("Creating goal...")

        log_transition(self.node, blackboard, "Mission")

        waypoints = blackboard.mission_waypoints
        
        goal = NavigateWaypoints.Goal()
        goal.poses = waypoints  # Directly assign the list of PoseStamped
        return goal
    
    
class GetPath(MonitorState):
    def __init__(self, node: Node) -> None:
        super().__init__(Odometry,  # msg type
                         "odometry/filtered",  # topic name
                         [SUCCEED, ABORT],  # outcomes
                         self.monitor_handler,
                         qos=qos_profile_sensor_data,  # monitor handler callback
                         msg_queue=10,  # queue of the monitor handler callback
                         timeout=10  # timeout to wait for msgs in seconds
                         )
        self.node = node
        self._guidance_ref = node.create_publisher(PoseStamped, '/guidance/ref', 10)
        self.guidance_ref = None

    def monitor_handler(self, blackboard: Blackboard, msg: Odometry) -> str:
        # Extract waypoint from the blackboard
        waypoint = blackboard.waypoint["docking_station"]

        log_transition(self.node, blackboard, "GetPath")

        # Extract robot's current position from the Odometry message
        current_position = msg.pose.pose.position

        blackboard.ref_point = current_position

        # Calculate the Euclidean distance
        dx = waypoint.pose.position.x - current_position.x
        dy = waypoint.pose.position.y - current_position.y
        dz = waypoint.pose.position.z - current_position.z
        self.distance = math.sqrt(dx**2 + dy**2 + dz**2)

        blackboard.distance = self.distance

        guidance_ref = PoseStamped()
        guidance_ref.header.frame_id = "odom"
        guidance_ref.pose.position.x = current_position.x
        guidance_ref.pose.position.y = current_position.y
        guidance_ref.pose.position.z = current_position.z
        self._guidance_ref.publish(guidance_ref)

        # Check if the distance is greater than the threshold
        if self.distance > 10.0:
            print("Distance: ", self.distance)
            return SUCCEED
        else:
            return ABORT

        
        

class ExecutingPath(ActionState):
    def __init__(self, node: Node) -> None:
        super().__init__(
            NavigateToPose,  # action type
            "/navigate_to_pose",  # action name
            self.create_goal_handler,  # cb to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            None  # cb to process the response
        )
        self.node = node
    
    def create_goal_handler(self, blackboard: Blackboard) -> NavigateToPose.Goal:

        log_transition(self.node, blackboard, "ExecutingPath")
        
        goal = NavigateToPose.Goal()
        goal.pose = blackboard.waypoint["docking_station"]
        goal.pose.header.frame_id = "odom"
        return goal

def normalize_angle(angle):
    """ Normalize an angle to the range of -pi to pi. """
    return (angle + np.pi) % (2 * np.pi) - np.pi



def main():

    print("Transit to distance of 50 meters from station")

    
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
            SUCCEED: "Mission",
            }
        )
    
    sm.add_state(
        "Mission",
        Mission(node),
        transitions={
            SUCCEED: "GetPath",
            ABORT: "WaitForWp"
        }
    )

    sm.add_state(
        "GetPath",
        GetPath(node),
        transitions={
            SUCCEED: "ExecutingPath",
            ABORT: "WaitForWp"
        }
    )
    
    sm.add_state(
        "ExecutingPath",
        ExecutingPath(node),
        transitions={
            SUCCEED: SUCCEED,
            ABORT: "ExecutingPath"
        }
    )

    # pub FSM info
    YasminViewerPub("TRANSIT", sm)

    # execute FSM
    
    
    blackboard = Blackboard()   
    outcome = sm(blackboard)
    print(outcome)

    # shutdown ROS 2
    rclpy.shutdown()
    node.destroy_node()


if __name__ == "__main__":
    main()
