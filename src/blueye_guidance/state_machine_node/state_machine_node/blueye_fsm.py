from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32, Empty, String, Float32
from robot_interfaces.msg import DesiredVelocity

import math
import numpy as np
from transforms3d.euler import euler2quat, quat2euler



import time
import rclpy
from rclpy.node import Node
from blueye_external.yasmin import CbState
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import MonitorState
from yasmin_viewer import YasminViewerPub
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub
from state_machine_node import homing_mk_ii, transit2

HAS_NEXT = "has_next"
END = "end"
HOMING = "homing"
TRANSIT = "transit"

def create_waypoint(blackboard: Blackboard) -> str:
    blackboard.waypoint = {
        "docking_station": [0.0, 0.0, 0.0]
        }
    
    time.sleep(3)
    return SUCCEED
    
class GetPath(MonitorState):
    def __init__(self) -> None:
        super().__init__( Odometry,  # msg type
                         "odometry/filtered",  # topic name
                         [TRANSIT, HOMING],  # outcomes
                         self.monitor_handler,
                         qos = qos_profile_sensor_data,  # monitor handler callback
                         msg_queue=10,  # queue of the monitor handler callback
                         timeout=10  # timeout to wait for msgs in seconds
                                     # if not None, CANCEL outcome is added
                         )
        

    def monitor_handler(self, blackboard: Blackboard, msg: Odometry) -> str:
        # Extract waypoint from the blackboard
        waypoint = blackboard.waypoint["docking_station"]
        
        # Extract robot's current position from the Odometry message
        current_position = msg.pose.pose.position
        
        # Calculate the Euclidean distance
        dx = waypoint[0] - current_position.x
        dy = waypoint[1] - current_position.y
        dz = waypoint[2] - current_position.z
        self.distance = math.sqrt(dx**2 + dy**2 + dz**2)

        heading = math.atan2(dy, dx)
        heading_ref = euler2quat(0, 0, heading, axes='sxyz')



        eta_d = PoseStamped()
        eta_d.header.frame_id = "world_ned"
        eta_d.pose.position.x = waypoint[0]
        eta_d.pose.position.y = waypoint[1]
        eta_d.pose.position.z = waypoint[2]

        eta_d.pose.orientation.x = heading_ref[1]
        eta_d.pose.orientation.y = heading_ref[2]
        eta_d.pose.orientation.z = heading_ref[3]
        eta_d.pose.orientation.w = heading_ref[0]

        blackboard.ref = eta_d
        blackboard.distance = self.distance
        
        # Check if the distance is greater than the threshold
        if self.distance > 50.0:
            print("Distance: ", self.distance, "Transit")
            return TRANSIT
        elif self.distance <= 50.0:
            print("Distance: ", self.distance, "Homing")
            return HOMING



# main
def main():

    print("Blueye FSM")

    # init ROS 2
    rclpy.init()
    node = rclpy.create_node("state_machine_node")


    # create state machines
    sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])
    transit_sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])
    homing_sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])
    dockign_sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])
    

    sm.add_state(
        "WaitForWp",
        CbState([SUCCEED], create_waypoint),
        transitions={
            SUCCEED: "GetPath",
            }
        )
    
    sm.add_state(
        "GetPath",
        GetPath(),
        transitions = {
            TRANSIT: "Transit",
            HOMING: "Homing"
        }
    )

    
    
    transit_sm.add_state(
        "ExecutingPathTransit",
        transit2.ExecutingPath(node),
        transitions={
            SUCCEED: SUCCEED,
            ABORT: "ExecutingPathTransit"
        }
    )

    
    homing_sm.add_state(
        "ExecutingPathHoming",
        homing_mk_ii.ExecutingPathHOMING(node),
        transitions={
            SUCCEED: "Scanning",
            ABORT: "ExecutingPathHoming"
        }
    )

    
    homing_sm.add_state(
        "Scanning",
        homing_mk_ii.Scanning(node),
        transitions={
            SUCCEED: "ObjectDetected",
            ABORT: "Scanning"
        }
    )
    
    homing_sm.add_state(
        "ObjectDetected",
        homing_mk_ii.ObjectDetected(node),
        transitions={
            END: SUCCEED,
            HAS_NEXT: "AdjustingPosition"
            }
    )

    homing_sm.add_state(
        "AdjustingPosition",
        homing_mk_ii.AdjustingPosition(node),
        transitions={
            SUCCEED: "Scanning",
            ABORT: "AdjustingPosition"
                    }
    )
    
    

    # add states
    sm.add_state(
        "Transit",
        transit_sm,
        transitions={
            SUCCEED: "GetPath",
            ABORT: ABORT
        }
    )

    sm.add_state(
        "Homing",
        homing_sm,
        transitions={
            SUCCEED: SUCCEED,
            ABORT: ABORT
        }
    )



    # pub FSM info
    YasminViewerPub("Blueye FSM", sm)

    # execute FSM
    blackboard = Blackboard()
    blackboard.waypoints_num = 2
    outcome = sm(blackboard)
    print(outcome)

    # shutdown ROS 2
    rclpy.shutdown()
    node.destroy_node()


if __name__ == "__main__":
    main()