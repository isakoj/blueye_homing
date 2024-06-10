#!/usr/bin/env python3

# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32, Empty, String
from blueye_interfaces.msg import DesiredVelocity

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
from yasmin_ros import MonitorState
from yasmin_viewer import YasminViewerPub
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL

END = "end"
HAS_NEXT = "has_next"

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
                         [SUCCEED, ABORT],  # outcomes
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
        if self.distance > 5.0:
            print("Distance: ", self.distance)
            return SUCCEED
        else:
            return ABORT  
        
        
class ExecutingPathHOMING(MonitorState):
    def __init__(self, node: Node) -> None:
        super().__init__( Odometry,  # msg type
                            "odometry/filtered",  # topic name
                            [SUCCEED, ABORT],  # outcomes
                            self.monitor_handler,
                            qos = qos_profile_sensor_data,  # monitor handler callback
                            msg_queue=10,  # queue of the monitor handler callback
                            timeout=10  # timeout to wait for msgs in seconds
                                        # if not None, CANCEL outcome is added
                            )
        
        self.ref_pub = node.create_publisher(PoseStamped, "/FSM/reference", 10)
        
    def monitor_handler(self, blackboard: Blackboard, msg: Odometry) -> str:
        # Extract waypoint from the blackboard
        waypoint = blackboard.waypoint["docking_station"]
        
        # Extract robot's current position from the Odometry message
        current_position = msg.pose.pose.position
        blackboard.current_position = current_position
        
        # Calculate the Euclidean distance
        dx = waypoint[0] - current_position.x
        dy = waypoint[1] - current_position.y
        dz = waypoint[2] - current_position.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        heading = math.atan2(dy, dx)
        heading_norm = normalize_angle(heading)
        heading_ref = euler2quat(0, 0, heading_norm, axes='sxyz')

        # Publish the reference waypoint
        eta_d = PoseStamped()
        eta_d.header.frame_id = "world_ned"
        eta_d.pose.position.x = waypoint[0]
        eta_d.pose.position.y = waypoint[1]
        eta_d.pose.position.z = waypoint[2]

        eta_d.pose.orientation.x = heading_ref[1]
        eta_d.pose.orientation.y = heading_ref[2]
        eta_d.pose.orientation.z = heading_ref[3]
        eta_d.pose.orientation.w = heading_ref[0]

        self.ref_pub.publish(eta_d)
        print("Distance: ", distance)
        
        # Check if the distance is greater than the threshold
        if distance < 3.0:
            print("Reached distance of 3 meters from station")
            eta_d = None


            return SUCCEED
        else:
            return ABORT
        

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
        self.ref_pub = node.create_publisher(DesiredVelocity, "/blueye/desired_velocity", 10)
        self.last_seen_tag = None
        self.processed_markers = set()  # Set to track processed markers

    def monitor_handler(self, blackboard: Blackboard, msg: Int32) -> str:
        print("Scanning for object")
        marker_id = msg.data

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

        self.ref_pub = node.create_publisher(DesiredVelocity, "/blueye/desired_velocity", 10)
        self.ref_pub_fsm = node.create_publisher(PoseStamped, "/FSM/reference", 10)
        self.already_targeted = False
        

    def execute(self, blackboard: Blackboard) -> str:
        print("Object detected")

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
            return END
        elif marker_id in [19, 20]:
            blackboard.target_marker_id = 18
            blackboard.heading = "south"
            blackboard.control = "left"
            return HAS_NEXT
        elif marker_id == 18:
            blackboard.target_marker_id = 17
            blackboard.heading = "west"
            blackboard.control = "left"
            print("Heading set to west")
            return HAS_NEXT
        elif marker_id == 17:
            blackboard.target_marker_id = 1
            blackboard.heading = "north"
            blackboard.control = "left"
            return HAS_NEXT
        elif marker_id == 16:
            blackboard.target_marker_id = 1
            blackboard.heading = "east"
            blackboard.control = "right"
            return HAS_NEXT
        
class AdjustHeading(MonitorState):
    def __init__(self, node: Node) -> None:
        super().__init__( Odometry,  # msg type
                            "odometry/filtered",  # topic name
                            [SUCCEED, ABORT],  # outcomes
                            self.monitor_handler,
                            qos = qos_profile_sensor_data,  # monitor handler callback
                            msg_queue=10,  # queue of the monitor handler callback
                            timeout=10  # timeout to wait for msgs in seconds
                                        # if not None, CANCEL outcome is added
                            )
        
        self.ref_pub = node.create_publisher(PoseStamped, "/FSM/yaw_reference", 10)
        self.heading = None
    
    def monitor_handler(self, blackboard: Blackboard, msg: Odometry) -> str:
        
        self.heading = blackboard.heading
        
        orientation = msg.pose.pose.orientation
        _, _, yaw = quat2euler([orientation.w, orientation.x, orientation.y, orientation.z])
        
        if self.heading == "south":
            print("Desired heading: south")
            desired_heading = -1.571
        elif self.heading == "north":
            print("Desired heading: north")
            desired_heading = 1.571
        elif self.heading == "east":
            print("Desired heading: east")
            desired_heading = 0.0
        elif self.heading == "west":
            print("Desired heading: west")
            desired_heading = -3.0
        
        # Check if the current heading is within a small angle of the desired heading
        if abs(desired_heading - yaw) < 0.1:
            return SUCCEED
        
        quat = euler2quat(0, 0, (desired_heading), axes='sxyz')
        pose = PoseStamped()
        pose.pose.orientation.z = quat[3]
        pose.pose.orientation.w = quat[0]
        self.ref_pub.publish(pose)
        
        return ABORT


class AdjustingPosition(MonitorState):
    def __init__(self, node: Node) -> None:
        super().__init__(Int32,  # msg type
                         "blueye/marker_id",  # topic name
                         [SUCCEED, ABORT],  # outcomes
                         self.monitor_handler,
                         qos=qos_profile_sensor_data,  # monitor handler callback
                         msg_queue=10,  # queue of the monitor handler callback
                         timeout=10)  # timeout to wait for msgs in seconds
        self.ref_pub = node.create_publisher(PoseStamped, "/FSM/yaw_reference", 10)
        self.vel_pub = node.create_publisher(DesiredVelocity, "/blueye/desired_velocity", 10)
        self.direction_pub = node.create_publisher(String, "/blueye/direction", 10)
        self.target_marker_id = None
        self.control = None
        self.heading = None  # Ensure control is initialized

    def monitor_handler(self, blackboard: Blackboard, msg: Int32) -> str:
        print("Adjusting position")
        self.target_marker_id = blackboard.target_marker_id
        self.control = blackboard.control
        self.heading = blackboard.heading

        if msg.data == self.target_marker_id:
            self.control = "stop"
            self.adjust_position(self.control)
            return SUCCEED
        else:
            if self.control:
                self.adjust_position(self.control)
            return ABORT

    def adjust_position(self, direction):
        pose = PoseStamped()

        if self.heading == "south":
            quat = euler2quat(0, 0, normalize_angle(-1.571), axes='sxyz')
        elif self.heading == "north":
            quat = euler2quat(0, 0, normalize_angle(1.571), axes='sxyz')
        elif self.heading == "east":
            quat = euler2quat(0, 0, 0.0, axes='sxyz')
        elif self.heading == "west":
            # Tried to use np.pi but that made it spin in the opposite direction
            quat = euler2quat(0, 0, -(3), axes='sxyz')
        pose.pose.orientation.z = quat[3]
        pose.pose.orientation.w = quat[0]
        self.ref_pub.publish(pose)

        direction_msg = String()
        direction_msg.data = direction
        self.direction_pub.publish(direction_msg)


class InitalizeDocking(State):
    pass

def normalize_angle(self, angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi
            


def main():

    print("Homing initiated to distance of 50 meters from station")

    
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
        GetPath(),
        transitions={
            SUCCEED: "ExecutingPath",
            ABORT: "WaitForWp"
        }
    )
    
    sm.add_state(
        "ExecutingPath",
        ExecutingPathHOMING(node),
        transitions={
            SUCCEED: "Scanning",
            ABORT: "ExecutingPath"
        }
    )

    
    sm.add_state(
        "Scanning",
        Scanning(node),
        transitions={
            SUCCEED: "ObjectDetected",
            ABORT: "Scanning"
        }
    )
    
    sm.add_state(
        "ObjectDetected",
        ObjectDetected(node),
        transitions={
            END: SUCCEED,
            HAS_NEXT: "AdjustHeading"
            }
    )

    sm.add_state(
        "AdjustHeading",
        AdjustHeading(node),
        transitions={
            SUCCEED: "AdjustingPosition",
            ABORT: "AdjustHeading"
        }
    )
    
    sm.add_state(
        "AdjustingPosition",
        AdjustingPosition(node),
        transitions={
            SUCCEED: "Scanning",
            ABORT: "AdjustingPosition"
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
