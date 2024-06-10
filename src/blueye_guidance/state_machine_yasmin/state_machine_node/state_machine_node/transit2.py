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
from blueye_interfaces.action import NavigateToPose
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
from yasmin_ros import MonitorState
from yasmin_viewer import YasminViewerPub
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL

END = "end"

def create_waypoint(blackboard: Blackboard) -> str:
    blackboard.waypoint = {
        "docking_station": [0.0, 0.0, 0.0]}
    
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
        if self.distance > 10.0:
            print("Distance: ", self.distance)
            return SUCCEED
        else:
            return ABORT  
        
        
class ExecutingPath(MonitorState):
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
        if distance < 50.0:
            print("Reached distance of 50 meters from station")
            eta_d = PoseStamped()
            self.ref_pub.publish(eta_d)


            return SUCCEED
        else:
            return ABORT
        

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
