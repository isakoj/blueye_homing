import time
from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL

def create_waypoint(blackboard: Blackboard) -> str:
    blackboard.waypoint = {
        "docking_station": [0.0, 0.0, 0.0]}
    
    time.sleep(3)
    return SUCCEED
    