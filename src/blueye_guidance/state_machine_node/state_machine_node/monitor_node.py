import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import MonitorState
from yasmin_ros.basic_outcomes import CANCEL
from yasmin_viewer import YasminViewerPub


class PrintOdometryState(MonitorState):
    def __init__(self, times: int) -> None:
        super().__init__(
            Odometry,  # msg type
            "odometry/filtered",  # topic name
            ["outcome1", "outcome2"],  # outcomes
            self.monitor_handler,  # monitor handler callback
            qos=qos_profile_sensor_data,  # qos for the topic sbscription
            msg_queue=10,  # queue of the monitor handler callback
            timeout=10  # timeout to wait for msgs in seconds
            # if not None, CANCEL outcome is added
        )
        self.times = times

    def monitor_handler(self, blackboard: Blackboard, msg: Odometry) -> str:
        print(msg)


        if self.times <= 0:
            return "outcome2"

        return "outcome1"


# main
def main():

    print("yasmin_monitor_demo")

    # init ROS 2
    rclpy.init()

    # create a FSM
    sm = StateMachine(outcomes=["outcome4"])

    # add states
    sm.add_state(
        "PRINTING_ODOM",
        PrintOdometryState(5),
        transitions={
            "outcome1": "PRINTING_ODOM",
            "outcome2": "outcome4",
            CANCEL: "outcome4"
        }
    )

    # pub FSM info
    YasminViewerPub("YASMIN_MONITOR_DEMO", sm)

    # execute FSM
    outcome = sm()
    print(outcome)

    # shutdown ROS 2
    rclpy.shutdown()


if __name__ == "__main__":
    main()

