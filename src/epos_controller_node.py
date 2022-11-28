#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger

from epos_controller import EposController


class EposControllerNode:
    def __init__(self) -> None:
        self.controller = EposController()
        rospy.Subscriber("/my_robot/epos_cmd_vel", Int32MultiArray, self._cmd_callback)
        rospy.Service("/epos_controller/stop_motors", Trigger, self._stop_motors_cb)
        rospy.Service(
            "/epos_controller/disable_motors", Trigger, self._disable_motors_cb
        )

    def _cmd_callback(self, msg: Int32MultiArray) -> None:
        wheel_speed_left = msg.data[0]
        wheel_speed_right = msg.data[1]
        self.controller.move(wheel_speed_left, wheel_speed_right)

    def _stop_motors_cb(self) -> None:
        self.controller.stop()

    def _disable_motors_cb(self) -> None:
        self.controller.disable_motors()


if __name__ == "__main__":
    rospy.init_node("epos_controller_node")
    try:
        epos_controller = EposControllerNode()
        rospy.loginfo("[epos_controller_node] Node initialized!")
    except rospy.ROSInterruptException:
        pass
    rospy.on_shutdown(epos_controller.disable())
    rospy.spin()
