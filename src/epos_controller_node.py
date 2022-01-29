#! /usr/bin/env python3

import rospy
import time
from epos_controller import EposController

from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

class EposControllerNode:
    def __init__(self) -> None:
        self.epos_ctrl = EposController()
        rospy.Subscriber('/my_robot/epos_cmd_vel', Float64MultiArray, self.cmd_callback)
        rospy.Service('/epos_controller/stop', Trigger, self.stop)
        rospy.Service('/epos_controller/disable_motors', Trigger, self.disable)

    def cmd_callback(self, msg):
        Vl = msg.data[0]
        Vr = msg.data[1]
        self.epos_ctrl.move(int(Vl), int(Vr))

    def disable(self):
        self.epos_ctrl.disable_motors()
    
    def stop(self):
        self.epos_ctrl.stop()

if __name__=="__main__":
    rospy.init_node('epos_controller_node')
    try:
        epos_controller = EposControllerNode()
        rospy.loginfo('epos_controller_node initialized!')
    except rospy.ROSInterruptException:
        pass
    rospy.on_shutdown(epos_controller.diable())
    rospy.spin()