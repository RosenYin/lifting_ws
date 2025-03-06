#!/usr/bin/env python3
import rospy
from bt_task_msgs.srv  import LiftMotorSrv, LiftMotorSrvRequest, LiftMotorSrvResponse 

from bt_task_msgs.msg import LiftMotorMsg

class C_ROS_Sub():
    def __init__(self) -> None:
        self.upLimit:bool = False
        self.downLimit:bool = False
        self.lift_motor_state:LiftMotorMsg
        pass
    def Callback(self, data):
        self.lift_motor_state = data
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", self.lift_motor_state.backHeight)
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", self.lift_motor_state)
    def Sub(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('lift_ctrl_monitor_node', anonymous=True)
        # rospy.init_node('PUB', anonymous=True)
        
        rospy.Subscriber("/LiftMotorStatePub", LiftMotorMsg, self.Callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    C_ROS_Sub().Sub()
