#!/usr/bin/env python3
import rospy
import time

from bt_task_msgs.srv  import LiftInterfaceSrv, LiftInterfaceSrvRequest, LiftInterfaceSrvResponse 
from bt_task_msgs.srv  import LiftMotorSrv, LiftMotorSrvRequest, LiftMotorSrvResponse 
from bt_task_msgs.msg import LiftMotorMsg

class C_Server_Interface:
    def __init__(self) -> None:
        rospy.init_node('lifting_ctrl_interface_node')
        motor_srv = rospy.Service('LiftingMotorService', LiftInterfaceSrv, self.SververCallbackBlock)
        self.motor_sub = None
        self.motor_pub = None
    def SubCallBack(self, msg):
        self.motor_pub = rospy.Publisher('/LiftMotorStatePub', LiftMotorMsg, queue_size=3)
        self.motor_pub.publish(msg)

    def SubscribeWithWaitForMessage(self, topic_name, timeout=3):
        try:
            msg = rospy.wait_for_message(topic_name, LiftMotorMsg, timeout=rospy.Duration(timeout))
            # self.SubCallBack(msg)
            return True
        except rospy.ROSException as e:
            print(f"Error: {e}")
            return False
    def SververCallbackBlock(self, req):
        response_ = LiftInterfaceSrvResponse()
        self.val = req.val
        self.mode = req.mode
        self.id = req.id
        sub_service_name = f'{self.id}' + '/LiftingMotorService'
        sub_topic_name = f'{self.id}' + '/LiftMotorStatePub'
        if(self.mode == -10):
            if(self.SubscribeWithWaitForMessage(sub_topic_name)):
                if(self.motor_sub != None and self.motor_pub!=None):
                    self.motor_pub.unregister()
                    self.motor_sub.unregister()
                self.motor_sub = rospy.Subscriber(sub_topic_name, LiftMotorMsg, callback=self.SubCallBack,queue_size=3)
                response_.message = "Lift interface Start pub"
                response_.success = True
                response_.code  = 13005
            else:
                response_.message = "Lift interface Start pub failed"
                response_.success = False
                response_.code  = 13007
            return response_
        if(self.mode == -11):
            if(self.motor_sub != None and self.motor_pub!=None):
                self.motor_pub.unregister()
                self.motor_sub.unregister()
                self.motor_sub = self.motor_pub = None
                response_.message = "Lift interface Stop pub successfully"
                response_.success = True
                response_.code  = 13011
            else:
                response_.message = "Lift interface Stop pub failed"
                response_.success = False
                response_.code  = 13012
            return response_
        rospy.wait_for_service(sub_service_name)  # 等待服务可用
        # 创建服务代理
        service_proxy = rospy.ServiceProxy(sub_service_name, LiftMotorSrv)
        # 设置超时时间（以秒为单位）
        timeout = rospy.Duration.from_sec(100.0)  # 100秒超时
        response = service_proxy(self.val, self.mode)
        # print("--------------------------")
        # rospy.loginfo(response.state)
        # rospy.loginfo(req.val)
        # rospy.loginfo(req.mode)
        # print("--------------------------")
        if(response.state == 1):
            response_.message = "Lift execut command successfully"
            response_.success = True
            response_.code  = 13000
        elif(response.state == -1):
            response_.message = "[error] Lift execut command failed"
            response_.success = False
            response_.code  = 13001
        elif(response.state == 2):
            response_.message = "[error] Lift touched downLimit"
            response_.success = False
            response_.code  = 13002
        elif(response.state == -2):
            response_.message = "[error] Lift touched upLimit"
            response_.success = False
            response_.code  = 13003
        elif(response.state == -9):
            response_.message = "[error] Lift motor(port) Loss"
            response_.success = False
            response_.code  = 13004
        elif(response.state == -12):
            response_.message = "[error] Lift execute timeout!!!"
            response_.success = False
            response_.code  = 13007
        elif(response.state == -13):
            response_.message = "[error] Lift execute timeout & init failed!!!"
            response_.success = False
            response_.code  = 13007
        elif(response.state == -14):
            response_.message = "[error] Lift init failed!!!"
            response_.success = False
            response_.code  = 13008
        else:
            response_.message = "[warning] Lift mode code unknown"
            response_.success = False
            response_.code  = 13010
        
        # if(self.mode == -10): 
        #     response_.message = "Lift interface Start pub"
        #     response_.success = True
        #     response_.code  = 13005
        # elif(self.mode == -11):
        #     response_.message = "Lift interface Stop pub, remain the topic name"
        #     response_.success = True
        #     response_.code  = 13006
        return response_
        # return LiftInterfaceSrvResponse(response)
if __name__ == "__main__":

    try:
        C_Server_Interface()
        rospy.spin()
        pass
        # 在这里处理服务响应
    except rospy.ServiceException as e:
        # 处理超时或其他错误
        print(f"Service call failed: {e}")
