#!/usr/bin/env python3
import rospy
import time

from bt_task_msgs.srv  import LiftMotorSrv, LiftMotorSrvRequest, LiftMotorSrvResponse 

from bt_task_msgs.msg import LiftMotorMsg

rospy.init_node('lifting_motor_client_node')
rospy.wait_for_service('/LiftingMotorService')  # 等待服务可用

# 创建服务代理
service_proxy = rospy.ServiceProxy('/LiftingMotorService', LiftMotorSrv)

# 设置超时时间（以秒为单位）
timeout = rospy.Duration.from_sec(3.0)  # 3秒超时

try:
    val = 0
    mode = 0
    response = service_proxy(val, mode)
    last_time = time.time()
    reach_pos_flag = False
    while(True):

        response = service_proxy(val, mode)
        rospy.loginfo(response.state)
        if(response.state == -1):
            reach_pos_flag = False
        elif(response.state == 1):
            reach_pos_flag = True

        if(reach_pos_flag == True):
            time.sleep(1)
            mode = 0
            if(val == 0): val = 800
            elif(val == 800): val = 0
            reach_pos_flag = False

        time.sleep(0.1)
    # 在这里处理服务响应
except rospy.ServiceException as e:
    # 处理超时或其他错误
    print(f"Service call failed: {e}")
