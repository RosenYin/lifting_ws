#!/usr/bin/env python3
import rospy
import time

from bt_task_msgs.srv  import LiftInterfaceSrv, LiftInterfaceSrvRequest, LiftInterfaceSrvResponse 
from bt_task_msgs.srv  import LiftMotorSrv, LiftMotorSrvRequest, LiftMotorSrvResponse 
from bt_task_msgs.msg import LiftMotorMsg
import random

# 生成在 [start, end] 范围内的整数
start1 = 0
end1 = 400
start2 = 401
end2 = 800
random_int1 = random.randint(start1, end1)
random_int2 = random.randint(start2, end2)
print("整数随机数1:", random_int1)
print("整数随机数2:", random_int2)

rospy.init_node('lifting_motor_client_node')
rospy.wait_for_service('/LiftingMotorService')  # 等待服务可用

# 创建服务代理
service_proxy = rospy.ServiceProxy('/LiftingMotorService', LiftInterfaceSrv)

# 设置超时时间（以秒为单位）
timeout = rospy.Duration.from_sec(3.0)  # 3秒超时

try:
    req = LiftInterfaceSrvRequest()
    req.mode = 0
    req.val = 0
    req.id = 'lift_404'
    response = service_proxy(req)
    last_time = time.time()
    reach_pos_flag = False
    while(True):
        response = service_proxy(req)
        rospy.loginfo(response.message)
        rospy.loginfo(response.success)
        rospy.loginfo(response.code)
        if(response.success == False):
            reach_pos_flag = False
        elif(response.success == True):
            reach_pos_flag = True

        if(reach_pos_flag == True):
            random_int1 = random.randint(start1, end1)
            random_int2 = random.randint(start2, end2)
            time.sleep(1)
            req.mode = 0
            if(req.val < 401): req.val = random_int2
            elif(req.val >400): req.val = random_int1
            reach_pos_flag = False

        time.sleep(0.1)
    # 在这里处理服务响应
except rospy.ServiceException as e:
    # 处理超时或其他错误
    print(f"Service call failed: {e}")
