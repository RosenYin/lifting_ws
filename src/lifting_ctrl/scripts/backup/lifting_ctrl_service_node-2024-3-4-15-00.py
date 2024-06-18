#!/usr/bin/env python3
# 升降柱控制服务端节点
# 同时将接收到的串口数据，publish出去
# 开了一个线程用来pub数据
import rospy
import _thread
import time

from lifting_motor_ctrl import C_LiftingMotorCtrl

from bt_task_msgs.srv  import LiftMotorSrv, LiftMotorSrvRequest, LiftMotorSrvResponse 

from bt_task_msgs.msg import LiftMotorMsg

class C_ROS_Server:
    def __init__(self,) -> None:
        '''
        初始化节点
        '''
        rospy.init_node('lifting_ctrl_server_node', anonymous=True)
        # 外部参数,判断控制哪一个电机编号
        self.motor_id = 4
        if rospy.has_param('~motor_id'):
            self.motor_id = rospy.get_param("~motor_id")
            rospy.loginfo("%s is %d", rospy.resolve_name('~motor_id'), self.motor_id)
        else: rospy.loginfo("未找到") 
        # 电机控制类实例化 
        self.input_callback_time = time.time()
        self.ctrl = C_LiftingMotorCtrl(self.motor_id)
        lift_port = self.ctrl.GetLiftPortName()
        self.add_to_param_list('/liftPort',lift_port)
        # 设置阻塞式回调函数超时时间，默认为300s，即5分钟
        if('responseTimeout' in self.ctrl.GetMotorJsonConfig().keys()):
            self.response_timeout = self.ctrl.GetMotorJsonConfig()["responseTimeout"]
        else: self.response_timeout = 60
        # 设置回调函数模式，如果是1则是立刻返回，0为阻塞式回调
        if('callBackMode' in self.ctrl.GetMotorJsonConfig().keys()):
            self.callBackMode = self.ctrl.GetMotorJsonConfig()["callBackMode"]
        else: self.callBackMode = 1
        if('liftSpd' in self.ctrl.GetMotorJsonConfig().keys()):
            self.liftTargetSpd = self.ctrl.GetMotorJsonConfig()["liftSpd"]
        if('initSpd' in self.ctrl.GetMotorJsonConfig().keys()):
            self.initSpd = self.ctrl.GetMotorJsonConfig()["initSpd"]
        else: self.initSpd = -100
        # 初始化位置
        if('initPos' in self.ctrl.GetMotorJsonConfig().keys()):
            self.__initPos = (self.ctrl.GetMotorJsonConfig()["initPos"])
        else: self.__initPos = 0

        self.motor_states = self.ctrl.ReadMotorData()
        
        self.target_height = self.ctrl.GetTargetHeight()
        self.target_speed = 0
        self.ctrl.LiftMovePos(self.target_height)
        # self.ctrl.MotorFreeVol()
        # self.ctrl.MotorEnableHold()
        # self.ctrl.MotorFree()
        self.mode = 1 # 初始阶段为初始化模式
        self.fps_error=0
        self.back_height = 0
        if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
            self.back_height = self.motor_states.back_lift_height
        self.motor_msgs = LiftMotorMsg()
        self.motor_pub = rospy.Publisher('LiftMotorStatePub', LiftMotorMsg, queue_size=1)
        if(self.callBackMode == 1):
            self.motor_srv = rospy.Service('LiftingMotorService', LiftMotorSrv, self.SververCallbackImmediately)
        elif(self.callBackMode == 0):
            self.motor_srv = rospy.Service('LiftingMotorService', LiftMotorSrv, self.SververCallbackBlock)
        self.first_up_flag = False
        self.first_down_flag = False
        self.previous_dir = 0
        self.command_send = 0
        self.offset = 0
        self.init_state = True
        self.callLock = self.last_callLock = True
        self.timeoutFlag:bool = False

    def add_to_param_list(self, param_name, new_element):
        # 检查参数是否存在
        if not rospy.has_param(param_name):
            # 如果参数不存在，则创建一个新的空列表参数
            rospy.set_param(param_name, [])

        # 获取参数值
        param_list = rospy.get_param(param_name)

        # 检查新元素是否已存在于列表中
        if new_element not in param_list:
            # 如果新元素不存在于列表中，则添加到列表中
            param_list.append(new_element)

            # 更新参数值
            rospy.set_param(param_name, param_list)
            rospy.loginfo("Added {} to param list {}".format(new_element, param_name))
        else:
            rospy.loginfo("{} already exists in param list {}".format(new_element, param_name))
    
    def JudgeFirstUpOrDown(self):
        '''
        判断电机是否切换状态，比如上一次是上升，当前要发送下降，那么判断当前是上升后的第一次下降，反之亦然
        '''
        dir = self.JudgeMotorDirection()
        if(self.command_send == 1):
            self.command_send = 0
            if(dir == 1 and self.previous_dir == -1):
                self.first_up_flag = True
                self.first_down_flag = False
            elif(dir == -1 and self.previous_dir == 1): 
                self.first_up_flag = False
                self.first_down_flag = True
            elif(dir == 1 and self.previous_dir == 1):
                self.first_up_flag = False
            elif(dir == -1 and self.previous_dir == -1):
                self.first_down_flag = False
        if(dir != 0):
            self.previous_dir = dir
        if(self.first_up_flag): 
            self.offset = 3
        if(self.first_down_flag):
            self.offset = -3
        
    def PublishMotorMsgs(self) -> None:
        '''
        将要pub出去的数据赋值
        '''
        if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
            # print("---------------")
            # 电机控制模式
            self.motor_msgs.ctrlMode = self.motor_states.ctrl_mode
            # 电机反馈电压
            self.motor_msgs.backVol = self.motor_states.back_vol
            # 电机反馈电流
            self.motor_msgs.backCurrent = self.motor_states.back_current
            # 电机状态位
            self.motor_msgs.stateBit = self.motor_states.state_bit
                # 电机是否运行，0-停机，1-启动
            self.motor_msgs.runState = (self.motor_msgs.stateBit&0x01)
                # 过流
            self.motor_msgs.overflowI = (bool)(self.motor_msgs.stateBit&0x02)
                # 过压
            self.motor_msgs.overflowV = (bool)(self.motor_msgs.stateBit&0x04)
                # 编码器故障
            self.motor_msgs.encodeErr = (bool)(self.motor_msgs.stateBit&0x8)
                # 位置偏差过大
            self.motor_msgs.posBiasOver = (bool)(self.motor_msgs.stateBit&0x10)
                # 欠压
            self.motor_msgs.underVol = (bool)(self.motor_msgs.stateBit&0x20)
                # 过载标志
            self.motor_msgs.overLoad = (bool)(self.motor_msgs.stateBit&0x40)
                # 外部控制标志 = 0-pc ； 1-外部 PLC
            self.motor_msgs.externalCtrlMode = (bool)(self.motor_msgs.stateBit&0x80)
            # 电机转速
            self.motor_msgs.motorSpeed = self.motor_states.back_speed
            # 升降柱转速abs(round((self.motor_states.back_speed*10000) / (self.ctrl.reductionRatio * 60))
            self.motor_msgs.backSpeed = round((self.motor_states.back_speed*10000) / (self.ctrl.reductionRatio * 60)) # mm/s
            # 电机定位完成
            self.motor_msgs.locationComplete = self.motor_states.location_complete
            # 电机反馈位置
            self.motor_msgs.backPos = self.motor_states.back_pos
            # 升降柱当前高度
            self.motor_msgs.backHeight = self.motor_states.back_lift_height
            # 升降柱目标高度
            # self.motor_msgs.targetHeight = self.ctrl.GetTargetHeight()
            self.motor_msgs.targetHeight = self.ctrl.GetTargetHeight()
            # 电机目标位置
            self.motor_msgs.targetPos = self.ctrl.GetTargetPos()
            # 电机上限位
            self.motor_msgs.upLimit = (self.motor_states.upLimit)
            # 电机下限位
            self.motor_msgs.downLimit = (self.motor_states.downLimit)
        # 电机是否到达了目标位置，误差小于1认为到达了
        if(abs(self.target_height - self.back_height) < 1): 
            self.motor_msgs.reachTargetPos = True
        else: self.motor_msgs.reachTargetPos = False

        self.motor_msgs.fpsError = self.fps_error
            # 发布
        try:
            self.motor_pub.publish(self.motor_msgs)
        except Exception as e:
            print("发布失败",e)
            pass
    
        
    
    def JudgeMotorDirection(self):
        '''
        判断电机方向,1上升，-1下降，0不动
        '''
        # 正向，向上升
        if(self.target_height-self.back_height > 0):
            return 1
        # 向下降
        elif(self.target_height-self.back_height < 0):
            return -1
        # 不动
        else:
            return 0 
    def JudgeMotorDirectionWithSpeed(self):
        '''
        判断电机方向，1正方向，-1负方向，0不动
        '''
        if(self.target_speed > 0):
            # print("电机正方向速度运动")
            return 1
        elif(self.target_speed < 0):
            # print("电机负方向速度运动")
            return -1
        # 不动
        elif(self.target_speed == 0):
            return 0 
        else: return 2
    # 回调函数
    def SververCallbackImmediately(self,req):
        '''
        服务端回调函数,立刻反馈数值，不等待电机到达位置就反馈
        '''
        self.command_send = 1
        self.mode = req.mode
        
        # 初始化模式
        if(req.mode == 1):
            if(self.init_state): resp = 1
            else: resp = -1
        # 位置模式
        elif(req.mode == 0 and self.init_state):
            self.target_height = self.ctrl.SetTargetHeight(req.val)
            rospy.loginfo("target height: %d", self.target_height)
            # 电机是否到达了目标位置，误差小于1认为到达了，反馈1为到达，反馈-1为未到达
            if(abs(self.target_height - self.back_height) < 1): 
                resp = 1
            else: resp = -1
        # 急停
        elif(req.mode == -2 and self.init_state):
            if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                if(self.motor_states.back_speed == 0):
                    resp = 1
            else: resp = -1
        # 初始化，用来另当前位置等于目标位置，如果当前位置-目标位置值小于1,认为数值初始化成功
        elif(req.mode == -3 and self.init_state):
            if(abs(self.target_height - self.back_height) < 1 and self.fps_error == 0):
                resp = 1
            else: resp = -1
        # 恒定速度
        elif(req.mode == -4 and self.init_state):
            self.target_speed = req.val
            if(self.target_speed > self.liftTargetSpd): self.target_speed=self.liftTargetSpd
            if(self.target_speed < -self.liftTargetSpd): self.target_speed=-self.liftTargetSpd
            if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                if(abs(round((self.motor_states.back_speed*10000) / (self.ctrl.reductionRatio * 60)) - self.target_speed)<5):
                    resp = 1
            else: resp = -1
        else:
            resp = 0
        # 如果错误帧率达到1以上
        if(self.fps_error > 3):
            resp = -9
        rospy.loginfo("服务端的resquest为%s", self.mode)
        rospy.loginfo("服务端的应答为%s", resp)
        return LiftMotorSrvResponse(resp)

    def SververCallbackBlock(self,req):
        '''
        服务端回调函数，阻塞式，等待电机到达目标位置才会反馈
        '''
        self.input_callback_time = time.time()
        self.command_send = 1
        
        self.mode = req.mode
        resp = -1
        self.timeoutFlag = False
        if(self.callLock == self.last_callLock):
            # 初始化模式
            while resp == -1:
                self.callLock = not self.last_callLock
                if(req.mode == 1):
                    if(self.init_state): resp = 1
                    else: resp = -1
                # 位置模式
                elif(req.mode == 0 and self.init_state):
                    self.target_height = self.ctrl.SetTargetHeight(req.val)
                    rospy.loginfo("target height: %d", self.target_height)
                    # 电机是否到达了目标位置，误差小于1认为到达了，反馈1为到达，反馈-1为未到达
                    if(abs(self.target_height - self.back_height) < 2): 
                        resp = 1
                    else: resp = -1
                # 急停
                elif(req.mode == -2 and self.init_state):
                    if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                        if(self.motor_states.back_speed == 0):
                            resp = 1
                    else: resp = -1
                # -3 指令用来初始化电机，并清空错误帧率
                elif(req.mode == -3):
                    if(self.init_state): 
                        resp = 1
                    else: resp = -1
                # 恒定速度
                elif(req.mode == -4 and self.init_state):
                    # rospy.loginfo("the max speed is: ",self.liftTargetSpd)
                    self.ctrl.SetTargetHeight(self.back_height)
                    self.target_speed = req.val
                    if(self.target_speed > self.liftTargetSpd): self.target_speed=self.liftTargetSpd
                    if(self.target_speed < -self.liftTargetSpd): self.target_speed=-self.liftTargetSpd
                    if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                        rospy.loginfo(self.motor_states.back_speed)
                        if(abs(round((self.motor_states.back_speed*10000) / (self.ctrl.reductionRatio * 60)) - self.target_speed)<5):
                            resp = 1
                    else: resp = -1
                elif(req.mode == -5 and self.init_state):
                    self.target_speed = req.val
                    if(self.target_speed > self.liftTargetSpd): self.target_speed=self.liftTargetSpd
                    if(self.target_speed < -self.liftTargetSpd): self.target_speed=-self.liftTargetSpd
                    self.target_speed = round((self.target_speed * self.ctrl.reductionRatio * 60)/10000)
                    self.ctrl.MotorSetMaxSpd(self.target_speed)
                    resp = 1
                else:
                    resp = 0
                    if(not self.init_state):
                        resp = -14
                
                if(resp == -1):
                    if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                        if(self.motor_states.upLimit):
                            if(self.mode == 0 and self.JudgeMotorDirection()==-1):
                                resp = -1
                            elif(self.mode == -4 and self.JudgeMotorDirectionWithSpeed()==-1):
                                resp = -1
                            else: resp = -2
                        elif(self.motor_states.downLimit):
                            if(self.mode == 0 and self.JudgeMotorDirection()==1):
                                resp = -1
                            elif(self.mode == -4 and self.JudgeMotorDirectionWithSpeed()==1):
                                resp = -1
                            else: resp = 2
                # 如果错误帧率达到4以上
                if(self.fps_error > 4):
                    resp = -9
                
                if(time.time()  - self.input_callback_time > self.response_timeout):
                    self.mode == -2
                    self.timeoutFlag = True 
                    rospy.logerr("lifting_motor timeout!!!!")
                    resp = -12
                    if(not self.init_state):
                        resp = -13
                    break

                time.sleep(0.5)
        self.last_callLock = self.callLock
        rospy.loginfo("resquest的mode为%s", self.mode)
        rospy.loginfo("服务端的应答为%s", resp)
        return LiftMotorSrvResponse(resp)
    
    def ShowLog(self):
        rospy.loginfo("--------------")
        rospy.loginfo("模式：%d", self.mode)
        rospy.loginfo("--------------")

    def ModeCtrl(self):
        '''
        电机模式控制
        '''
        def OverFlowIProtect():
            if(self.motor_msgs.backCurrent > 11.0 and self.motor_msgs.backSpeed == 0):
                return True
            return False
        if(self.timeoutFlag):
            self.mode == -2
        # 判断初始化标志位，进行电机初始化
        if(not self.init_state): self.mode = 1
        if(self.mode == 1 and self.fps_error <= 3):
            # print("初始化ing...速度为：",self.initSpd)
            self.init_state = self.ctrl.LiftLimitInit(self.initSpd, self.motor_states)
            if(not self.init_state and time.time()  - self.input_callback_time > 10):
                pass
            self.mode = 0xF0
            if(self.init_state): 
                self.mode = 0xF1
                self.target_height = self.__initPos
                time.sleep(1)
        if(self.mode == 0xF1):
            if(self.__initPos > 0):
                self.ctrl.MotorStop(self.JudgeMotorDirection(), self.motor_states, force=False)
                # 运动电机
                self.ctrl.LiftMovePos(self.target_height)
            self.mode = 0xF2
        if(self.mode == -2):# -2 指令用来紧急停止电机
            self.target_speed = 0
            self.ctrl.LiftMoveSpd(self.target_speed)
            self.ctrl.MotorStop(self.JudgeMotorDirectionWithSpeed(), self.motor_states, force=True)
        if(self.mode == -3):# -3 指令用来初始化电机，并清空错误帧率
            self.ctrl.MotorInit(self.motor_id)
            self.fps_error = 0
        if(self.mode == -4 and self.fps_error <= 1):# -4指令用来运动电机指定速度
            self.ctrl.SetTargetHeight(self.back_height)
            self.ctrl.LiftMoveSpd(self.target_speed)
            self.ctrl.MotorStop(self.JudgeMotorDirectionWithSpeed(), self.motor_states, force=False)
        if(self.mode == -5 and self.fps_error <= 1):# -4指令用来运动i电机指定速度
            self.ctrl.MotorSetMaxSpd(self.target_speed)
        if(self.mode == 0 and self.fps_error <= 1):
            self.ctrl.MotorStop(self.JudgeMotorDirection(), self.motor_states, force=False)
            # 运动电机
            self.ctrl.LiftMovePos(self.target_height)
        # 过流保护
        if(OverFlowIProtect()):
            self.ctrl.MotorStop(self.JudgeMotorDirection(), self.motor_states, force=True)
            self.mode == -2
            rospy.logerr("电机电流超过11A并且反馈速度为0，判定是卡住")
            # raise("电机电流超过11A并且反馈速度为0，判定是卡住")
    # 发布电机各种信息线程
    def Thread(self, state) -> None:
        success_fps = 0
        # lasst_time = time.time()
        # count = 0
        while state == 1:
            self.motor_states = self.ctrl.ReadMotorData() #读取电机状态消息
            # ## 返回的字符串消息如果非空，publish电机消息并赋值当前升降柱高度
            # print(self.motor_states)
            if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                self.back_height = self.motor_states.back_lift_height
                success_fps = success_fps + 1
                if(success_fps > 5):  self.fps_error = 0
                self.PublishMotorMsgs()
            # ## 如果反馈消息为空，计算错误帧率
            else: 
                success_fps = 0
                self.fps_error = self.fps_error + 1
            
            self.JudgeFirstUpOrDown()
            # ## 模式控制
            self.ModeCtrl()
            time.sleep(0.02)
    def Main(self, state) -> None:
        '''
        主函数
        '''
        # 创建线程
        _thread.start_new_thread(self.Thread, (state,))
        # 设置循环帧率为8hz

        rospy.spin()


if __name__ == "__main__":
    # 外部参数,判断控制哪一个电机编号
    
    # global_example = 5
    time.sleep(2)
    # 实例化
    server = C_ROS_Server()
    # 调用服务端函数
    server.Main(1)
