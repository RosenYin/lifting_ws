#!/usr/bin/env python3
# -*-coding:utf8-*-

import time

from serial_encapsulation import C_SerialEncapsulation

from json_config import C_JsonConfig

from crc_check import C_CRC


class C_DealDataBase():

    def __init__(self, motor_id:int) -> None:
        self.sensor_serial = C_SerialEncapsulation()
        json_name = str(motor_id) + '_lifting_motor_config.json'
        print("读取文件： ",json_name)
        self.__config = C_JsonConfig().GetDataFromJson(json_name)
        super().__init__()
    
    def GetPortJsonConfig(self):
        return self.__config
    def GetWorkablePort(self):
        return self.sensor_serial.Print_Used_Com()
    # 处理发送数据，主要是crc计算
    def DealSendData(self, tx_list:list, crc_data_len:int, crc_flag:bool=True):
        '''
        处理发送数据，主要是crc计算
        '''
        # 获取要发送的列表
        tx_data = tx_list
        # 如果需要进行crc校验的话
        if(crc_flag):
            # 如果发送的列表长度大于等于要计算的crc索引长度，将发送列表截取到crc索引长度
            if(len(tx_list) >= crc_data_len):
                tx_data = tx_list[:crc_data_len]
            # 如果发送的列表长度小于要计算的crc索引长度，抛出异常
            else:
                raise IndexError("待计算CRC数据的长度小于CRC索引长度")
            # 使用MODBUS规定的CRC16校验
            crc16 = C_CRC().CalCRC16(tx_data, crc_data_len, 0x8005, 0xFFFF, True, 0x0000)
            # 将crc16校验值结果分割成高位和低位存在列表中
            crc16 = C_CRC().CrcSplit(16, "int", crc16)
            # 在发送数据后增加
            tx_data.append(crc16[0])
            tx_data.append(crc16[1])
        return tx_data
    def __ReceiveAndSendSensorData(self, txdata:list, port_name=None, data_length:int=-2, wait_time:int = 0):
        '''
        串口收发，发送一帧消息后立刻接收
        '''
        # 配置串口参数
        if(port_name == None):
            self.sensor_serial.PortParamConfig(json_data = self.__config)
        else: self.sensor_serial.PortParamConfig(port_name, self.__config)
        rxdata = []
        # 打开串口，打开失败就直接返回空列表
        if(self.sensor_serial.PortOpen()):
            # 串口发送
            self.sensor_serial.PortSendListData(txdata, auto_open=False)
            if(wait_time > 0):
                time.sleep(wait_time)
            # print(txdata)
            # 串口接收
            rxdata = self.sensor_serial.PortReadContinuousData(auto_open=False, data_len=data_length)
            # rxdata = self.sensor_serial.PortReadSizeData(auto_open=False)
            self.sensor_serial.PortClose()
        return rxdata
    def DealAllData(self, tx_list:list, crc_data_len:int, crc_flag:bool=True, port_name=None, rxdata_len:int=-2, wait_time:int = 0):
        '''
        处理接收到的传感器数据，数据通过了CRC就会返回列表
        '''
        # 调用类内发送数据处理函数
        txdata = self.DealSendData(tx_list, crc_data_len, crc_flag)
        # print("------------")
        # print("1111=======22222",txdata)
        # 调用类内收发数据函数
        if(port_name == None):
            rxdata = self.__ReceiveAndSendSensorData(txdata, rxdata_len, wait_time)
        else: rxdata = self.__ReceiveAndSendSensorData(txdata, port_name, rxdata_len, wait_time)
        # print("-----",rxdata)
        rxlist = rxdata
        length = len(rxlist)
        # 如果接收到的列表不为空列表
        if(length > 2):
            # 计算接收消息的crc
            crc16 = C_CRC().CalCRC16(rxlist, length-2, 0x8005, 0xFFFF, True, 0x0000)
            crc16 = C_CRC().CrcSplit(16, "int", crc16)
            # 将计算的与接收到的进行校验
            if(crc16[1] == rxlist[-1] and crc16[0] == rxlist[-2]):
                # print(rxlist)
                # 问讯应答帧
                if(rxlist[1] == 0x03):
                    # 第3位为数据长度位
                    data_len = rxlist[2]
                    rxlist = rxlist[3:3+(data_len)]
                    return rxlist
            else: 
                pass


class C_LiftingMotorCtrl():
    class motor_msg():
        ctrl_mode:int
        back_vol:int
        back_current:float
        state_bit:int
        back_speed:int
        location_complete:int
        target_pos:int
        back_pos:int
        back_lift_height:int
        upLimit:bool = False
        downLimit:bool = False

    def __init__(self, motor_id:int, ignore_port:str=None) -> None:
        super().__init__()
        self.__motor_id = motor_id
        self.__deal_data = C_DealDataBase(motor_id)
        # 设定电机所在的端口
        if('portName' in self.GetMotorJsonConfig().keys()):
            self.__port_name=self.GetMotorJsonConfig()["portName"]
        else: self.__port_name = self.JudgeCurrentMotorPort(ignore_port)
        print("------self.__port_name-------",self.__port_name)
        # 设定电机启停时间
        if('motorTime' in self.GetMotorJsonConfig().keys()):
            self.__motor_time = self.GetMotorJsonConfig()["motorTime"]
        else: 
            self.__motor_time = 0
        # 设置电机最大速度
        if('motorSpd' in self.GetMotorJsonConfig().keys()):
            self.motorSpd = self.GetMotorJsonConfig()["motorSpd"]
        else: 
            self.motorSpd = 1000
        # 电机减速比，执行末端运行1mm，电机执行多少圈数n * 10000 = 脉冲数
        if('reductionRatio' in self.GetMotorJsonConfig().keys()):
            self.reductionRatio = self.GetMotorJsonConfig()["reductionRatio"]
        else: self.reductionRatio = 10000
        print("设定 ",self.__motor_id, " 号电机当前减速比为: ",self.reductionRatio,"pause")
        print("设定 ",self.__motor_id," 号电机经减速比转换后的速度最大值为: ",abs(round((self.motorSpd * 10000) / (self.reductionRatio * 60), 3)), "mm/s")
        # 电机向正向运行的最大位置
        if('upLimitVal' in self.GetMotorJsonConfig().keys()):
            self.__upLimitVal = self.GetMotorJsonConfig()["upLimitVal"]
        else: self.__upLimitVal = 400
        # 电机负向运行的最小位置
        if('downLimitVal' in self.GetMotorJsonConfig().keys()):
            self.__downLimitVal = self.GetMotorJsonConfig()["downLimitVal"]
        else: self.__downLimitVal = 0
        # 自定义限制电流保护
        if('motorStallCurrent' in self.GetMotorJsonConfig().keys()):
            self.__current_limit = self.GetMotorJsonConfig()["motorStallCurrent"]
        else: self.__current_limit = 11
        # 创建电机消息类
        self.__motor_msgs = self.motor_msg()
        self.__motor_msgs = self.ReadMotorData()
        self.stop_flag:bool = False
        # 初始化目标位置和目标高度
        if isinstance(self.__motor_msgs, self.motor_msg):
            self._target_pos = self.__motor_msgs.back_pos
            self._target_height = self.__motor_msgs.back_lift_height
        else: 
            self._target_pos = 0
            self._target_height = 0
        self.motor_list = []
        self.limit_list = []
        self.offset = 0

    def MotorInit(self, motor_id:int):
        self.__init__(motor_id)

    def MotorConfigInit(self):
        # 开启电机
        self.StartCtrlMotor()
        # 电机设置绝对位置模式
        self.MotorSetAbsoluteMode()
        # 清除故障
        self.MotorClearBug()
        # 开启通讯中断自动停机
        self.MotorAutoStopStart()
        # 设定电机启停时间
        self.MotorSetTime(self.__motor_time)
        # self.MotorSetSpeedTime(self.__motor_time)
        # 设置电机最大速度
        self.MotorSetMaxSpd(abs(self.motorSpd))

    def GetMotorJsonConfig(self):
        return self.__deal_data.GetPortJsonConfig()
    def GetLiftPortName(self):
        return self.__port_name
    def GetPosOffset(self):
        return self.pos_offset
    def GetMotorMaxSpd(self):
        return self.motorSpd
    def GetOverflowILimit(self):
        return self.__current_limit
    
    def JudgeCurrentMotorPort(self, ignore_port:str=None):
        workable_port_list = C_DealDataBase(self.__motor_id).GetWorkablePort()
        print(workable_port_list)
        print("即将寻找设备 ",self.__motor_id)
        is_cur_port = False
        if(len(workable_port_list)!=0):
            for l in workable_port_list :
                if(l[:11] == '/dev/ttyUSB' and l!=ignore_port):
                    print("--------", self.__motor_id,"--",l)
                    if(self.__deal_data.sensor_serial.is_serial_port_available(l)):
                        for i in range(10):
                            time.sleep(0.2)
                            txlist = [self.__motor_id, 0x03, 0x00, 0xE0, 0x00, 0x0A]
                            rxdata = self.__deal_data.DealAllData(txlist, 6, True, l, 25)
                            print("----", self.__motor_id,"--",rxdata)
                            if(rxdata is not None and len(rxdata)==20):
                                is_cur_port = True
                            if(is_cur_port == True):
                                # print(rxdata)
                                print("找到设备",self.__motor_id,",端口名为", l)
                                return l
                    else: print("无法打开串口")
            print("未找到设备",self.__motor_id,)
            exit(0)
    
    def ReadMotorData(self)->motor_msg:
        '''
        读取电机有关所有信息，需要循环读取
        '''
        def convert_to_valid_32bit(value):
            '''
            接受一个32位整数值，然后检查它是否溢出了32位有符号整数的范围。
            如果是溢出值，它会将其转换为对应的有效值。
            如果是正溢出，它会减去两倍的32位最大值，如果是负溢出，它会加上两倍的32位最大值，以得到对应的有效值。
            这种方法可以将溢出的值转换为32位有符号整数的有效值。
            '''
            max_value = 2**31 - 1
            min_value = -2**31

            if value > max_value:
                return value - 2 * (max_value + 1)
            elif value < min_value:
                return value + 2 * (max_value + 1)
            else:
                return value
        def convert_to_valid_16bit(value):
            max_value = 2**15 - 1
            min_value = -2**15

            if value > max_value:
                return value - 2 * (max_value + 1)
            elif value < min_value:
                return value + 2 * (max_value + 1)
            else:
                return value
        def round_with_sign(value):
            # 判断正负号
            sign = 1 if value >= 0 else -1
            # 对绝对值进行四舍五入
            rounded_value = round(abs(value))
            # 乘回原来的符号
            result = rounded_value * sign
            return result
        txlist = [self.__motor_id, 0x03, 0x00, 0xE0, 0x00, 0x0A]
        self.motor_list =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        self.limit_list = self.ReadMotorLimitData()
        # self.limit_list = [False,False]
        # print("+++++",self.motor_list)
        # print(self.limit_list)

        if(self.motor_list is not None and self.limit_list is not None):
            if(len(self.motor_list)==20 and len(self.limit_list)==2):
                self.__motor_msgs = self.motor_msg()
                backPos = (int)((self.motor_list[-4]<<24) + (self.motor_list[-3]<<16) + (self.motor_list[-2]<<8) + (self.motor_list[-1]))
                self.__motor_msgs.back_pos = convert_to_valid_32bit(backPos)
                self.__motor_msgs.back_lift_height = round((self.__motor_msgs.back_pos-self.__deal_data.GetPortJsonConfig()["posOffset"]) / self.reductionRatio)
                target_pos = (self.motor_list[-8]<<24) + (self.motor_list[-7]<<16) + (self.motor_list[-6]<<8) + (self.motor_list[-5])
                self.__motor_msgs.target_pos = convert_to_valid_32bit(target_pos)
                self.__motor_msgs.location_complete = self.motor_list[11]
                back_speed = (self.motor_list[8]<<8) + (self.motor_list[9])
                self.__motor_msgs.back_speed = convert_to_valid_16bit(back_speed)
                self.__motor_msgs.state_bit = self.motor_list[7]
                self.__motor_msgs.back_current = ((self.motor_list[4]<<8) + (self.motor_list[5])) / 100
                self.__motor_msgs.back_vol = self.motor_list[3]
                self.__motor_msgs.ctrl_mode = self.motor_list[2]
                self.__motor_msgs.upLimit = not (bool)(self.limit_list[1])
                self.__motor_msgs.downLimit = not (bool)(self.limit_list[0])
                return self.__motor_msgs
        # else: print("lift 接收消息数据有问题")
        return False
    
    def ReadMotorLimitData(self):
        '''
        读取限位开关消息，0位是上限位，1位是下限位
        '''
        # 01 03 00 58 00 01 05 D9 #读限位
        txlist = [self.__motor_id, 0x03, 0x00, 0x58, 0x00, 0x01]
        rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        return rxdata
    
    def SetTargetHeight(self, height:int):
        '''
        设定目标高度值，循环限幅
        '''
        if(height > self.__upLimitVal): self._target_height = self.__upLimitVal
        elif(height < self.__downLimitVal): self._target_height = self.__downLimitVal
        else: self._target_height = height
        return self._target_height
    
    def GetTargetHeight(self):
        return self._target_height
    
    def GetTargetPos(self):
        return self._target_pos
    
    def LiftMovePos(self, height:int):
        '''
        运动升降柱指定高度
        '''
        # 0x01, 0x10, 0x00, 0x50, 0x00, 0x02, 0x04, 0xFF, 0xFF, 0xD8, 0xF0 #-10000
        # 0x01, 0x10, 0x00, 0x50, 0x00, 0x02, 0x04, 0x00, 0x00, 0x27, 0x10 #+10000
        height = self.SetTargetHeight(height)
        # 电机转10937.5脉冲，升降柱末端升高1mm
        pause = round(self.reductionRatio * height) + self.__deal_data.GetPortJsonConfig()["posOffset"]
        self._target_pos = pause
        self.MotorMovePos(self._target_pos)
        if isinstance(self.__motor_msgs, self.motor_msg):
            if(abs(self.__motor_msgs.back_lift_height - self._target_height)<5):
                return True
        else: return False

    def LiftLimitInit(self, motor_speed:int, motor_state:motor_msg):
        '''
        运动升降柱指定速度来用限位来初始化位置
        '''
        if not isinstance(motor_state, C_LiftingMotorCtrl.motor_msg):
            return False
        limit:bool
        # motor_speed = -abs(motor_speed)
        # limit = motor_state.downLimit
        motor_speed = motor_speed
        if(motor_speed < 0):
            limit = motor_state.downLimit
        elif(motor_speed > 0):
            limit = motor_state.upLimit
        else:
            print("发送的初始化速度不应为0!!!")
            return False
        if( not limit): 
            self.MotorMoveSpd(motor_speed)
            return False
        else:
            for i in range(3):
                self.MotorMoveSpd(0)
                self.MotorClearCircle()
                self.MotorClearPosZero()
            self.__motor_msgs = self.ReadMotorData()
            if isinstance(self.__motor_msgs, self.motor_msg):
                self._target_pos = self.__motor_msgs.back_pos
                self._target_height = self.__motor_msgs.back_lift_height
            return True
    
    def LiftTimeoutInit(self, timeout_flag:bool, motor_state:motor_msg):
        if not isinstance(motor_state, C_LiftingMotorCtrl.motor_msg):
            return False
        if(not timeout_flag):
            return False
        print("开始 ",self.__motor_id," 号电机超时初始化...")
        for i in range(3):
            self.MotorClearBug()
            self.MotorMoveSpd(0)
            self.MotorClearCircle()
            self.MotorClearPosZero()
        self.__motor_msgs = self.ReadMotorData()
        if isinstance(self.__motor_msgs, self.motor_msg):
            self._target_pos = self.__motor_msgs.back_pos
            self._target_height = self.__motor_msgs.back_lift_height
        return True
        
    def MotorMovePos(self, pause:int):
        '''
        运动电机
        '''
        # 0x01, 0x10, 0x00, 0x50, 0x00, 0x02, 0x04, 0xFF, 0xFF, 0xD8, 0xF0 #-10000
        # 0x01, 0x10, 0x00, 0x50, 0x00, 0x02, 0x04, 0x00, 0x00, 0x27, 0x10 #+10000
        target_pos = []
        target_pos.append(((pause&0xFF000000)>>24)&0xFF)
        target_pos.append(((pause&0xFF0000)>>16)&0xFF)
        target_pos.append(((pause&0xFF00)>>8)&0xFF)
        target_pos.append(((pause&0xFF)&0xFF))
        # crc计算拼接并发送
        txlist = [self.__motor_id, 0x010, 0x00, 0x50, 0x00, 0x02, 0x04] + target_pos
        if(not self.GetStopFlag()):
            # self.MotorClearBug()
            rxdata =  self.__deal_data.DealAllData(txlist, 11, True, self.__port_name)
        pass

    def SetStopFlag(self, flag:bool):
        self.stop_flag = flag

    def GetStopFlag(self):
        return self.stop_flag

    def MotorStop(self, direction, motor_state:motor_msg, force:bool=False):
        '''
        急停电机
        '''
        if not isinstance(motor_state, C_LiftingMotorCtrl.motor_msg):
            return False
        # 01 06 00 4D 00 00 19 DD #急停
        txlist = [self.__motor_id, 0x06, 0x00, 0x4D, 0x00, 0x00]
        # 如果强制停止标志位为True，强制停止电机
        if(force):
            self.SetStopFlag(True)
            self.MotorMoveSpd(0)
            rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        else:
            self.SetStopFlag(False)
            # 如果电机方向为正向(向上走)，同时下限位被触发，那么不停止电机
            if(direction==1):
                if isinstance(motor_state, C_LiftingMotorCtrl.motor_msg):
                    if(motor_state.downLimit):
                        self.SetStopFlag(False)
                    elif((motor_state.upLimit)):
                        self.SetStopFlag(True)
                    
                    if(motor_state.back_lift_height >= self.__upLimitVal):
                        self.SetStopFlag(True)
                    else: 
                        self.SetStopFlag(False)
            # 如果电机方向为反向(向下走)，同时上限位被触发，那么不停止电机
            elif(direction==-1 ):
                if isinstance(motor_state, C_LiftingMotorCtrl.motor_msg):
                    if(motor_state.upLimit):
                        self.SetStopFlag(False)
                    elif((motor_state.downLimit)):
                        self.SetStopFlag(True)
                    if(motor_state.back_lift_height <= self.__downLimitVal):
                        self.SetStopFlag(True)
                    else: 
                        self.SetStopFlag(False)
            # 如果电机方向为0，也就是发送的目标位置和当前位置相同，也不停止电机
            elif(direction==0):
                if(motor_state.back_lift_height > self.__downLimitVal and motor_state.back_lift_height < self.__upLimitVal):
                    self.SetStopFlag(False)
                else:
                    self.SetStopFlag(True)
            else:
                self.SetStopFlag(True)
            #print("------当前方向：",direction)
        if(self.GetStopFlag()):
            rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
            # print('停止电机运动')
            if isinstance(self.__motor_msgs, self.motor_msg):
                if(self.__motor_msgs.back_speed == 0 and self.__motor_msgs.back_current < 15.0):
                    return True
            else: return False
    
    def MotorMoveSpd(self, motor_speed:int):
        '''
        830abs电机以指定速度移动
        '''
        if(motor_speed > self.motorSpd): motor_speed = self.motorSpd
        elif(motor_speed < -self.motorSpd): motor_speed = -self.motorSpd
        # 停止标志位为true，速度为0
        if(self.stop_flag): motor_speed = 0
        speed = []
        speed.append(((motor_speed&0xFF00)>>8)&0xFF)
        speed.append(((motor_speed&0xFF)&0xFF))
        # print('令电机以',motor_speed,'rpm','运动')
        txlist = [self.__motor_id, 0x06, 0x00, 0x10] + speed
        rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        if isinstance(self.__motor_msgs, self.motor_msg):
            if(abs(self.__motor_msgs.back_speed - motor_speed) < 1): 
                return True
        else: return False

    def LiftMoveSpd(self, lift_speed:int):
        lift_end_spd_max = abs((self.motorSpd * 10000) / (self.reductionRatio * 60))
        if(lift_speed > lift_end_spd_max): lift_speed = lift_end_spd_max
        elif(lift_speed < -lift_end_spd_max): lift_speed = -lift_end_spd_max
        lift_speed = round((lift_speed * self.reductionRatio * 60)/10000)
        self.MotorMoveSpd(lift_speed)
    
    def MotorZeroPos(self):
        '''
        电机回到0位
        '''
        # 01 06 00 53 00 00 79 DB #回0位
        txlist = [self.__motor_id, 0x06, 0x00, 0x53, 0x00, 0x00]
        rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        print("控制 ",self.__motor_id," 号电机回到0位")
    
    def MotorClearPosZero(self):
        '''
        电机位置清零
        '''
        # 01 06 00 4C 00 00 48 1D #多圈清零
        txlist = [self.__motor_id, 0x06, 0x00, 0x4B, 0x00, 0x00]
        rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        print(self.__motor_id, " 号电机进行清零指令，注意这个指令失电失效，每次上电初始化都会执行")
        if isinstance(self.__motor_msgs, self.motor_msg):
            if(self.__motor_msgs.back_pos == 0):
                return True
        else: return False
    
    def MotorClearCircle(self):
        '''
        电机多圈清零
        '''
        # 01 06 00 4C 00 00 48 1D #多圈清零
        txlist = [self.__motor_id, 0x06, 0x00, 0x4C, 0x00, 0x00]
        rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        print(self.__motor_id, " 号电机进行多圈位置软件清零")
        if isinstance(self.__motor_msgs, self.motor_msg):
            if(round(self.__motor_msgs.back_pos / 10000) == 0):
                print(self.__motor_id, " 号电机圈数已经清零,电机当前位置的脉冲数小于10000(10000为电机旋转一圈的脉冲数)")
                return True
        else: return False

    def MotorSetAbsoluteMode(self):
        '''
        电机设置绝对位置模式
        '''
        # 01 06 00 51 00 00 D8 1B #设置绝对模式
        txlist = [self.__motor_id, 0x06, 0x00, 0x51, 0x00, 0x00]
        rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        print("设定 ",self.__motor_id," 号电机为绝对位置模式运动")
        pass

    def MotorSetTime(self, time:int):
        '''
        设定电机启停时间
        '''
        txlist = [self.__motor_id, 0x06, 0x00, 0x09, time, time]
        rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        print("设定 ",self.__motor_id," 号电机启动和停止时间为: ",time, "* 65MS")
        pass

    def MotorAutoStopStart(self):
        '''
        开启通讯中断自动停机
        '''
        # 01 06 00 1C 00 07 09 CE #开启通讯中断自动停机
        txlist = [self.__motor_id, 0x06, 0x00, 0x1C, 0x00, 0x07]
        rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        print("开启 ",self.__motor_id," 号电机通讯中断自动停机")

    def MotorAutoStopClose(self):
        '''
        关闭通讯中断自动停机
        '''
        # 01 06 00 1C 00 07 09 CE #开启通讯中断自动停机
        txlist = [self.__motor_id, 0x06, 0x00, 0x1C, 0x00, 0x00]
        rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        print("关闭 ",self.__motor_id," 号电机通讯中断自动停机")
    
    def StartCtrlMotor(self):
        '''
        开启电机
        '''
        txlist = [self.__motor_id, 0x06, 0x00, 0x00, 0x00, 0x01]
        rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        print("开始控制 ",self.__motor_id," 号电机")

    def StopCtrlMotor(self):
        '''
        关闭电机
        '''
        txlist = [self.__motor_id, 0x06, 0x00, 0x00, 0x00, 0x00]
        rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        print("停止控制 ",self.__motor_id," 号电机")

    def MotorSetMaxSpd(self, max_spd):
        '''
        设置电机在位置模式下最大速度
        '''
        # 0x01, 0x10, 0x00, 0x50, 0x00, 0x02, 0x04, 0xFF, 0xFF, 0xD8, 0xF0 #-10000
        # 0x01, 0x10, 0x00, 0x50, 0x00, 0x02, 0x04, 0x00, 0x00, 0x27, 0x10 #+10000
        if(max_spd>self.motorSpd): max_spd = self.motorSpd
        if(max_spd<-self.motorSpd): max_spd = -self.motorSpd
        maxSpd = []
        maxSpd.append(((max_spd&0xFF00)>>8)&0xFF)
        maxSpd.append(((max_spd&0xFF)&0xFF))
        txlist = [self.__motor_id, 0x06, 0x00, 0x1D] + maxSpd
        rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        print("设置 ", self.__motor_id, " 号电机最大速度为: ",max_spd, "RPM")

    def OverflowIProtect(self):
        '''
        电机超过指定电流检测
        '''
        if isinstance(self.__motor_msgs, self.motor_msg):
            if(self.__motor_msgs.back_current > self.__current_limit and self.__motor_msgs.back_speed == 0):
                return True
        return False

    def MotorSetSpdCtrl(self):
        '''
        设定电机为速度控制模式-PC控制
        '''
        txlist = [self.__motor_id, 0x06, 0x00, 0x02, 0x00, 0xC4]
        rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        # print("设定 ",self.__motor_id," 号电机控制模式为速度控制模式")

    def MotorSetPosCtrl(self):
        '''
        设定电机为位置控制模式-PC控制
        '''
        txlist = [self.__motor_id, 0x06, 0x00, 0x02, 0x00, 0xD0]
        rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        # print("设定 ",self.__motor_id," 号电机控制模式为位置控制模式")

    def MotorClearBug(self):
        '''
        清除故障
        '''
        txlist = [self.__motor_id, 0x06, 0x00, 0x4A, 0x00, 0x00]
        rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        print("清除 ",self.__motor_id," 号电机故障")
# -------------------------------- 带电磁抱闸的电机 ----------------------------------------------
    def MotorFreeVol(self):
        txlist = [self.__motor_id, 0x06, 0x00, 0x26, 0x00, 0x55]
        rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        print('电机能耗泄放电压设置为: 85')

    def MotorEnableHold(self):
        txlist = [self.__motor_id, 0x06, 0x00, 0x5E, 0x00, 0x01]
        rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        print('电机抱闸')

    def MotorHold(self):
        txlist = [self.__motor_id, 0x06, 0x00, 0x5F, 0x00, 0x00]
        rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        print('电机抱闸')
    
    def MotorFree(self):
        txlist = [self.__motor_id, 0x06, 0x00, 0x5F, 0x00, 0x01]
        rxdata =  self.__deal_data.DealAllData(txlist, 6, True, self.__port_name)
        print('电机释放抱闸')
# ----------------------------------------------------------------------------------------------

