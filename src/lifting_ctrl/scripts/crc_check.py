#!/usr/bin/env python3 
#codeing =utf-8 
# Author: RosenYin 2356838399@qq.com
# Date: 2024-01-05 09:42:24
# LastEditors: RosenYin 2356838399@qq.com
# LastEditTime: 2024-01-19 11:36:42
# FilePath: /ROS_workspace/src/BehaviorTree/lifting_ctrl/scripts/crc_check.py
# Description: 
#   CRC通用公式
#   计算链接：http://www.ip33.com/crc.html
# Copyright (c) 2024 by RosenYin, All Rights Reserved. 
#/ 

class C_CRC:
    # 8位数据按位翻转
    # 如：0x03(0000 0011) -> 0xC0(1100 0000)
    #       3            ->  192
    def _invert_uint8(self, data):
        result = 0
        # 初始化 或数据，这个数值会随着原始数值位移而位移
        # 1000 0000
        a = 0x80
        for i in range(8):
            # 如果低位为1，则按位与后的数值不为0，进入判断，为0就不进入
            if(data & 1):
                # 将结果值与或数据进行按位或
                result = result | a
            # 位移原始数据，以进行下一次循环
            data = data >> 1
            # 位移 或数据
            a = a >> 1
        # 将结果值限制在8bit
        result = result & 0xFF
        return result
    # 16位数据按位翻转
    def _invert_uint16(self, data):
        result = 0
        a = 0x8000
        for i in range(16):
            if(data & 1):
                result = result | a
            data = data >> 1
            a = a >> 1
        # 将结果值限制在16bit
        result = result & 0xFFFF
        return result
    # 32位数据按位翻转，返回翻转后的数值
    def _invert_uint32(self, data):
        result = 0
        a = 0x80000000
        for i in range(32):
            if(data & 1):
                result = result | a
            data = data >> 1
            a = a >> 1
        # 将结果值限制在32bit
        result = result & 0xFFFFFFFF
        return result
    # 计算8位crc值，返回的crc值高位在左低位在右，使用时请注意高低位顺序
    def CalCRC8(self, list, size, crc_poly, init_value, ref_flag, xorout):
        if(len(list) < size or size < 0):
            raise IndexError("size值设定有误")
        # crc结果中间值
        crc_reg = init_value
        poly = crc_poly
        tmp, num = 0, 0
        # 循环自定义列表数据长度
        while (size):
            size = size - 1
            # 赋值原始数据
            byte = list[num]
            num = num +1
            # 如果输入数据反转flag为1
            if (ref_flag):
                # 将原始数据按位翻转
                byte = self._invert_uint8(byte)
            # 如果是第一次循环，将第0位的数据与初值进行抑或
            # 如果不是第一次循环，将上一次数据按位异或后的结果 与 当前位的原始数据进行异或
            crc_reg ^= byte
            # 因为列表一个元素是8bit，所以按位异或8次
            for i in range(8):
                # tmp作为判断条件，判断当前的crc结果中间值最高位(第8位)是否是1
                tmp = crc_reg & 0x80
                # 位移操作，每次for循环位移1位，总共循环8次刚好到最高位
                crc_reg <<= 1
                # 如果前的crc结果中间值最高位(第8位)是1，将crc结果中间值与多项式poly异或
                if (tmp != 0):
                    crc_reg ^= poly
        # 如果输入数据反转flag为1，进行结果翻转
        if (ref_flag):
            crc_reg = self._invert_uint8(crc_reg)
        # 输出数据反转，不需要判断，一般值为0x00或者0xFF，如果为0那么就是原值
        crc_reg = crc_reg ^ xorout
        # 去掉高位垃圾值，令最终结果限制在8bit，最终结果是与网页一样的
        crc_reg = crc_reg & 0xFF
        
        return crc_reg
    # 计算16位crc值，返回的crc值高位在左低位在右，使用时请注意高低位顺序
    def CalCRC16(self, list:list, size:int, crc_poly:hex, init_value:hex, ref_flag:bool, xorout:hex):
        if(len(list) < size or size < 0):
            raise IndexError("size值设定有误")
        crc_reg = init_value
        poly = crc_poly
        size_ = size
        tmp, num = 0, 0
        while (size_):
            size_ = size_ - 1
            byte = list[num]
            num = num +1
            if (ref_flag):
                byte = self._invert_uint8(byte)
            # 如果是第一次循环，将第0位的数据与初值进行抑或
            # 如果不是第一次循环，将上一次数据按位异或后的结果 与 当前位的原始数据进行异或
            # 为什么要向左位移：因为一般原始数据是8位的字节数据，是存放在低8位的，需要手动位移到高8位，即16-8=8
            crc_reg ^= byte << 8
            for j in range(8):
                tmp = crc_reg & 0x8000
                crc_reg <<= 1
                if (tmp):
                    crc_reg ^= poly
        if (ref_flag):
            crc_reg = self._invert_uint16(crc_reg)
        crc_reg = crc_reg ^ xorout
        # 限定输出位为16bit
        crc_reg = crc_reg & 0xFFFF

        return crc_reg
    # 计算32位crc值，返回的crc值高位在左低位在右，使用时请注意高低位顺序
    def CalCRC32(self, list, size, crc_poly, init_value, ref_flag, xorout):
        if(len(list) < size or size < 0):
            raise IndexError("size值设定有误")
        crc_reg = init_value
        poly = crc_poly
        tmp, num = 0, 0
        while (size):
            size = size - 1
            byte = list[num]
            num = num +1
            if (ref_flag):
                byte = self._invert_uint8(byte)
            # 如果是第一次循环，将第0位的数据与初值进行抑或
            # 如果不是第一次循环，将上一次数据按位异或后的结果 与 当前位的原始数据进行异或
            # 为什么要向左位移：因为一般原始数据是8位的字节数据，是存放在低8位的，需要手动位移到高8位，即32-8=24
            crc_reg ^= byte << 24
            for j in range(8):
                tmp = crc_reg & 0x80000000
                crc_reg <<= 1
                if (tmp):
                    crc_reg ^= poly
        if (ref_flag):
            crc_reg = self._invert_uint32(crc_reg)
        crc_reg = crc_reg ^ xorout
        # 限定输出位为32bit
        crc_reg = crc_reg & 0xFFFFFFFF

        return crc_reg
    # 将crc结果进行处理，crc_bit用来指示crc的位数如8，16，32，return_type用来决定返回类型:"int"就是返回数字，"str"是返回字符串；
    # crc_result为使用上面crc计算函数的返回值作为crc结果
    def CrcSplit(self, crc_bit:int, return_type: 'str', crc_result:int):
        '''
        分割1位crc校验值，低位在[0]位
        '''
        # 如果 crc_result 不为空并且是int类型
        if(crc_result is not None and isinstance(crc_result, int)):
            # 如果返回类型为int
            if(return_type == "int"):
                # 如果crc位数为8
                if(crc_bit == 8):
                    return crc_result
                # 如果crc位数为16
                elif(crc_bit == 16):
                    tmp = crc_result
                    crc16 = []
                    # 低位在[0]位
                    crc16.append(tmp & 0xFF) # 低位
                    # 高位在[1]位
                    crc16.append((tmp & 0xFF00) >> 8) # 高位
                    return crc16
                # 如果crc位数为32
                elif(crc_bit == 32):
                    tmp = crc_result
                    crc32 = []
                    # 低位在[0]位
                    crc32.append((tmp & 0xFF))
                    crc32.append((tmp & 0xFF00) >> 8)
                    crc32.append((tmp & 0xFF00) >> 8)
                    crc32.append((tmp & 0xFF0000) >> 16)
                    crc32.append((tmp & 0xFF000000) >> 24)
                    return crc32
            elif(return_type == "str"):
                # 将int类型的 crc_result 转化为字符串，以hex格式显示
                return hex(int(crc_result))
            else:
                raise TypeError("return_type 不为'int'或'str'")
        else:
            raise TypeError("crc_result 为空或者不为int类型")


# 测试函数
def CrcTest():
    crc = C_CRC()
    # 0x01, 0x10, 0x00, 0x50, 0x00, 0x02, 0x04, 0xFF, 0xFF, 0xD8, 0xF0 #-10000
    # 0x01, 0x10, 0x00, 0x50, 0x00, 0x02, 0x04, 0x00, 0x00, 0x27, 0x10 #+10000
    # 01 03 00 E8 00 02 44 3F #读位置
    # 02 03 00 E8 00 02 44 0C #读位置
    # 01 06 00 51 00 00 D8 1B #设置绝对模式
    # 01 06 00 51 00 01 19 DB #设置相对模式
    # 01 06 00 4C 00 00 48 1D #多圈清零
    # 01 06 00 4C 00 00 48 1D #单纯清零
    # 01 06 00 53 00 00 79 DB #回0位
    # 01 06 00 4D 00 00 19 DD #急停
    # 01 06 00 1C 00 07 09 CE #开启通讯中断自动停机
    # 01 03 00 E0 00 0A C4 3B #读取所有数据
    # 01 06 00 4F 00 00 B8 1D #缓冲急停
    # 01 06 00 4A 00 00 A8 1C #清除故障
    # 01 06 00 09 xx xx xx xx #设置启停时间
    # 01 03 00 58 00 01 05 D9 #读限位
    # 02 03 00 E3 00 01 75 CF #状态
    data1 = [0x01, 0x06, 0x00, 0x02, 0x00, 0xC4]
    a = crc.CalCRC16(data1, 6, 0x8005, 0xFFFF, 1, 0x0000)
    # a = crc.CalCRC8(data1, 1, 0x31, 0x00, 1, 0x00)
    # a = crc.CalCRC32(data1, 1, 0x04C11DB7, 0xFFFFFFFF, 0, 0x00000000)
    # 将10进制转换成16进制
    a=hex(int(a))
    # [2:]的作用是将4位16进制的0x消除
    # .upper()可以让字母变成大写，只是为了格式好看而已，并不影响校验结果
    a=a[2:].upper()
    print(a)

    length = len(a)
    # [0:length]是将得到的4位16进制切片成两个校验码而已
    # 一些结果以0开头，会自动把0给吞掉 .zfill(2)可以让结果以两位二进制的形式出现
    high=a[0:length-2].zfill(2)
    high=str(high)
    
    low=a[length-2:length].zfill(2)
    low=str(low)
    # print(type(low))
    print("校验码低位："+low.upper())
    print("校验码高位："+high.upper())

if __name__ == '__main__':

    CrcTest()