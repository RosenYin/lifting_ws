# 松灵750w&400w电机控制功能包

Time: 2024.6.24

有关`rospackage`细节详见:

[lifting_ctrl README](./src/lifting_ctrl/README.MD)

有关USB绑定规则详见:

[usb-rules规则编写](./usb-rules规则编写.md)

## 注意事项

1. 驱动器正面会有标签，可以看驱动器型号，目前只支持830ABS、850pro
2. 驱动器有两个网口，走的一个是232协议，一个是485/can协议，
   1. 232那个口是用来插它自带的灰色的网口转232的，然后232一端连接usb转串口线，再插到win系统电脑上，使用win上位机控制
   2. 485用来连接工控机使用ros包控制，接线检查是否正确
3. 若重新拆装了电机,建议将json的`initMode`参数设定为0,然后手动执行初始化指令(val: 0, mode: 1),设定这个参数之后启动程序,电机会发生微动,属正常现象
   因为假设初始上电位置超过了json中设定的限位范围[downLimitVal,upLimitVal],`initMode`参数设定为0会导致刚上电数值超限,会向反方向旋转,这个只在`initMode`参数设定为0时出现,
   如果`initMode`参数设定为1,则会上电初始化,不会出现这个问题

## Dependencies

```bash
cd lifting_ws
pip3 install -r ./requirement.txt
sudo apt install ros-noetic-serial
```

## USB Permission

```bash
sudo cp ttyUSB_permission.rules /etc/udev/rules.d/
```

```shell
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## compile

```bash
cd lifting_ws
catkin_make
```

## run

```bash
# source
source devel/setup.bash
# roscore
roscore
#run node
rosrun lifting_ctrl lifting_ctrl_service_node.py _motor_id:=1 #830abs
rosrun lifting_ctrl lifting_ctrl_service_node_850pro.py _motor_id:=1 #850pro
#topic echo
rostopic echo /LiftMotorStatePub
#service call
rosservice call /LiftingMotorService val: 0 mode: 0
```

推荐使用`byobu`命令行工具多开终端,一般开三个窗格,一个用来启动代码,一个用来`echo topic`,一个用来`service call`

## Json

需要根据移动模组的参数来定,需要修改的如下:

1. 确定丝杆导程,如果有减速箱,确定减速比,

   导程: 丝杆螺杆导程，影响着末端移动1mm，丝杆应该转多少圈

      比如导程为5，也就是丝杆转1圈，末端移动5mm

   减速比: 电机旋转几圈，丝杆转1圈，比如1:10减速比，那么电机转10圈，丝杆转1圈

   例:

   导程=5, 减速比= 1:1

   `reductionRatio`应该设定为:10000 * (1/5) = 2000

   导程=8, 减速比= 1:10

   `reductionRatio`应该设定为:10000 \* (1/8) \* (10) = 12500

2. 需要确定初始化的方向:

   如果是负方向初始化,一般电机倒置安装,丝杆为右旋情况下,也就是设定给`initSpd`为负数,电机向负方向运行是朝下限位运行,则需要将`downLimitVal`设定为0,`upLimitVal`设定为模组下限位到上限位距离(正值),单位为mm

   如果是正方向初始化,一般电机倒置安装,丝杆为右旋情况下,也就是设定给`initSpd`为正数,电机向负方向运行是朝上限位运行,则需要将`downLimitVal`设定为模组下限位到上限位距离(负值),`upLimitVal`设定为0,单位为mm

   正负方向初始化依照`initSpd`参数来确定,该参数为负则为负方向初始化,正数则为正方向初始化
3. 初始化成功后,要运行的位置,单位mm:

   `initPos`电机初始化成功后,如果是依靠限位初始化成功,会识别这个数值,然后作为初始化后运行位置,然后驱动电机运行到这里
4. `motorSpd`电机最大速度设定,须为正数,单位rpm
5. `responseTimeout`是应答超时时间设定参数,call之后,会阻塞在终端,这个数值用来设定阻塞时间,依照模组行程确定,如果行程很长可以适当加大
6. `initMode`用来确定程序刚启动时候的模式,设定1为初始化模式,如果不需要初始化,请设置为0
7. `portName`用来设定电机信号线所在的usb端口,这个参数如果不存在,程序会自动寻找`ttyUSB*`
8. `motorStallCurrent`用来设定电机保护电流，根据实际情况去设定,负载比较大的时候酌情加大
9. `motorTime`用来设定电机加减速时间,在位置模式下,也就是梯形加减速模型

   如果是830ABS,则只用来设定位置模式下加减速时间

   如果是850pro,则同时设定位置模式和速度模式下加减速时间

```json
{
   "portName": "/dev/lifter_1",//串口名字,如果不写,程序会自动寻找
   "initMode":0,//开始上电时的模式,0为位置控制,1为上电开始初始化,建议测试好功能后,将电机装到模组上面并测试好下限位正常再去改为1
   "baudRate": 38400,//串口波特率
   "timeout":0.02,//串口读取一帧数据的超时时间，数值越小读取的数据帧率越快，但是越可能出现丢帧，经测试，830abs为0.02,850pro为0.008最宜
   "dataFormat":"hex",//使用16进制格式化数据
   "reductionRatio":20000,//执行器末端执行1mm，电机运行多少脉冲(1圈10000脉冲)，单位脉冲数
   "responseTimeout":70,//应答超时时间，单位s
   "motorSpd":800,//电机最大速度设定，单位rpm
   //电机加减速时间(梯形曲线)，
   //(830ABS):单位 1 为 65ms，假设设定为10,则加减速时间为650ms
   //(850pro):单位 1 为 1ms，假设设定为10,则加减速时间为10ms,这个同时还会设置加减速时间
   "motorTime":0,
   "callBackMode":1,//0为阻塞式应答，1为非阻塞应答
   "upLimitVal":305,//电机正方向运行最大值，单位mm
   "downLimitVal":0,//电机负方向运行最小值，单位mm
   "initSpd":-800,//初始化时电机运行速度，单位rpm
   "initPos":0,//初始化后执行器要到达的位置，单位mm
   "motorStallCurrent":17,//电机堵转电流限幅,单位A
}
```
