# 升降柱电机控制功能包

## 830ABS,绝对值

lifting_ctrl_service_node.py 为开启服务控制电机节点`lifting_ctrl_server_node`，服务名为`/LiftingMotorService`

## 850pro,增量式

lifting_ctrl_service_node_850pro.py 为开启服务控制电机节点`lifting_ctrl_850pro_server_node`，服务名为`/LiftingMotorService`

2025-3-6 之前很多客户发现电机只能初始化一次，之后就无效了，是因为速度控制下的限位被驱动器限制了，需要使用电机win上位机，然后设定can通讯组号为1，取消速度模式下限位，就可以恢复正常

## 启动升降柱控制节点
注意，_motor_id参数必需，参数为1,2,3,4中一个，跟升降柱的电机驱动器ID相关，无默认值

```bash
rosrun lifting_ctrl lifting_ctrl_service_node.py _motor_id:=1
rosrun lifting_ctrl lifting_ctrl_service_node_850pro.py _motor_id:=1
```

也可以指定启动的namespace:

```bash
rosrun lifting_ctrl lifting_ctrl_service_node.py _motor_id:=1 __ns:=lifter_1
rosrun lifting_ctrl lifting_ctrl_service_node_850pro.py _motor_id:=1 __ns:=lifter_1
```

也有自带的launch

这是使用默认参数,ns=lifter_1,motor_id=1

```bash
roslaunch lifting_ctrl start_motor.launch
```

这是使用自定义参数

```bash
roslaunch lifting_ctrl start_850pro_motor.launch motor_id_:=1 lifter_ns:=lifter_1
```

## 电机控制json参数文件

```json
{
    "portName": "/dev/lifter_1",//串口名字
    "initMode":0,//开始上电时的模式,0为位置控制,1为上电开始初始化,建议测试好功能后,将电机装到模组上面并测试好下限位正常再去改为1
    "baudRate": 38400 ,//串口波特率
    "dataFormat":"hex",//是否16进制转换
    "reductionRatio":20000,//执行器末端执行1mm，电机运行多少脉冲(1圈10000脉冲)，单位脉冲数
    "posOffset": 0,//位置偏移量(暂时不用)，，单位脉冲
    "responseTimeout":70,//应答超时时间，单位s
    "motorSpd":800,//电机最大速度设定，单位rpm
    "motorTime":0,//电机加减速时间(梯形曲线)，单位 1 为 65ms，假设设定为10,则加减速时间为650ms
    "callBackMode":0,//回调函数模式，0为阻塞应答，1为立刻应答，其它值无效
    "upLimitVal":305,//电机正方向运行最大值，单位mm
    "downLimitVal":0,//电机负方向运行最小值，单位mm
    "initSpd":-800,//初始化时电机运行速度，单位rpm
    "initPos":150,//初始化后执行器要到达的位置，单位mm
    "motorStallCurrent":11,//电机堵转电流限幅,单位A
}
```

## service的数据类型如下

```C
int64 val 
int64 mode
---
int64 state
```

`val` 为要发送的目标值，与mode有关：

- mode为0，为位置控制模式，val值表示要运动的升降柱高度[0,700];
- mode为-4.为速度控制模式，val值表示要控制电机运动的目标速度，其中，速度为正表示升降柱向上，速度为负表示升降柱向下[-1000,1000] (单位：rpm);

`mode` 为控制模式，其中：

- 1为初始化电机，当电机识别到下限位时，结束初始化，如果下限位的线坏了，有过流保护，会直接退出该程序;
  
  注意：运行程序默认初始化，如果下限位出问题，电机转到最底下堵转电流会到11A以上，我写了电流保护，可以在堵转位置初始化为0位置，然后会执行到设定位置(也就是json中initPos这个参数)
- 0为控制电机运动，位置控制模式，控制升降柱指定高度;
- -2为控制电机急停;
- -3为清除错误并初始化电机(只是将电机的一些必须的变量赋值，比如另当前电机位置赋值给目标电机位置等等，实际没有电机操作);
- -4为控制电机恒定速度运动，其中，速度为正表示升降柱向上，速度为负表示升降柱向下,单位是mm/s;
- -5为设定位置模式下电机运动速度，单位rpm，需为正整数，范围为0~[ config里设定的电机最大速度 ];
- -6为清零位置指令
- -7为控制电机以多少RPM运行,需为正整数，范围为0~[ config里设定的电机最大速度 ]
- ...后期还可以添加

初始化：

程序第一次执行时，会读取对应id电机的config的initMode参数,如果为1,即默认初始化，初始化过程中屏蔽其它模式指令，只有初始化成功后，发送其他模式才会生效，如果想要知道是否初始化完成，发送mode值为1,查看反馈即可，1为初始化成功，-1为未完成

## state 为response，应答值

### 回调函数的选择

- SververCallbackImmediately 为立刻应答，即call一次服务端立刻response(后面调试都用的阻塞式回调了，立刻应答没有维护，建议不要改为立刻应答);
- SververCallbackBlock 为阻塞应答，即call一次服务端，直到完成发送的指令才会response。

### res反馈值的含义

- -14 初始化过程中没有将初始化标志位置为True，发送任何指令都会反馈-14，也就是初始化未成功(注意，如果初始化没有成功，之后发送的任何指令都会反馈-14);
- 1 & -1 (初始化成功为前提)
  - 初始化电机模式下(mode=1)，1表示初始化成功，-1表示初始化未完成(也就是初始化标志位被置为True);
  - 位置模式下(mode=0)，1表示到达了目标位置(就是目标位置与当前位置差值小于2mm)，-1表示没有到达(同上反之亦得);
  - 急停模式下(mode=-2)，1表示已经停止(也就是电机反馈速度为0rpm)，-1表示电机还没有停止;
  - 速度模式下(mode=-2)，1表示已经到达设定速度(其实是电机的rpm转化为末端执行器的mm/s速度后，设定速度与反馈速度差值小于5,就返回1,但是因为单位转换了，所以会很快返回res)，-1表示没有到达目标速度;
  - 设定位置模式速度(mode=-5)，1表示已经设定完毕，-1表示没有设定成功
- 0 如果发送的模式不是上述数字，同时初始化成功，反馈值为0;
- 2 & -2 (初始化成功)
  - 当上面的指令反馈为-1,也就是还没有满足指令成功的条件和或者指令执行失败，判断当前电机运行方向和当前指令模式：如果当前触碰了上限位and当前处于0或者-4模式and电机运行方向为负方向，跳过，否则反馈-2，表示触碰了上限位;如果当前触碰了下限位and当前处于0或者-4模式and电机运行方向为正方向，跳过，否则反馈2，表示触碰了下限位;
  - 注意，上下限位并不一定是所谓的上下，只用来表示电机运行方向，电机向负方向运行限制就是下限位，电机正方向运行限制就是上限位;
- -12 & -13 如果执行指令超过设定的responseTimeout应答超时时间，会自动将mode置为-2也就是急停模式，并且反馈-12，如果没有完成初始化还执行指令超时，则反馈-13(这个-13貌似不会反馈>_<);
- -9,如果电机断电同时程序还在运行，fps_error变量会累加，该值大于1时，反馈-9,说明电机离线(只有给service发消息读取反馈才行，非主动)
- -15执行电机位置清零指令,如果执行清零指令的时候电机速度不为0,反馈该值

## topic

这个node文件还会pub topic类型为`/LiftMotorStatePub`的消息，具体如下

```C
uint8 motorId       //电机id
bool initState      //初始化状态

uint8 ctrlMode      //控制模式
uint8 backVol       //反馈电压
float64 backCurrent //反馈电流

uint8 stateBit      //状态位，用来表示电机状态，8位数据每位表示一种状态，下面连续的8个bool类型数据就是这个数值的具体表示
bool runState       //状态位，运行状态，False停机，True启动
bool overflowI      //状态位，过流状态,False则没有,True则触发,下面都是一样的
bool overflowV      //状态位，过压状态
bool encodeErr      //状态位，编码器故障
bool posBiasOver    //状态位，位置偏差过大
bool underVol       //状态位，欠压状态
bool overLoad       //状态位，过载标志
bool externalCtrlMode//状态位，外部控制标志，False-PC控制模式，True-外部PLC控制模式

int16 motorSpeed    //电机反馈速度，rpm
int16 backSpeed     //通过电机速度解算升降柱速度，单位：mm/s
uint8 locationComplete//目标位置，实际是电机内部驱动器的反馈值，不是你发送的
int32 backPos       //反馈位置,电机编码器数值,单位是脉冲数，1圈10000脉冲
int32 backHeight    //反馈升降柱高度，这个是用上面那个反馈位置解算得到(四舍五入),mm
int64 targetHeight  //目标高度，这个是你给service发送height的值(四舍五入),mm
int64 targetPos     //目标位置，这个是目标高度经过解算后得到的目标位置值,mm
bool upLimit        //上限位,0/1
bool downLimit      //下限位,0/1

bool reachTargetPos //是否到达目标位置，这个值是用反馈高度和目标高度相减是否大于1判断得到，大于1就False，小于1就True
int64 fpsError      //帧率错误，即电机离线计数值，发送-3的mode值后会软件清零
```

## 外层code含义

| 代码 |                    含义                  |
|:---:|:------------:|
|13000 - 1| 升降柱控制服务-->发送的指令执行成功|
|13001 - -1| 升降柱控制服务-->发送的指令执行失败|
|13002 - 2| 升降柱控制服务-->下限位触发，优先级比较高，会覆盖前两个|
|13003 - -2| 升降柱控制服务-->上限位触发，同上|
|13004 - -9| 升降柱控制服务-->电机离线，串口收发出现问题，优先级更高，会覆盖其他所有反馈|
|13005| 升降柱控制服务-->开启发布电机msg|
|13006| 升降柱控制服务-->关闭发布电机msg，但是topic名会保留，实际已经取消了发送|
|13007 - -13| 升降柱控制服务-->电机执行指令超时并且初始化失败|
|13008 - -14| 升降柱控制服务-->电机初始化失败|
|13010| 升降柱控制服务-->发送的指令不是设定好的几个指令码|
