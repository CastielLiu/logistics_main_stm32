该程序为AR_SmartChassis_LowerHost_v0.0.3_ZLun版硬件配套测试程序
该电路板功能：
1.通过串口接收遥控命令；
2.通过串口控制转向电机驱动板；

20201214
1.接收遥控命令测试，，，，

20201215
1.闪灯测试，正常；
2.串口1测试，正常；
3.串口5(接收遥控命令)测试，正常；
4.ADC测试，备用端口（PC1,PC2,PC3）测试，正常；
5.串口4（给转向驱动器发送命令）,测试正常；
6.串口3（向后轮驱动器读取参数），测试，正常；
7.串口3（向后轮驱动器发送控制指令），测试，正常；

20201221
1.Switch,EN端口测试，正常；
2.蜂鸣器测试，正常；
3.工控机通信，测试，正常；

20210108
1.一拖八超声波探测器模块，CRC测试，正常；
2.一拖八超声波探测器模块，单个模块通信测试，正常；

20210111
1.一拖八超声波探测器模块，两个模块通信测试，正常；

20210222
1.转向电机驱动器测试，上电后，回中无误差（或者回中误差很小，可以忽略）；


20210223
1.修改定时器中断处理函数，整合数据，转向测试，正常；
2.修改下位机主从之间的通讯协议，测试，正常；
3.修改下位机（主）与居逸驱动器之间的通讯协议，测试,正常；

20210226
1.修改上位机下位机通讯协议，测试，正常；
2.

20210331
1.修改上位机与下位机通讯协议，上传左右后轮实际转速；

20210421
1. 修改了定时器溢出频率，由1ms改为10ms，并新增全局时间(s)变量，经测试准确
2. 修改轮速接收函数，将输入值直接÷5得到真实转速(prm)，新增速度(m/s)成员
3. 新增车速增量型PID控制控制，输入为期望速度(m/s)，输出为期望扭矩值(0-255)
    添加PID参数串口配置接口(USART2, 工控机)，数据包ID为0x05
4. 新增自动驾驶指令超时保护，超过100ms未收到新的控制指令，期望速度置零

20210518
1. 新增独立看门狗 1s溢出

20210519
1.增加串口1超声波主动上传数据解析；
2.增加超声波决策函数；


20210526
1.减小前轮转角至10.25deg，降低前轮转向力度，防止过流；

20210602
1.增加超声波安全距离随速度变化；