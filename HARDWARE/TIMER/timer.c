#include "timer.h"
#include "led.h"
#include "key.h"
#include "adc.h"
#include "remoter.h"
#include "dms055a.h"
#include "can.h"
#include "sonic.h"
#include "delay.h"
#include "yq.h"
#include "industry.h"
#include "client.h"
#include "jy01.h"
#include "pid.h"
#include "math.h"

// T = (72MHZ/(71+1))*(9999+1) = 100hz => 10ms
void TIM7_Init(void)
{
    TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
    NVIC_InitTypeDef					NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); //时钟使能

    //定时器TIM7初始化
    TIM_TimeBaseStructure.TIM_Period = 9999; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler =71; //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位

    TIM_ClearFlag(TIM7,TIM_FLAG_Update);

    TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE ); //使能指定的TIM7中断,允许更新中断

    //中断优先级NVIC设置
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;  //TIM7中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级2级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //从优先级0级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器

    TIM_Cmd(TIM7, ENABLE);  //使能TIMx
}

int Angle,Alpha,RealRPM,TargetTorque;
float autoDriveExpextSpeed = 0.0;
float manualDriveExpectSpeed = 0.0;
float g_vehicleSpeed = 0.0;

double g_seconds = 0.0;          //从定时器开启经过的时间
uint32_t g_timerCnt10ms = 0;     //10ms计数器
double g_lastValidCmdTime = 0.0; //上一次接收有效指令的时间,当指令超时后需紧急制动！
	
void TIM7_IRQHandler(void)   //TIM7中断
{
	s8 expectSpeedDir = 0;  //必须初始化为0
    if (TIM_GetITStatus(TIM7,TIM_IT_Update)!= RESET)  //检查TIM7更新中断发生与否
    {
        TIM_ClearITPendingBit(TIM7,TIM_IT_Update);  //清除TIMx更新中断标志
        g_timerCnt10ms ++;
		g_seconds += 0.01;
    }
	
    /*每10ms进行一次数据更新*/
    {
        RealRPM = (YQRL_info.RPM+YQRR_info.RPM)/2;	//后轮获取转速r/min
		g_vehicleSpeed = (YQRL_info.speed + YQRR_info.speed)/2; // m/s
        Alpha = Angle*12.3/66/65;		//根据目标转角换算成实际前轮转角,单位(0.1deg)
		TargetTorque = 0;
		
        /*自动驾驶状态*/
        if(rc.sw2==3)
        {
			//判断控制指令是否超时，如超过100ms未更新指令，速度置零
			if(g_seconds-g_lastValidCmdTime < 0.1)
				autoDriveExpextSpeed = Industry_info.TargetSpeed;
			else
			{
				autoDriveExpextSpeed = 0.0;
			}
			//速度 PID
			if(autoDriveExpextSpeed == 0.0 && fabs(g_vehicleSpeed <= 0.3))
				TargetTorque = IncrementPIDspeedCtrl(&g_speedPID,autoDriveExpextSpeed, g_vehicleSpeed, g_seconds, 1);
			else
				TargetTorque = IncrementPIDspeedCtrl(&g_speedPID,autoDriveExpextSpeed, g_vehicleSpeed, g_seconds, 0);
			
			if(TargetTorque > 255) TargetTorque =255;
			else if(TargetTorque < -255) TargetTorque = -255;
            Angle = Industry_info.TargetAngle;
			
			//Angle = -rc.ch1*65; //自动驾驶状态下使用遥控器转角进行测试
			
			if(autoDriveExpextSpeed > 0) expectSpeedDir = 1;
			else if(autoDriveExpextSpeed < 0) expectSpeedDir = -1;	
        }
        /*手动驾驶状态*/
        else                                                                   
        {
            TargetTorque = rc.ch4*256/660;
            Angle = -rc.ch1*65;
			
			if(TargetTorque > 0) expectSpeedDir = 1;
			else if(TargetTorque < 0) expectSpeedDir = -1;	
        }
    }
    /*每40ms控制一次前轮*/
    if(g_timerCnt10ms%4 == 0)
    {
        if(Angle==0)
            DMS055A_SendPosition(1);
        else
            DMS055A_SendPosition(Angle);
    }
//		/*每45ms读取一次转向电机绝对位置*/
//		if(flag_1ms%=45)
//		{
//			DMS055A_ReadCurrent();
//		}

    /*每50ms控制一次左后轮*/
    if(g_timerCnt10ms%5 == 0)
    {	
        YQRL_TRctrl(rc.sw1,Sonic_info.SonicAlarm,TargetTorque,expectSpeedDir);
    }
    /*每50ms控制一次右后轮*/
    if(g_timerCnt10ms%5 == 1)
    {
        YQRR_TRctrl(rc.sw1,Sonic_info.SonicAlarm,TargetTorque,expectSpeedDir);
    }
//    /*每70ms控制一次超声波*/
//    if(flag_1ms%=70)
//    {
//        Sonic_SendHostCmd();
//        Sonic_SendClientCmd();
//    }
    /*每50ms发送指令给居逸驱动器和上位机*/
    if(g_timerCnt10ms%5 == 2)
    {
        JY01_Ctrl(rc.sw2,rc.sw1,Sonic_info.SonicAlarm,TargetTorque,RealRPM,Alpha); //发送前轮制动指令
        Industry_Reply(YQRL_info.RPM,YQRR_info.RPM,Alpha,TargetTorque); //向工控上报车辆状态
    }
    /*每80ms发送指令给下位机（从）*/
    if(g_timerCnt10ms%8 == 0)
    {
        Client_Ctrl(rc.sw2,rc.sw1,Sonic_info.SonicAlarm,RealRPM,Alpha);
    }
	
	/*每100ms检查一下控制指令是否超时(上次有效指令是否足够新)*/
	if(g_timerCnt10ms%10 ==0 && g_seconds-g_lastValidCmdTime >0.1)
	{
		
	}
}

