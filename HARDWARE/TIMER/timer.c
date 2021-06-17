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

float Angle,Alpha,RealRPM,TargetTorque;
float expectSpeed = 0.0;

const float munualCtrlMaxSpeed = 2.5; //m/s
float g_vehicleSpeed = 0.0;           //m/s

double g_seconds = 0.0;          //从定时器开启经过的时间
uint32_t g_timerCnt10ms = 0;     //10ms计数器
double g_lastValidCmdTime = 0.0; //上一次接收有效指令的时间,当指令超时后需紧急制动！

s8 expectSpeedDir = 0;  //必须初始化为0
u16 expectBrakeVal = 0;  //必须初始化为0
u8  sonicDecisionMakingResult = 0;
	
void TIM7_IRQHandler(void)   //TIM7中断
{
	expectSpeedDir = 0;
	expectBrakeVal = 0;
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
        Alpha = Angle*10.25/66/65;		//根据目标转角换算成实际前轮转角,单位(0.1deg)
		TargetTorque = 0;
		
        if(rc.sw2==3 ||  //右侧拨码中位 /*自动驾驶状态  PID速度控制*/
		   rc.sw2==2)    //右侧拨码下位 /*手动驾驶状态  PID速度控制*/
        {
			if(rc.sw2==3) //自动
			{
				//判断控制指令是否超时，如超过100ms未更新指令，速度置零
				if(g_seconds-g_lastValidCmdTime < 0.1)
					expectSpeed = Industry_info.TargetSpeed;
				else
				{
					expectSpeed = 0.0;
				}
				targetAngleTransform(Industry_info);//目标指令角度转换为适应此驱动器的指令
				Angle = Industry_info.TargetAngle;
				//Angle = -rc.ch1*65; //自动驾驶状态下使用遥控器转角进行测试
			}
			else  //手动
			{
				expectSpeed = 1.0 * rc.ch4/660 * munualCtrlMaxSpeed;
				Angle = -rc.ch1*50;	//rc.ch1 [-660, 660] Angle [-660*50, 660*50]
			}
			
			//速度 PID
			if(expectSpeed == 0.0 && fabs(g_vehicleSpeed <= 0.3))
				TargetTorque = IncrementPIDspeedCtrl(&g_speedPID,expectSpeed, g_vehicleSpeed, g_seconds, 1);
			else
				TargetTorque = IncrementPIDspeedCtrl(&g_speedPID,expectSpeed, g_vehicleSpeed, g_seconds, 0);
			
			if(TargetTorque > 255) TargetTorque =255;
			else if(TargetTorque < -255) TargetTorque = -255;
			
			if(expectSpeed > 0) expectSpeedDir = 1;
			else if(expectSpeed < 0) expectSpeedDir = -1;	
			
			if(expectSpeedDir * TargetTorque < 0)  // 期望速度方向与期望扭矩方向不一致时，表明需要进行制动操作
				expectBrakeVal = abs(TargetTorque);
			else if(expectSpeedDir == 0)		   // 当期望速度为0时，无论如何都需要制动(全扭矩制动)
				expectBrakeVal = 255;
			

        }
        /*手动驾驶状态  扭矩控制*/
        else if(rc.sw2==1) //右侧拨码上位                                                            
        {
            TargetTorque = rc.ch4*255/660;   //rc.ch4 [-660, 660]
            Angle = -rc.ch1*50;              //rc.ch1 [-660, 660] Angle [-660*50, 660*50]
			
			if(g_vehicleSpeed > 0 && TargetTorque < 0)
				expectBrakeVal = -TargetTorque;
			else if(g_vehicleSpeed < 0 && TargetTorque > 0)
				expectBrakeVal = TargetTorque;
			
			if(g_vehicleSpeed > 0)
				expectSpeedDir = 1;
			else if(g_vehicleSpeed < 0)
				expectSpeedDir = -1;
			else //g_vehicleSpeed == 0  
			{
				//汽车速度为0时，若不经此判断，默认方向为0 驱动器将不执行驱动指令
				if(TargetTorque > 0)
					expectSpeedDir = 1;
				else if(TargetTorque < 0)
					expectSpeedDir = -1;
			}
        }
    }
#if (ENABLE_SONIC == 1)
	if(g_timerCnt10ms%7 == 0) /*每70ms执行一次超声波决策*/
		sonicDecisionMakingResult = SonicDecisionMaking();
	
	//当超声波报警使车辆无法运行时需重置pid参数以防止超声波解除报警后由于误差累计导致的大扭矩
	if(sonicDecisionMakingResult&0x01 && expectSpeedDir==1)//禁止前进且当前期望速度方向为正
	{
		TargetTorque = -255;
		IncrementPIDspeedCtrl(&g_speedPID,expectSpeed, g_vehicleSpeed, g_seconds, 1); //重置pid
	}
	if(sonicDecisionMakingResult&0x02 && expectSpeedDir==-1)//禁止后退且当前期望速度方向为负
	{
		TargetTorque = 255;
		IncrementPIDspeedCtrl(&g_speedPID,expectSpeed, g_vehicleSpeed, g_seconds, 1); //重置pid
	}

#endif

    /*每40ms控制一次前轮*/
    if(g_timerCnt10ms%4 == 0)
    {
        if(Angle==0)
            DMS055A_SendPosition(1);
        else
            DMS055A_SendPosition(Angle);
				// + g_sensorAngle
    }
//		/*每45ms读取一次转向电机绝对位置*/
//		if(flag_1ms%=45)
//		{
//			DMS055A_ReadCurrent();
//		}

    if(g_timerCnt10ms%5 == 0)/*每50ms控制一次左后轮*/
        YQRL_TRctrl(rc.sw1,Sonic_info.SonicAlarm,TargetTorque,expectSpeedDir);
 
    if(g_timerCnt10ms%5 == 1) /*每50ms控制一次右后轮*/
        YQRR_TRctrl(rc.sw1,Sonic_info.SonicAlarm,TargetTorque,expectSpeedDir);
	
    /*每50ms发送指令给居逸驱动器和上位机*/
    if(g_timerCnt10ms%5 == 3)
    {
		expectBrakeVal *= 2;     //放大制动力
		if(expectBrakeVal > 255)  
			expectBrakeVal = 255;  //制动力限幅
		else if(expectBrakeVal < 80 && expectBrakeVal > 0) 
			expectBrakeVal = 80;
		
        JY01_Ctrl(rc.sw2,rc.sw1,Sonic_info.SonicAlarm,expectBrakeVal,RealRPM,Alpha); //发送前轮制动指令
        Industry_Reply(YQRL_info.RPM,YQRR_info.RPM,Alpha,TargetTorque, expectBrakeVal); //向工控上报车辆状态
    }
    
    if(g_timerCnt10ms%8 == 0) /*每80ms发送指令给下位机（从）*/
        Client_Ctrl(rc.sw2,rc.sw1,sonicDecisionMakingResult,RealRPM,Alpha);
 
	
}

