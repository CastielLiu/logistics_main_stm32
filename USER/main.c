#include "sys.h"
#include "delay.h"
#include "led.h"
#include "key.h"
#include "remoter.h"
#include "dms055a.h"
#include "timer.h"
#include "can.h"
#include "yq.h"
#include "industry.h"
#include "common.h"
#include "wdg.h"
#include "sonic.h"
#include "dma.h"
#include "adc.h"

int main(void)
{	
	int delay_cnt = 50;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	//初始化CAN接口，与JY01通信，控制后轮  //CAN_Mode_LoopBack  CAN_Mode_Normal
    CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_Normal);	
    delay_init();	    		//延时函数初始化 
	
    LED_Init();
    Remoter_Uart_Init(100000);	//初始化遥控接收串口
    Industry_Init(115200);		//工控机通信串口
    DMS055A_Init(19200);		//初始化转向驱动串口
	Sonic_Init(115200);         //初始化超声波485
	
	init_pid_params(&g_speedPID, 50.0, 0.3, 1.0);  //初始化速度PID参数
    TIM7_Init();
	Adc_Init();
	
//	IWDG_Init(4,625);    //预分频系数64,重装载值625,溢出时间1s	
//	Beep_Alarm(3,200);
	
    while(1)
    {
        MCU_Light = !MCU_Light;
		g_roadwheelAngle = getRoadWheelAngle(); //循环获取前轮转角角度值
		
		if(!g_angleSensorInited)
		{
			if(--delay_cnt == 0)
			{
				g_firstRoadwheelAngle = g_roadwheelAngle;
				g_angleSensorInited = 1;
			}
		}
				
//		IWDG_Feed();
    }
}
