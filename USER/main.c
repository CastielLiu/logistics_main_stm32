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
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	//��ʼ��CAN�ӿڣ���JY01ͨ�ţ����ƺ���  //CAN_Mode_LoopBack  CAN_Mode_Normal
    CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_Normal);	
    delay_init();	    		//��ʱ������ʼ�� 
	
    LED_Init();
    Remoter_Uart_Init(100000);	//��ʼ��ң�ؽ��մ���
    Industry_Init(115200);		//���ػ�ͨ�Ŵ���
    DMS055A_Init(19200);		//��ʼ��ת����������
	Sonic_Init(115200);         //��ʼ��������485
	
	init_pid_params(&g_speedPID, 50.0, 0.3, 1.0);  //��ʼ���ٶ�PID����
    TIM7_Init();
	Adc_Init();
	
//	IWDG_Init(4,625);    //Ԥ��Ƶϵ��64,��װ��ֵ625,���ʱ��1s	
//	Beep_Alarm(3,200);
	
    while(1)
    {
        MCU_Light = !MCU_Light;
		g_roadwheelAngle = getRoadWheelAngle(); //ѭ����ȡǰ��ת�ǽǶ�ֵ
		
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
