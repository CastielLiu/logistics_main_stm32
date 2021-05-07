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

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); //ʱ��ʹ��

    //��ʱ��TIM7��ʼ��
    TIM_TimeBaseStructure.TIM_Period = 9999; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
    TIM_TimeBaseStructure.TIM_Prescaler =71; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    TIM_ClearFlag(TIM7,TIM_FLAG_Update);

    TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM7�ж�,��������ж�

    //�ж����ȼ�NVIC����
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;  //TIM7�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�2��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //�����ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���

    TIM_Cmd(TIM7, ENABLE);  //ʹ��TIMx
}

int Angle,Alpha,RealRPM,TargetTorque;
float autoDriveExpextSpeed = 0.0;
float manualDriveExpectSpeed = 0.0;
float g_vehicleSpeed = 0.0;

double g_seconds = 0.0;          //�Ӷ�ʱ������������ʱ��
uint32_t g_timerCnt10ms = 0;     //10ms������
double g_lastValidCmdTime = 0.0; //��һ�ν�����Чָ���ʱ��,��ָ�ʱ��������ƶ���
	
void TIM7_IRQHandler(void)   //TIM7�ж�
{
	s8 expectSpeedDir = 0;  //�����ʼ��Ϊ0
    if (TIM_GetITStatus(TIM7,TIM_IT_Update)!= RESET)  //���TIM7�����жϷ������
    {
        TIM_ClearITPendingBit(TIM7,TIM_IT_Update);  //���TIMx�����жϱ�־
        g_timerCnt10ms ++;
		g_seconds += 0.01;
    }
	
    /*ÿ10ms����һ�����ݸ���*/
    {
        RealRPM = (YQRL_info.RPM+YQRR_info.RPM)/2;	//���ֻ�ȡת��r/min
		g_vehicleSpeed = (YQRL_info.speed + YQRR_info.speed)/2; // m/s
        Alpha = Angle*12.3/66/65;		//����Ŀ��ת�ǻ����ʵ��ǰ��ת��,��λ(0.1deg)
		TargetTorque = 0;
		
        /*�Զ���ʻ״̬*/
        if(rc.sw2==3)
        {
			//�жϿ���ָ���Ƿ�ʱ���糬��100msδ����ָ��ٶ�����
			if(g_seconds-g_lastValidCmdTime < 0.1)
				autoDriveExpextSpeed = Industry_info.TargetSpeed;
			else
			{
				autoDriveExpextSpeed = 0.0;
			}
			//�ٶ� PID
			if(autoDriveExpextSpeed == 0.0 && fabs(g_vehicleSpeed <= 0.3))
				TargetTorque = IncrementPIDspeedCtrl(&g_speedPID,autoDriveExpextSpeed, g_vehicleSpeed, g_seconds, 1);
			else
				TargetTorque = IncrementPIDspeedCtrl(&g_speedPID,autoDriveExpextSpeed, g_vehicleSpeed, g_seconds, 0);
			
			if(TargetTorque > 255) TargetTorque =255;
			else if(TargetTorque < -255) TargetTorque = -255;
            Angle = Industry_info.TargetAngle;
			
			//Angle = -rc.ch1*65; //�Զ���ʻ״̬��ʹ��ң����ת�ǽ��в���
			
			if(autoDriveExpextSpeed > 0) expectSpeedDir = 1;
			else if(autoDriveExpextSpeed < 0) expectSpeedDir = -1;	
        }
        /*�ֶ���ʻ״̬*/
        else                                                                   
        {
            TargetTorque = rc.ch4*256/660;
            Angle = -rc.ch1*65;
			
			if(TargetTorque > 0) expectSpeedDir = 1;
			else if(TargetTorque < 0) expectSpeedDir = -1;	
        }
    }
    /*ÿ40ms����һ��ǰ��*/
    if(g_timerCnt10ms%4 == 0)
    {
        if(Angle==0)
            DMS055A_SendPosition(1);
        else
            DMS055A_SendPosition(Angle);
    }
//		/*ÿ45ms��ȡһ��ת��������λ��*/
//		if(flag_1ms%=45)
//		{
//			DMS055A_ReadCurrent();
//		}

    /*ÿ50ms����һ�������*/
    if(g_timerCnt10ms%5 == 0)
    {	
        YQRL_TRctrl(rc.sw1,Sonic_info.SonicAlarm,TargetTorque,expectSpeedDir);
    }
    /*ÿ50ms����һ���Һ���*/
    if(g_timerCnt10ms%5 == 1)
    {
        YQRR_TRctrl(rc.sw1,Sonic_info.SonicAlarm,TargetTorque,expectSpeedDir);
    }
//    /*ÿ70ms����һ�γ�����*/
//    if(flag_1ms%=70)
//    {
//        Sonic_SendHostCmd();
//        Sonic_SendClientCmd();
//    }
    /*ÿ50ms����ָ�����������������λ��*/
    if(g_timerCnt10ms%5 == 2)
    {
        JY01_Ctrl(rc.sw2,rc.sw1,Sonic_info.SonicAlarm,TargetTorque,RealRPM,Alpha); //����ǰ���ƶ�ָ��
        Industry_Reply(YQRL_info.RPM,YQRR_info.RPM,Alpha,TargetTorque); //�򹤿��ϱ�����״̬
    }
    /*ÿ80ms����ָ�����λ�����ӣ�*/
    if(g_timerCnt10ms%8 == 0)
    {
        Client_Ctrl(rc.sw2,rc.sw1,Sonic_info.SonicAlarm,RealRPM,Alpha);
    }
	
	/*ÿ100ms���һ�¿���ָ���Ƿ�ʱ(�ϴ���Чָ���Ƿ��㹻��)*/
	if(g_timerCnt10ms%10 ==0 && g_seconds-g_lastValidCmdTime >0.1)
	{
		
	}
}

