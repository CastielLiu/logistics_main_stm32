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

float TargetAngle,RealRPM,TargetTorque;
float expectSpeed = 0.0;

const float munualCtrlMaxSpeed = 2.5; //m/s
float g_vehicleSpeed = 0.0;           //m/s
float g_sensorVoltage = 0.0;
u8    g_angleSensorInited = 0;   //�����ʼ��Ϊ0
float g_roadwheelAngle = 0.0;
float g_firstRoadwheelAngle;     //�ϵ�ʱǰ�ֵ�ת��ֵ
u32 g_currentPulse;			 //���ʵʱ������ֵ

double g_seconds = 0.0;          //�Ӷ�ʱ������������ʱ��
uint32_t g_timerCnt10ms = 0;     //10ms������
double g_lastValidCmdTime = 0.0; //��һ�ν�����Чָ���ʱ��,��ָ�ʱ��������ƶ���

s8 expectSpeedDir = 0;  //�����ʼ��Ϊ0
u16 expectBrakeVal = 0;  //�����ʼ��Ϊ0
u8  sonicDecisionMakingResult = 0;
	
void TIM7_IRQHandler(void)   //TIM7�ж�
{
	expectSpeedDir = 0;
	expectBrakeVal = 0;
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
		TargetTorque = 0;
		
        if(rc.sw2==3 ||  //�Ҳದ����λ /*�Զ���ʻ״̬  PID�ٶȿ���*/
		   rc.sw2==2)    //�Ҳದ����λ /*�ֶ���ʻ״̬  PID�ٶȿ���*/
        {
			if(rc.sw2==3) //�Զ�
			{
				//�жϿ���ָ���Ƿ�ʱ���糬��100msδ����ָ��ٶ�����
				if(g_seconds-g_lastValidCmdTime < 0.1)
					expectSpeed = Industry_info.TargetSpeed;
				else
				{
					expectSpeed = 0.0;
				}
				TargetAngle = Industry_info.TargetAngle;
				//Angle = -rc.ch1*65; //�Զ���ʻ״̬��ʹ��ң����ת�ǽ��в���
			}
			else  //�ֶ�
			{
				expectSpeed = 1.0 * rc.ch4/660 * munualCtrlMaxSpeed;
				TargetAngle = -rc.ch1 /660.0 * 12.3;	//rc.ch1 [-660, 660]
			}
			
			//�ٶ� PID
			if(expectSpeed == 0.0 && fabs(g_vehicleSpeed <= 0.3))
				TargetTorque = IncrementPIDspeedCtrl(&g_speedPID,expectSpeed, g_vehicleSpeed, g_seconds, 1);
			else
				TargetTorque = IncrementPIDspeedCtrl(&g_speedPID,expectSpeed, g_vehicleSpeed, g_seconds, 0);
			
			if(TargetTorque > 255) TargetTorque =255;
			else if(TargetTorque < -255) TargetTorque = -255;
			
			if(expectSpeed > 0) expectSpeedDir = 1;
			else if(expectSpeed < 0) expectSpeedDir = -1;	
			
			if(expectSpeedDir * TargetTorque < 0)  // �����ٶȷ���������Ť�ط���һ��ʱ��������Ҫ�����ƶ�����
				expectBrakeVal = abs(TargetTorque);
			else if(expectSpeedDir == 0)		   // �������ٶ�Ϊ0ʱ��������ζ���Ҫ�ƶ�(ȫŤ���ƶ�)
				expectBrakeVal = 255;
        }
        /*�ֶ���ʻ״̬  Ť�ؿ���*/
        else if(rc.sw2==1) //�Ҳದ����λ                                                            
        {
            TargetTorque = rc.ch4*255/660;   //rc.ch4 [-660, 660]
            TargetAngle = -rc.ch1 /660.0 * 12.3;              //rc.ch1 [-660, 660] Angle [-660*50, 660*50]
			
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
				//�����ٶ�Ϊ0ʱ�����������жϣ�Ĭ�Ϸ���Ϊ0 ����������ִ������ָ��
				if(TargetTorque > 0)
					expectSpeedDir = 1;
				else if(TargetTorque < 0)
					expectSpeedDir = -1;
			}
        }
    }
#if (ENABLE_SONIC == 1)
	if(rc.sw1==3) //��ದ����λ������״̬�ҳ�������Ч
	{
		if(g_timerCnt10ms%7 == 0) /*ÿ70msִ��һ�γ���������*/
			sonicDecisionMakingResult = SonicDecisionMaking();
		//������������ʹ�����޷�����ʱ������pid�����Է�ֹ�����������������������ۼƵ��µĴ�Ť��
		if(sonicDecisionMakingResult&0x01 && expectSpeedDir==1)//��ֹǰ���ҵ�ǰ�����ٶȷ���Ϊ��
		{
			TargetTorque = -255; //ȫŤ���ƶ�
			IncrementPIDspeedCtrl(&g_speedPID,expectSpeed, g_vehicleSpeed, g_seconds, 1); //����pid
		}
		if(sonicDecisionMakingResult&0x02 && expectSpeedDir==-1)//��ֹ�����ҵ�ǰ�����ٶȷ���Ϊ��
		{
			TargetTorque = 255;  //ȫŤ���ƶ�
			IncrementPIDspeedCtrl(&g_speedPID,expectSpeed, g_vehicleSpeed, g_seconds, 1); //����pid
		}
	}
	else if(rc.sw1==1) //��ದ����λ������״̬�����γ�����
		sonicDecisionMakingResult = 0;
#endif

    /*ÿ40ms����һ��ǰ��*/
    if(g_timerCnt10ms%5 == 0)
    {
//		DMS055A_SendPosition(-rc.ch1*110);
		if(g_angleSensorInited)
			setRoadWheelAngle(TargetAngle);
    }
	/*ÿ40ms��ȡһ��ת��������λ��*/
	if(g_timerCnt10ms%4 == 0)
	{
		DMS055A_ReadPosition();//��������
	}

    if(g_timerCnt10ms%5 == 0)/*ÿ50ms����һ�������*/
        YQRL_TRctrl(rc.sw1,Sonic_info.SonicAlarm,TargetTorque,expectSpeedDir);
 
    if(g_timerCnt10ms%5 == 1) /*ÿ50ms����һ���Һ���*/
        YQRR_TRctrl(rc.sw1,Sonic_info.SonicAlarm,TargetTorque,expectSpeedDir);
	
    /*ÿ50ms����ָ�����������������λ��*/
    if(g_timerCnt10ms%5 == 3)
    {
		expectBrakeVal *= 2;     //�Ŵ��ƶ���
		if(expectBrakeVal > 255)  
			expectBrakeVal = 255;  //�ƶ����޷�
		else if(expectBrakeVal < 80 && expectBrakeVal > 0) 
			expectBrakeVal = 80;
		
        JY01_Ctrl(rc.sw2,rc.sw1,Sonic_info.SonicAlarm,expectBrakeVal,RealRPM,TargetAngle); //����ǰ���ƶ�ָ��
        Industry_Reply(YQRL_info.RPM,YQRR_info.RPM,TargetAngle,TargetTorque, expectBrakeVal); //�򹤿��ϱ�����״̬
    }
    
    if(g_timerCnt10ms%8 == 0) /*ÿ80ms����ָ�����λ�����ӣ�*/
        Client_Ctrl(rc.sw2,rc.sw1,sonicDecisionMakingResult,RealRPM,TargetAngle);
}

