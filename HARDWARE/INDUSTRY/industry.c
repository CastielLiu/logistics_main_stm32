#include "industry.h"
#include "remoter.h"
#include "stdlib.h"
#include "dms055a.h"
#include "string.h"
#include "timer.h"

u8 Industry_Buf[Industry_Buf_Len];
Industry_info_t Industry_info;

void Industry_Init(u32 baudrate)
{
    //GPIO�˿�����
    GPIO_InitTypeDef 			GPIO_InitStructure;
    USART_InitTypeDef 		USART_InitStructure;
    DMA_InitTypeDef				DMA_InitStructure;
    NVIC_InitTypeDef 			NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);

    //Industry_TX -> USART2_TX -> PA.2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //Industry_RX -> USART2_RX -> PA.3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //USART ��ʼ������
    USART_InitStructure.USART_BaudRate = baudrate;//���ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART2, &USART_InitStructure); //��ʼ������2

    USART_Cmd(USART2, ENABLE);						//ʹ�ܴ���2
    USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);	//ʹ�ܴ���2_DMA����

    //Usart2 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 1;//��ռ���ȼ�1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

    USART_ClearFlag(USART2,USART_FLAG_IDLE);		//��������жϱ�־λ
    USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);	//��������2�����ж�

    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)Industry_Buf;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART2->DR;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = Industry_Buf_MaxLen;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel6,&DMA_InitStructure);

    DMA_ClearFlag(DMA1_FLAG_TC6);
    DMA_Cmd(DMA1_Channel6,ENABLE);
}


void Industry_SendData(u8 *s,u8 len)
{
    while(len--)
    {
        while(USART_GetFlagStatus(USART2,USART_FLAG_TC )==RESET);
        USART_SendData(USART2,*s);
        s++;
    }
}

void Get_Industry_Data(Industry_info_t	*Industry_info,u8 *buf, int len)
{
	u8 pkgId, dataLen;
	
	if(len < 5) //���ݰ�����С����Ϊ5(�����κ�����)
		return;
	
    if((buf[0]==0x66)&&(buf[1]==0xCC))
    {
		pkgId = buf[2];
		dataLen = buf[3];
		if(generateCheckVal(buf+2, dataLen+2) != buf[dataLen+4])
			return ;
			
        if(pkgId == 0x01 && len >= 11) //�ٶȡ�ת�����ָ�����ݰ�
        {
			g_lastValidCmdTime = g_seconds;
            Industry_info->EN = buf[4];

            Industry_info->TargetSpeed = (((buf[5]<<8)|buf[6]) - 30000)/100.0;

            Industry_info->TargetAngle = (buf[7]<<8)|buf[8];
            Industry_info->TargetAngle = (Industry_info->TargetAngle-30000);
					
            Industry_info->TargetAngle = Industry_info->TargetAngle*660/1230*65;
			/*ת���޷�*/
			if(Industry_info->TargetAngle>660*65)
				Industry_info->TargetAngle = 660*65;
			else if(Industry_info->TargetAngle<-660*65)
				Industry_info->TargetAngle = -660*65;

            Industry_info->BrakeSig = buf[9];
            Industry_info->SumCheck = buf[10];
        }
		else if(pkgId == 0x05 && len >= 11) //�����ٶ�PID����
		{
			g_speedPID.kp = ((buf[4] * 256 + buf[5]) - 30000)/100.0;
			g_speedPID.ki = ((buf[6] * 256 + buf[7]) - 30000)/100.0;
			g_speedPID.kd = ((buf[8] * 256 + buf[9]) - 30000)/100.0;
			feedbackSpeedPID();
		}
		else if(pkgId == 0x06) //��ȡ�ٶ�PID����
		{
			feedbackSpeedPID();
		}
		
		buf[2] = 0; //�����ݰ�ID��λ
    }
	/*else
	{
		memset(Industry_info, 0, sizeof(Industry_info_t));
        return ;
	}*/
}

void feedbackSpeedPID()
{
	u8 buf[11] = {0x66, 0xcc, 0x05, 0x06};
	buf[4] = ((uint16_t)(g_speedPID.kp*100 + 30000))/256;
	buf[5] = ((uint16_t)(g_speedPID.kp*100 + 30000))%256;
	buf[6] = ((uint16_t)(g_speedPID.ki*100 + 30000))/256;
	buf[7] = ((uint16_t)(g_speedPID.ki*100 + 30000))%256;
	buf[8] = ((uint16_t)(g_speedPID.kd*100 + 30000))/256;
	buf[9] = ((uint16_t)(g_speedPID.kd*100 + 30000))%256;
	buf[10] = generateCheckVal(buf+2, buf[3]+2);
	
	Industry_SendData(buf, 11);
}

void DriveLess_Ctrl(void)
{
	
}

void Industry_Reply(s16 LrRealRPM,s16 RrRealRPM,s16 Angle,s16 targetTorque, u8 targetBrakeVal)
{
	const int dataLen = 9;
	
    u8 IndustryRepMsg[dataLen + 5] = {0x66,0xCC,0x02,dataLen};

//		Speed = Speed*PI*2/60;			//���ٶ�rad/s
//		Speed = Speed*WheelRadius;	//���ٶ�m/s
//		Speed = Speed*360;					//���ٶ�0.01km/h
		
    //�����ת��Ӧ�� r/min
    IndustryRepMsg[4] = (LrRealRPM+30000)/256;
    IndustryRepMsg[5] = (LrRealRPM+30000)%256;
		
	//�Һ���ת��Ӧ��
    IndustryRepMsg[6] = (RrRealRPM+30000)/256;
    IndustryRepMsg[7] = (RrRealRPM+30000)%256;

    //ǰ��ת��Ӧ��
    IndustryRepMsg[8] = ((Angle*10+30000)>>8)&0xFF;
    IndustryRepMsg[9] = (Angle*10+30000)&0xFF;
	
	IndustryRepMsg[10] = (targetTorque + 30000) >> 8;
	IndustryRepMsg[11] = (targetTorque + 30000);
	
	IndustryRepMsg[12] = targetBrakeVal;
	

    //У��ͼ���
    IndustryRepMsg[dataLen+4] = generateCheckVal(IndustryRepMsg+2, dataLen+2);

    Industry_SendData(IndustryRepMsg,14);
}

void USART2_IRQHandler(void)
{
	int length;
    if(USART_GetITStatus(USART2,USART_IT_IDLE)!=RESET)	//�� ���յ�һ֡����
    {
        (void)USART2->SR;
        (void)USART2->DR;

        DMA_Cmd(DMA1_Channel6,DISABLE);
			
		length = Industry_Buf_MaxLen-DMA_GetCurrDataCounter(DMA1_Channel6);

        //if(length == Industry_Buf_Len)
        {
            Get_Industry_Data(&Industry_info,Industry_Buf,length);
        }
				
        DMA_SetCurrDataCounter(DMA1_Channel6,Industry_Buf_MaxLen);
        DMA_Cmd(DMA1_Channel6,ENABLE);
    }
}

u8 generateCheckVal(u8* buf, int len)
{
	int i=0;
	u8 sum = 0;
	for(; i<len; ++i)
		sum += buf[i];
	return sum;
}
