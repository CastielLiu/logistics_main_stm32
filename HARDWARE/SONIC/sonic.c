#include "sonic.h"
#include "string.h"
#include "remoter.h"
#include "can.h"

Sonic_info_t Sonic_info;
u8 SonicHost_Buf[Sonic_Buf_Len];
u8 SonicClient_Buf[Sonic_Buf_Len];

void Sonic_Init(u32 baudrate)
{
    //GPIO�˿�����
    GPIO_InitTypeDef 			GPIO_InitStructure;
    USART_InitTypeDef 			USART_InitStructure;
    NVIC_InitTypeDef 			NVIC_InitStructure;
    DMA_InitTypeDef				DMA_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE);

    /*������ģ�飨��������->USART1*/
    //Sonic_TX -> USART1_TX -> PB.6
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    //Sonic_RX -> USART1_RX -> PB.7
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    /*������ģ�飨�ӣ�����->USART3*/
    //Sonic_TX -> USART3_TX -> PB.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    //Sonic_RX -> USART3_RX -> PB.11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    /*������ģ�飨����USART1 ��ʼ������*/
    USART_InitStructure.USART_BaudRate = baudrate;//���ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART1,&USART_InitStructure); //��ʼ������1

    USART_Cmd(USART1,ENABLE);						//ʹ�ܴ���1
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);	//ʹ�ܴ���1����DMA����

    /*������ģ�飨�ӣ�USART3 ��ʼ������*/
    USART_InitStructure.USART_BaudRate = baudrate;//���ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART3,&USART_InitStructure); //��ʼ������3

    USART_Cmd(USART3,ENABLE);						//ʹ�ܴ���3
    USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);	//ʹ�ܴ���3����DMA����

    /*������ģ�飨����USART1 NVIC ����*/
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

    USART_ClearFlag(USART1,USART_FLAG_IDLE);
    USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);	//��������1�����ж�

    /*������ģ�飨�ӣ�USART3 NVIC ����*/
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

    USART_ClearFlag(USART3,USART_FLAG_IDLE);
    USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);	//��������3�����ж�

    /*������ģ�飨����USART1-DMA����*/
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)SonicHost_Buf;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = Sonic_Buf_MaxLen;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5,&DMA_InitStructure);

    DMA_ClearFlag(DMA1_FLAG_TC5);
    DMA_Cmd(DMA1_Channel5,ENABLE);

    /*������ģ�飨�ӣ�USART3-DMA����*/
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)SonicClient_Buf;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = Sonic_Buf_MaxLen;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel3,&DMA_InitStructure);

    DMA_ClearFlag(DMA1_FLAG_TC3);
    DMA_Cmd(DMA1_Channel3,ENABLE);
}


u16 Sonic_CRC16(u8 *puchMsg,u16 usDataLen)
{
    unsigned char uchCRCHi = 0xFF ; /* ��CRC�ֽڳ�ʼ�� */
    unsigned char uchCRCLo = 0xFF ; /* ��CRC �ֽڳ�ʼ�� */
    unsigned uIndex ; /* CRCѭ���е����� */

    while (usDataLen--) /* ������Ϣ������ */
    {
        uIndex = uchCRCHi ^ *puchMsg++ ; /* ����CRC */
        uchCRCHi = uchCRCLo ^ SonicCRCHi[uIndex];
        uchCRCLo = SonicCRCLo[uIndex] ;
    }

    return (uchCRCHi<<8|uchCRCLo) ;
}

void Sonic_SendHostData(u8 *s,u8 len)
{
    while(len--)
    {
        while(USART_GetFlagStatus(USART1,USART_FLAG_TC )==RESET);
        USART_SendData(USART1,*s);
        s++;
    }
}

void Sonic_SendClientData(u8 *s,u8 len)
{
    while(len--)
    {
        while(USART_GetFlagStatus(USART3,USART_FLAG_TC )==RESET);
        USART_SendData(USART3,*s);
        s++;
    }
}

void Sonic_SendHostCmd(void)
{
    static u8 cmd[8] = {0x09,0x03,0x00,0x01,0x00,0x08,0x14,0x84};

//    cmd[6] = (Sonic_CRC16(cmd,6)>>8)&0xFF;
//    cmd[7] = Sonic_CRC16(cmd,6)&0xFF;

    Sonic_SendHostData(cmd,8);
}

void Sonic_SendClientCmd(void)
{
    static u8 cmd[8] = {0x09,0x03,0x00,0x01,0x00,0x08,0x14,0x84};

//    cmd[6] = (Sonic_CRC16(cmd,6)>>8)&0xFF;
//    cmd[7] = Sonic_CRC16(cmd,6)&0xFF;

    Sonic_SendClientData(cmd,8);
}


void Get_SonicData(Sonic_info_t *Sonic_info,u8 *buf)
{
    u16 CRC_Data;

    CRC_Data  = (buf[19]<<8)|buf[20];

    if(CRC_Data==Sonic_CRC16(buf,19))
    {
        if(buf[0]==0x09)
        {
            Sonic_info->A1 	= (buf[3]<<8)|buf[4];
            Sonic_info->A2  = (buf[5]<<8)|buf[6];
            Sonic_info->A3	= (buf[7]<<8)|buf[8];
            Sonic_info->A4	= (buf[9]<<8)|buf[10];
            Sonic_info->A5	= (buf[11]<<8)|buf[12];
            Sonic_info->A6	= (buf[13]<<8)|buf[14];
            Sonic_info->A7	= (buf[15]<<8)|buf[16];
            Sonic_info->A8	= (buf[17]<<8)|buf[18];
        }
        else if(buf[0]==0x07)
        {
            Sonic_info->B1 	= (buf[3]<<8)|buf[4];
            Sonic_info->B2  = (buf[5]<<8)|buf[6];
            Sonic_info->B3	= (buf[7]<<8)|buf[8];
            Sonic_info->B4	= (buf[9]<<8)|buf[10];
            Sonic_info->B5	= (buf[11]<<8)|buf[12];
            Sonic_info->B6	= (buf[13]<<8)|buf[14];
        }
    }
    else
    {
        memset(Sonic_info,0,sizeof(Sonic_info_t));
        return;
    }
}


void Safety_Ctrl(int Speed,int Alpha)
{
    if(Speed>0)
    {
        if(Alpha<-50)
        {
            if((Sonic_info.A1<EmergenDis))
                Sonic_info.SonicAlarm = 1;
            else
                Sonic_info.SonicAlarm = 0;
        }
        else if((Alpha>-50)&&(Alpha<50))
        {
            if((Sonic_info.A2<EmergenDis)||(Sonic_info.A3<EmergenDis))
                Sonic_info.SonicAlarm = 1;
            else
                Sonic_info.SonicAlarm = 0;
        }
        else if(Alpha>50)
        {
            if((Sonic_info.A4<EmergenDis))
                Sonic_info.SonicAlarm = 1;
            else
                Sonic_info.SonicAlarm = 0;
        }
    }
    else if(Speed<0)
    {
        if(Alpha<-50)
        {
            if((Sonic_info.A8<EmergenDis))
                Sonic_info.SonicAlarm = 1;
            else
                Sonic_info.SonicAlarm = 0;
        }
        else if((Alpha>-50)&&(Alpha<50))
        {
            if((Sonic_info.A6<EmergenDis)||(Sonic_info.A7<EmergenDis))
                Sonic_info.SonicAlarm = 1;
            else
                Sonic_info.SonicAlarm = 0;
        }
        else if(Alpha>50)
        {
            if((Sonic_info.A5<EmergenDis))
                Sonic_info.SonicAlarm = 1;
            else
                Sonic_info.SonicAlarm = 0;
        }
    }
//		if(Sonic_info.SonicAlarm)
//			Beep_Alarm(2,100);
}


u8 db_usart1_len,db_usart3_len;
u8 db_cmd7;

void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1,USART_IT_IDLE)!=RESET)
    {
        (void)USART1->SR;
        (void)USART1->DR;

        DMA_Cmd(DMA1_Channel5,DISABLE);
        if(Sonic_Buf_MaxLen-DMA_GetCurrDataCounter(DMA1_Channel5)==Sonic_Buf_Len)
            Get_SonicData(&Sonic_info,SonicHost_Buf);
		
		db_usart1_len = DMA_GetCurrDataCounter(DMA1_Channel5);

        DMA_SetCurrDataCounter(DMA1_Channel5,Sonic_Buf_MaxLen);
        DMA_Cmd(DMA1_Channel5,ENABLE);
    }
}

void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3,USART_IT_IDLE)!=RESET)
    {
        (void)USART3->SR;
        (void)USART3->DR;

        DMA_Cmd(DMA1_Channel3,DISABLE);
        if(Sonic_Buf_MaxLen-DMA_GetCurrDataCounter(DMA1_Channel3)==Sonic_Buf_Len)
            Get_SonicData(&Sonic_info,SonicClient_Buf);
		
		db_usart3_len = DMA_GetCurrDataCounter(DMA1_Channel3);

        DMA_SetCurrDataCounter(DMA1_Channel3,Sonic_Buf_MaxLen);
        DMA_Cmd(DMA1_Channel3,ENABLE);
    }
}

