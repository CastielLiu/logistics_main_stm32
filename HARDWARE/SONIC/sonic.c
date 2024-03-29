#include "sonic.h"
#include "string.h"
#include "remoter.h"
#include "can.h"

Sonic_info_t Sonic_info;
SonicPositive_info_t	SonicPositive_info;
u8 SonicHost_Buf[Sonic_Buf_MaxLen];
u8 SonicClient_Buf[Sonic_Buf_MaxLen];

void Sonic_Init(u32 baudrate)
{
    //GPIO端口设置
    GPIO_InitTypeDef 			GPIO_InitStructure;
    USART_InitTypeDef 		USART_InitStructure;
    NVIC_InitTypeDef 			NVIC_InitStructure;
    DMA_InitTypeDef				DMA_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE);

    /*超声波模块（主）串口->USART1*/
    //Sonic_TX -> USART1_TX -> PB.6
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    //Sonic_RX -> USART1_RX -> PB.7
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    /*超声波模块（从）串口->USART3*/
    //Sonic_TX -> USART3_TX -> PB.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    //Sonic_RX -> USART3_RX -> PB.11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
		

    /*超声波模块（主）USART1 初始化设置*/
    USART_InitStructure.USART_BaudRate = baudrate;//串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART1,&USART_InitStructure); //初始化串口1

    USART_Cmd(USART1,ENABLE);						//使能串口1
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);	//使能串口1接收DMA传输

    /*超声波模块（从）USART3 初始化设置*/
    USART_InitStructure.USART_BaudRate = baudrate;//串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART3,&USART_InitStructure); //初始化串口3

    USART_Cmd(USART3,ENABLE);						//使能串口3
    USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);	//使能串口3接收DMA传输

    /*超声波模块（主）USART1 NVIC 配置*/
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

    USART_ClearFlag(USART1,USART_FLAG_IDLE);
    USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);	//开启串口1空闲中断

    /*超声波模块（从）USART3 NVIC 配置*/
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

    USART_ClearFlag(USART3,USART_FLAG_IDLE);
    USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);	//开启串口3空闲中断

    /*超声波模块（主）USART1-DMA设置*/
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

    /*超声波模块（从）USART3-DMA设置*/
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
    unsigned char uchCRCHi = 0xFF ; /* 高CRC字节初始化 */
    unsigned char uchCRCLo = 0xFF ; /* 低CRC 字节初始化 */
    unsigned uIndex ; /* CRC循环中的索引 */

    while (usDataLen--) /* 传输消息缓冲区 */
    {
        uIndex = uchCRCHi ^ *puchMsg++ ; /* 计算CRC */
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

/*
*********************************************************************
* @  name :Get_SonicData
* @  func :获取超声波数据（DBUS_RTU模式）
* @ prama :Soniccc_info-超声波解析数据结构
**********：buf-超声波接收缓冲区
* @retval :无
*********************************************************************
* @attention :
*********************************************************************
*/
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

/*
*********************************************************************
* @  name :Get_SonicData
* @  func :获取超声波数据（主动上传模式）
* @ prama :SonicPositive_info-超声波解析数据结构
**********：buf-超声波接收缓冲区
* @retval :无
*********************************************************************
* @attention :
*********************************************************************
*/
void Get_SonicDataPositive(SonicPositive_info_t *SonicPositive_info,u8 *buf)
{
	u8 index;
	if((buf[0]==0x55)&&(buf[1]==0x54))
	{
		for(index=3;index<=10;index++)
		{
			if(buf[index]==0)
				buf[index]=255;
		}
		SonicPositive_info->A1 = buf[3];
		SonicPositive_info->A2 = buf[4];
		SonicPositive_info->A3 = buf[5];
		SonicPositive_info->A4 = buf[6];
		SonicPositive_info->A5 = buf[7];
		SonicPositive_info->A6 = buf[8];
		SonicPositive_info->A7 = buf[9];
		SonicPositive_info->A8 = buf[10];
	}
 else
	 {
        memset(SonicPositive_info,0,sizeof(SonicPositive_info_t));
        return;
 }
}

u8 g_sonicEmergenDis = 50; //cm

void Safety_Ctrl(int Speed,int Alpha)
{
	
    if(Speed>0)
    {
        if(Alpha<-50)
        {
            if((Sonic_info.A1<g_sonicEmergenDis))
                Sonic_info.SonicAlarm = 1;
            else
                Sonic_info.SonicAlarm = 0;
						
        }
        else if((Alpha>-50)&&(Alpha<50))
        {
            if((Sonic_info.A2<g_sonicEmergenDis)||(Sonic_info.A3<g_sonicEmergenDis))
                Sonic_info.SonicAlarm = 1;
            else
                Sonic_info.SonicAlarm = 0;
        }
        else if(Alpha>50)
        {
            if((Sonic_info.A4<g_sonicEmergenDis))
                Sonic_info.SonicAlarm = 1;
            else
                Sonic_info.SonicAlarm = 0;
        }
    }
    else if(Speed<0)
    {
        if(Alpha<-50)
        {
            if((Sonic_info.A8<g_sonicEmergenDis))
                Sonic_info.SonicAlarm = 1;
            else
                Sonic_info.SonicAlarm = 0;
        }
        else if((Alpha>-50)&&(Alpha<50))
        {
            if((Sonic_info.A6<g_sonicEmergenDis)||(Sonic_info.A7<g_sonicEmergenDis))
                Sonic_info.SonicAlarm = 1;
            else
                Sonic_info.SonicAlarm = 0;
        }
        else if(Alpha>50)
        {
            if((Sonic_info.A5<g_sonicEmergenDis))
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
		db_usart1_len = DMA_GetCurrDataCounter(DMA1_Channel5);
		
        if(Sonic_Buf_MaxLen-DMA_GetCurrDataCounter(DMA1_Channel5)==Sonic_Buf_Len)
            Get_SonicData(&Sonic_info,SonicHost_Buf);
		
		
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
		db_usart3_len = DMA_GetCurrDataCounter(DMA1_Channel3);
			
        if(Sonic_Buf_MaxLen-DMA_GetCurrDataCounter(DMA1_Channel3)==Sonic_Buf_Len)
            Get_SonicData(&Sonic_info,SonicClient_Buf);
		else if(Sonic_Buf_MaxLen-DMA_GetCurrDataCounter(DMA1_Channel3)==Sonic_PositiveBuf_Len)
			Get_SonicDataPositive(&SonicPositive_info,SonicClient_Buf);
		
        DMA_SetCurrDataCounter(DMA1_Channel3,Sonic_Buf_MaxLen);
        DMA_Cmd(DMA1_Channel3,ENABLE);
    }
}

//根据超声波检测具体进行决策
/*return  决策结果
		 0x01 禁止前进
		 0x02 禁止后退
		 0x03 禁止运行
*/

u8 generateSonicEmergencyDis()
{
	float res;
	//超声波安全距离随速度变化，但不小于50cm
	
	if(g_vehicleSpeed > 0)
		res = g_vehicleSpeed  * 25 +50;
	else
		res = -g_vehicleSpeed  * 25 +50;
	
	if(res > 255)
		res = 255;
	return (u8)res;
}

u8 SonicDecisionMaking()
{
	static s8 lastRes = 0;
	static double lastBandonForwardTime = 0;
	static double lastBandonBackwardTime = 0;
	static float timeout = 1.0; //s
	
	double now = g_seconds;
	u8 result = 0;
	g_sonicEmergenDis = generateSonicEmergencyDis(); 
	
	if((SonicPositive_info.A1 < g_sonicEmergenDis) || (SonicPositive_info.A2 < g_sonicEmergenDis) ||
	    (SonicPositive_info.A3 < g_sonicEmergenDis) || (SonicPositive_info.A4 < g_sonicEmergenDis)) //禁止前进
	{
		result |= 0x01;
		lastBandonForwardTime = now;
	}
	if((SonicPositive_info.A5 < g_sonicEmergenDis) || (SonicPositive_info.A6 < g_sonicEmergenDis) ||
	    (SonicPositive_info.A7 < g_sonicEmergenDis) || (SonicPositive_info.A8 < g_sonicEmergenDis)) //禁止后退
	{
		result |= 0x02;
		lastBandonBackwardTime = now;
	}
	
	if((result&0x01) == 0) //可前进
	{
		if(now - lastBandonForwardTime < timeout)
			result |= 0x01;
	}
	if((result&0x02) == 0) //可后退
	{
		if(now - lastBandonBackwardTime < timeout)
			result |= 0x02;
	}

	lastRes = result;	
	return result;
}

