#include "dms055a.h"
#include "remoter.h"
#include "math.h"
#include "string.h"


u8 DMS055A_Buf[DMS055A_Buf_Len];

void DMS055A_Init(u32 baudrate)
{
    GPIO_InitTypeDef			GPIO_InitStructure;
    USART_InitTypeDef			USART_InitStructure;
    DMA_InitTypeDef				DMA_InitStructure;
    NVIC_InitTypeDef			NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);

	#if AUTO_RXTX_485 == 0 //���Զ��շ����ֶ�����Ϊ����ģʽ��PD7���ߣ�����ת����
	//PD7 �շ���������
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; 
    GPIO_Init(GPIOD,&GPIO_InitStructure);
	GPIO_SetBits(GPIOD,GPIO_Pin_7);	
	
#endif
	
    //DMS055A_TX -> UART4_TX -> PC.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC,&GPIO_InitStructure);

    //DMS055A_RX -> UART4_RX -> PC.11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC,&GPIO_InitStructure);

    //USART ��ʼ������
    USART_InitStructure.USART_BaudRate = baudrate;//���ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(UART4,&USART_InitStructure); //��ʼ������4

    USART_Cmd(UART4,ENABLE);						//ʹ�ܴ���4
    USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);	//ʹ�ܴ���4_DMA����

    //UART4 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;	//��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

    USART_ClearFlag(UART4,USART_FLAG_IDLE);		//�������4�����жϱ�־λ
    USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);	//��������4�����ж�

    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)DMS055A_Buf;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART4->DR;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = DMS055A_Buf_Len;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA2_Channel3,&DMA_InitStructure);

    DMA_ClearFlag(DMA2_FLAG_TC3);
    DMA_Cmd(DMA2_Channel3,ENABLE);
}
/*
*********************************************************************
* @  name :DMS055A_SendData
* @  func :��ת��������������Ϣ
* @ prama :	*s:��Ҫ���͵���Ϣ	len:��Ϣ����
* @retval :��
*********************************************************************
* @attention :ת���������ӵ�����4
*********************************************************************
*/
void DMS055A_SendData(u8 *s,u8 len)
{
    while(len--)
    {
        while(USART_GetFlagStatus(UART4,USART_FLAG_TC )==RESET);
        USART_SendData(UART4,*s);
        s++;
    }
}

/*
*********************************************************************
* @  name :Get_DMS055A_Data
* @  func :����ת��������Ӧ����Ϣ
* @ prama : *DMS055A_info:���������ݴ�Žṹ��	buf:���ڽ��յ��Ļ�����
* @retval : ��
*********************************************************************
* @attention :
*********************************************************************
*/
void Get_DMS055A_Data(DMS055A_info_t *DMS055A_info,u8 *buf)
{
    DMS055A_info->DevAddr = buf[0];
    DMS055A_info->FunCode = buf[1];

    if(DMS055A_info->FunCode==0x03)//485��ȡ����ʱ�ӻ�Ӧ��
    {
        DMS055A_info->DataLen = buf[2];
        DMS055A_info->Current = (buf[3]<<8)|buf[4];
        DMS055A_info->CRCdata = (buf[5]<<8)|buf[6];
    }
}

/*
*********************************************************************
* @  name :DMS055A_SendPosition
* @  func :����ǰ�����������Ŀ��λ�ã�����λ�ã�
* @ prama :Target:Ŀ��λ��
* @retval :��
*********************************************************************
* @attention :ң�ش�������Ϊ�з���������Ҫת�����޷�����ͨ�����ڷ���
*********************************************************************
*/
void DMS055A_SendPosition(int Target)
{
    u8  modbus_data[13] = {0x01,0x10,0x00,0x16,0x00,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00};
    u8  PU24_31,PU16_23,PU8_15,PU0_7;
    u16 crc_data;

    /*��������ת����32λ16�����޷���������*/
    if(Target<0)
    {
        Target = 0x7FFFFFFF+(Target+1);
        PU24_31 = (Target/(256*65536))+0x80;
    }
    else if(Target==0)
        Target = 1;
    else
    {
        PU24_31 =(Target/(256*65536));
    }
    PU16_23 = (Target/65536)%256;
    PU8_15 = (Target/256)%256;
    PU0_7 = Target%256;

    /*��ת���õ�����������������*/
    modbus_data[7] = PU8_15;
    modbus_data[8] = PU0_7;
    modbus_data[9] = PU24_31;
    modbus_data[10] = PU16_23;

    crc_data = DMS055A_CRC16(modbus_data,11);

    modbus_data[11] = crc_data/256%256;
    modbus_data[12] = crc_data%256;

    /*�����ݰ����͸�ǰ��������*/
    DMS055A_SendData(modbus_data,13);
}

/*
*********************************************************************
* @  name :DMS055A_FindEN
* @  func :ǰ����������ԭ��λ��
* @ prama :��
* @retval :��
*********************************************************************
* @attention :��0x19�Ĵ���д2����������EN�źţ�������ֵ������ֵ��������EN�ź�
*********************************************************************
*/
void DMS055A_FindEN(void)
{
    u8  FindEN[8] = {0x01,0x06,0x00,0x19,0x00,0x02,0x00,0x00};
    u16 crc_data;

    crc_data = DMS055A_CRC16(FindEN,6);

    FindEN[6] = crc_data/256%256;
    FindEN[7] = crc_data%256;
    DMS055A_SendData(FindEN,8);
}

/*
*********************************************************************
* @  name :DMS055A_ReadCurrent
* @  func :��ȡת���������ĵ���ֵ
* @ prama :��
* @retval :��
*********************************************************************
* @attention :
*********************************************************************
*/
void DMS055A_ReadCurrent(void)
{
    u8  ReadCurrent[8] = {0x01,0x03,0x00,0x0F,0x00,0x01,0x00,0x00};
    u16 crc_data;

    crc_data = DMS055A_CRC16(ReadCurrent,6);

    ReadCurrent[6] = crc_data/256%256;
    ReadCurrent[7] = crc_data%256;

    DMS055A_SendData(ReadCurrent,8);
}

void DMS055A_ReadPosition(void)
{
    u8 ReadPos[8] = {0x01,0x03,0x00,0x16,0x00,0x02,0x25,0xCF};

    DMS055A_SendData(ReadPos,8);
}

/*
******************************
* @ 
******************************
*/
void targetAngleTransform(Industry_info_t	Industry_info)
{
		Industry_info.TargetAngle = Industry_info.TargetAngle*660/1025*65*100.0;
		/*ת���޷�*/
		if(Industry_info.TargetAngle>660*65)
			Industry_info.TargetAngle = 660*65;
		else if(Industry_info.TargetAngle<-660*65)
			Industry_info.TargetAngle = -660*65;
}

//��ȡg_sensorAngle��ֵ
//void sendTargetAngle(float t_angle)
//{
//	//float sensor_angle = 0.0;
////	float angle = ;
////	
////	DMS055A_SendPosition(angle)
//}

/*
*********************************************************************
* @  name :CRC16
* @  func :DMS-055A������CRCУ��
* @ prama :*puchMsg��Ҫ����CRC У�����Ϣ
			usDataLen����Ϣ���ֽ���
* @retval :CRCУ����
*********************************************************************
* @attention :
*********************************************************************
*/
unsigned short DMS055A_CRC16(unsigned char *puchMsg,unsigned short usDataLen)
{
    unsigned char uchCRCHi = 0xFF ; /* ��CRC �ֽڳ�ʼ��*/
    unsigned char uchCRCLo = 0xFF ; /* ��CRC �ֽڳ�ʼ��*/
    unsigned  uIndex; /* CRC ѭ���е�����*/

    while (usDataLen--) /* ������Ϣ������*/
    {
        uIndex = uchCRCHi ^ *puchMsg++; /* ����CRC */
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];
    }
    return (uchCRCHi<<8|uchCRCLo);
}


DMS055A_info_t  DMS055A_info;

void UART4_IRQHandler(void)
{
    if(USART_GetITStatus(UART4,USART_IT_IDLE)!=RESET)
    {
        (void)UART4->SR;
        (void)UART4->DR;
		DMA_Cmd(DMA2_Channel3,DISABLE);
			
        if(DMS055A_Buf[0]==0x01) //�豸��ַ(ǰ��ת��������)
        {
            Get_DMS055A_Data(&DMS055A_info,DMS055A_Buf);
        }
        DMA_SetCurrDataCounter(DMA2_Channel3,DMS055A_Buf_Len);
        DMA_Cmd(DMA2_Channel3,ENABLE);
    }
}

