#include "can.h"
#include "led.h"
#include "delay.h"
#include "remoter.h"

float g_prm2speed = 3.1415926*0.355/60.0;  //Pi*D/60 ת��r/min���Ը�ϵ���õ�m/s  
YQ_info_t YQFL_info,YQFR_info,YQRL_info,YQRR_info;

//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024;  tq=(brp)*tpclk1
//������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ36M,�������CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);
//������Ϊ:36M/((8+9+1)*4)=500Kbps
//����ֵ:0,��ʼ��OK;
//����,��ʼ��ʧ��;
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
    GPIO_InitTypeDef 				GPIO_InitStructure;
    CAN_InitTypeDef        			CAN_InitStructure;
    CAN_FilterInitTypeDef  			CAN_FilterInitStructure;
#if CAN_RX0_INT_ENABLE
    NVIC_InitTypeDef  				NVIC_InitStructure;
#endif
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��

#if CAN1_REMAP2_PB89
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//ʹ��PORTBʱ��

	GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);

    //CAN_TX->PB.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//��������
    GPIO_Init(GPIOB, &GPIO_InitStructure);			//��ʼ��IO
    //CAN_RX->PB.8
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//��������
    GPIO_Init(GPIOB, &GPIO_InitStructure);			//��ʼ��IO

#else
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��PORTBʱ��
	//CAN_TX->PA12
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);			//��ʼ��IO
    //CAN_RX->PA11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//��ʼ��IO

#endif

    //CAN��Ԫ����
    CAN_InitStructure.CAN_TTCM=DISABLE;			//��ʱ�䴥��ͨ��ģʽ
    CAN_InitStructure.CAN_ABOM=DISABLE;			//����Զ����߹���
    CAN_InitStructure.CAN_AWUM=ENABLE;			//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
    CAN_InitStructure.CAN_NART=ENABLE;			//�������Զ�����
    CAN_InitStructure.CAN_RFLM=DISABLE;		 	//���Ĳ�����,�µĸ��Ǿɵ�
    CAN_InitStructure.CAN_TXFP=ENABLE;			//���ȼ��ɱ��ı�ʶ������
    CAN_InitStructure.CAN_Mode= mode;	        //ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ;
    //���ò�����
    CAN_InitStructure.CAN_SJW=tsjw;				//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
    CAN_InitStructure.CAN_BS1=tbs1; 			//Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq
    CAN_InitStructure.CAN_BS2=tbs2;				//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler=brp;        //��Ƶϵ��(Fdiv)Ϊbrp+1
    CAN_Init(CAN1, &CAN_InitStructure);        	//��ʼ��CAN1

    CAN_FilterInitStructure.CAN_FilterNumber=0;	//������0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 	//����λģʽ
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 	//32λ��
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;	//32λID
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//���������0

    CAN_FilterInit(&CAN_FilterInitStructure);			//�˲�����ʼ��

#if CAN_RX0_INT_ENABLE
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);				//FIFO0��Ϣ�Һ��ж�����.

    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     // �����ȼ�Ϊ2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;            // �����ȼ�Ϊ3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
    return 0;
}


/**
*********************************************************************
* @brief :Get_YQ1_Data
* @func  :������������������������֡1��Ϣ
* @prama :YQ_info-Զ����Ϣ�����ṹ�壬buf-CAN����������
* @retval:��
********************************************************************
* @attention :
*********************************************************************
  */
void Get_YQ1_Data(YQ_info_t *YQ_info,u8 *buf)
{
    YQ_info->Shift = buf[0];  //0 �յ� 1ǰ�� 2���� 3����
    YQ_info->Throttle = (buf[2]<<8)|buf[1];
    YQ_info->WorkState = buf[3];
    YQ_info->RPM = (buf[5]<<8)|buf[4];

    if(YQ_info->Shift==2)
        YQ_info->RPM = -YQ_info->RPM;
	
	YQ_info->RPM /= 5;
	YQ_info->speed = g_prm2speed * YQ_info->RPM;
    YQ_info->MotorTemp = buf[6]-40;
    YQ_info->CtrlTemp = buf[7]-40;
}

/**
*********************************************************************
* @brief :Get_YQ2_Data
* @func  :������������������������֡2��Ϣ
* @prama :YQ_info-Զ����Ϣ�����ṹ�壬buf-CAN����������
* @retval:��
********************************************************************
* @attention :
*********************************************************************
  */
void Get_YQ2_Data(YQ_info_t *YQ_info,u8 *buf)
{
    YQ_info->VBusVol = (buf[1]<<8)|buf[0];
    YQ_info->MotorCur = (buf[3]<<8)|buf[2];
    YQ_info->Lap     = (buf[5]<<8)|buf[4];
    YQ_info->Fault   = (buf[7]<<8)|buf[6];
}

#if CAN_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����

CanRxMsg RxMessage;
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!=RESET)
    {
        CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);

        CAN_Receive(CAN1,0,&RxMessage);
    }
    switch(RxMessage.ExtId)
    {
    /*��ȡ��ǰ����Ϣ*/
    case YQFL_EXTID1:
    {
        Get_YQ1_Data(&YQFL_info,RxMessage.Data);
        break;
    }
//    case YQFL_EXTID2:
//    {
//        Get_YQ2_Data(&YQFL_info,RxMessage.Data);
//        break;
//    }
    /*������ǰ����Ϣ*/
    case YQFR_EXTID1:
    {
        Get_YQ1_Data(&YQFR_info,RxMessage.Data);
        break;
    }
//    case YQFR_EXTID2:
//    {
//        Get_YQ2_Data(&YQFR_info,RxMessage.Data);
//        break;
//    }
    /*�����������Ϣ*/
    case YQRL_EXTID1:
    {
        Get_YQ1_Data(&YQRL_info,RxMessage.Data);
        break;
    }
//    case YQRL_EXTID2:
//    {
//        Get_YQ2_Data(&YQRL_info,RxMessage.Data);
//        break;
//    }
    /*�����Һ�����Ϣ*/
    case YQRR_EXTID1:
    {
        Get_YQ1_Data(&YQRR_info,RxMessage.Data);
        break;
    }
//    case YQRR_EXTID2:
//    {
//        Get_YQ2_Data(&YQRR_info,RxMessage.Data);
//        break;
//    }
    default:
        break;
    }
}
#endif

/**
*********************************************************************
* @brief :Can_SendStdMsg
* @func  :���ͱ�׼֡����
* @prama :	msg-��Ҫ���͵�����ָ�룬len-�������ݳ��ȣ�stdID-��׼��ʶ��
* @retval:	0-�ɹ�������-ʧ��
********************************************************************
* @attention :
*********************************************************************
  */
u8 Can_SendStdMsg(u8 *msg,u8 len,u32 stdId)
{
    u8 mbox;
    u16 i=0;
    CanTxMsg 	TxMessage;

    TxMessage.StdId=stdId;				// ��׼��ʶ��
    TxMessage.ExtId=0x12;					// ������չ��ʾ��
    TxMessage.IDE=CAN_Id_Standard;// ��׼֡
    TxMessage.RTR=CAN_RTR_Data;		// ����֡
    TxMessage.DLC=len;						// Ҫ���͵����ݳ���
    for(i=0; i<len; i++)
        TxMessage.Data[i]=msg[i];

    mbox= CAN_Transmit(CAN1, &TxMessage);
    i=0;

    while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))
        i++;	//�ȴ����ͽ���

    if(i>=0XFFF)
        return 1;

    return 0;
}

/**
*********************************************************************
* @brief :Can_SendExtMsg
* @func  :������չ֡����
* @prama :msg-��Ҫ���͵�����ָ��
					len-�������ݵĳ���
					extID:��չ֡��ʶ��
* @retval: 0-�ɹ�������-ʧ��
********************************************************************
* @attention :
*********************************************************************
  */
 CanTxMsg 	TxMessage;
u8 Can_SendExtMsg(u8 *msg,u8 len,u32 extID)
{
    u8 mbox;
    u16 i=0;

	TxMessage.StdId = 0x12;
    TxMessage.ExtId = extID;				// ������չ��ʾ��
    TxMessage.IDE = CAN_Id_Extended;        //��չ֡
    TxMessage.RTR=CAN_RTR_Data;			    // ����֡
    TxMessage.DLC=len;						// Ҫ���͵����ݳ���

    for(i=0; i<len; i++)
        TxMessage.Data[i]=msg[i];

    mbox= CAN_Transmit(CAN1, &TxMessage);
    i=0;

    while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))
        i++;	//�ȴ����ͽ���

    if(i>=0XFFF)
        return 1;

    return 0;
}

//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;
//����ֵ:0,�����ݱ��յ�;
//����,���յ����ݳ���;

u8 Can_Receive_Msg(u8 *buf,u32 *stdId)
{
    u8 i;
	CanRxMsg RxMessage;

    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)
        return 0;		//û�н��յ�����,ֱ���˳�
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����
    *stdId = RxMessage.StdId;
    for(i=0; i<8; i++)
        buf[i]=RxMessage.Data[i];
    return RxMessage.DLC;
}
