#include "can.h"
#include "led.h"
#include "delay.h"
#include "remoter.h"

float g_prm2speed = 3.1415926*0.355/60.0;  //Pi*D/60 转速r/min乘以该系数得到m/s  
YQ_info_t YQFL_info,YQFR_info,YQRL_info,YQRR_info;

//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024;  tq=(brp)*tpclk1
//波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
//Fpclk1的时钟在初始化的时候设置为36M,如果设置CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);
//则波特率为:36M/((8+9+1)*4)=500Kbps
//返回值:0,初始化OK;
//其他,初始化失败;
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
    GPIO_InitTypeDef 				GPIO_InitStructure;
    CAN_InitTypeDef        			CAN_InitStructure;
    CAN_FilterInitTypeDef  			CAN_FilterInitStructure;
#if CAN_RX0_INT_ENABLE
    NVIC_InitTypeDef  				NVIC_InitStructure;
#endif
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟

#if CAN1_REMAP2_PB89
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能PORTB时钟

	GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);

    //CAN_TX->PB.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽
    GPIO_Init(GPIOB, &GPIO_InitStructure);			//初始化IO
    //CAN_RX->PB.8
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//上拉输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);			//初始化IO

#else
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能PORTB时钟
	//CAN_TX->PA12
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽
    GPIO_Init(GPIOA, &GPIO_InitStructure);			//初始化IO
    //CAN_RX->PA11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//初始化IO

#endif

    //CAN单元设置
    CAN_InitStructure.CAN_TTCM=DISABLE;			//非时间触发通信模式
    CAN_InitStructure.CAN_ABOM=DISABLE;			//软件自动离线管理
    CAN_InitStructure.CAN_AWUM=ENABLE;			//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
    CAN_InitStructure.CAN_NART=ENABLE;			//允许报文自动传送
    CAN_InitStructure.CAN_RFLM=DISABLE;		 	//报文不锁定,新的覆盖旧的
    CAN_InitStructure.CAN_TXFP=ENABLE;			//优先级由报文标识符决定
    CAN_InitStructure.CAN_Mode= mode;	        //模式设置： mode:0,普通模式;1,回环模式;
    //设置波特率
    CAN_InitStructure.CAN_SJW=tsjw;				//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
    CAN_InitStructure.CAN_BS1=tbs1; 			//Tbs1=tbs1+1个时间单位CAN_BS1_1tq ~CAN_BS1_16tq
    CAN_InitStructure.CAN_BS2=tbs2;				//Tbs2=tbs2+1个时间单位CAN_BS2_1tq ~	CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler=brp;        //分频系数(Fdiv)为brp+1
    CAN_Init(CAN1, &CAN_InitStructure);        	//初始化CAN1

    CAN_FilterInitStructure.CAN_FilterNumber=0;	//过滤器0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 	//屏蔽位模式
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 	//32位宽
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;	//32位ID
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//激活过滤器0

    CAN_FilterInit(&CAN_FilterInitStructure);			//滤波器初始化

#if CAN_RX0_INT_ENABLE
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);				//FIFO0消息挂号中断允许.

    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     // 主优先级为2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;            // 次优先级为3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
    return 0;
}


/**
*********************************************************************
* @brief :Get_YQ1_Data
* @func  :解析后轮驱动器发来的数据帧1消息
* @prama :YQ_info-远驱消息解析结构体，buf-CAN接收数据组
* @retval:无
********************************************************************
* @attention :
*********************************************************************
  */
void Get_YQ1_Data(YQ_info_t *YQ_info,u8 *buf)
{
    YQ_info->Shift = buf[0];  //0 空挡 1前进 2后退 3低速
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
* @func  :解析后轮驱动器发来的数据帧2消息
* @prama :YQ_info-远驱消息解析结构体，buf-CAN接收数据组
* @retval:无
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

#if CAN_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数

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
    /*获取左前轮消息*/
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
    /*解析右前轮消息*/
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
    /*解析左后轮消息*/
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
    /*解析右后轮消息*/
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
* @func  :发送标准帧数据
* @prama :	msg-需要发送的数据指针，len-发送数据长度，stdID-标准标识符
* @retval:	0-成功，其他-失败
********************************************************************
* @attention :
*********************************************************************
  */
u8 Can_SendStdMsg(u8 *msg,u8 len,u32 stdId)
{
    u8 mbox;
    u16 i=0;
    CanTxMsg 	TxMessage;

    TxMessage.StdId=stdId;				// 标准标识符
    TxMessage.ExtId=0x12;					// 设置扩展标示符
    TxMessage.IDE=CAN_Id_Standard;// 标准帧
    TxMessage.RTR=CAN_RTR_Data;		// 数据帧
    TxMessage.DLC=len;						// 要发送的数据长度
    for(i=0; i<len; i++)
        TxMessage.Data[i]=msg[i];

    mbox= CAN_Transmit(CAN1, &TxMessage);
    i=0;

    while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))
        i++;	//等待发送结束

    if(i>=0XFFF)
        return 1;

    return 0;
}

/**
*********************************************************************
* @brief :Can_SendExtMsg
* @func  :发送扩展帧数据
* @prama :msg-需要发送的数据指针
					len-发送数据的长度
					extID:扩展帧标识符
* @retval: 0-成功，其他-失败
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
    TxMessage.ExtId = extID;				// 设置扩展标示符
    TxMessage.IDE = CAN_Id_Extended;        //扩展帧
    TxMessage.RTR=CAN_RTR_Data;			    // 数据帧
    TxMessage.DLC=len;						// 要发送的数据长度

    for(i=0; i<len; i++)
        TxMessage.Data[i]=msg[i];

    mbox= CAN_Transmit(CAN1, &TxMessage);
    i=0;

    while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))
        i++;	//等待发送结束

    if(i>=0XFFF)
        return 1;

    return 0;
}

//can口接收数据查询
//buf:数据缓存区;
//返回值:0,无数据被收到;
//其他,接收的数据长度;

u8 Can_Receive_Msg(u8 *buf,u32 *stdId)
{
    u8 i;
	CanRxMsg RxMessage;

    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)
        return 0;		//没有接收到数据,直接退出
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据
    *stdId = RxMessage.StdId;
    for(i=0; i<8; i++)
        buf[i]=RxMessage.Data[i];
    return RxMessage.DLC;
}
