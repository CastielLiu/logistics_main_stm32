#ifndef __CAN_H
#define __CAN_H	 

#include "sys.h"	
#include "common.h"


/*左后侧远驱驱动器扩展ID定义*/
#define YQRL_EXTID0				0x10F8E3F3	//下位机--->>>左后侧驱动器。下位机控制驱动器
#define YQRL_EXTID1				0x10F8139A	//左后侧驱动器--->>>下位机，左后侧驱动器消息反馈
#define YQRL_EXTID2				0x10F8138D	//左后侧驱动器--->>>下位机，左后侧驱动器消息反馈

/*右后侧远驱驱动器扩展ID定义*/
#define YQRR_EXTID0				0x10F8E4F3	//下位机--->>>右后侧驱动器。下位机控制驱动器
#define YQRR_EXTID1				0x10F8149A	//右后侧驱动器--->>>下位机，右后侧驱动器消息反馈
#define YQRR_EXTID2				0x10F8148D	//右后侧驱动器--->>>下位机，右后侧驱动器消息反馈

/*左前侧远驱驱动器扩展ID定义*/
#define YQFL_EXTID0				0x10F8E5F3	//下位机--->>>左前侧驱动器。下位机控制驱动器
#define YQFL_EXTID1				0x10F8159A	//左前侧驱动器--->>>下位机，左前侧驱动器消息反馈
#define YQFL_EXTID2				0x10F8158D	//左前侧驱动器--->>>下位机，左前侧驱动器消息反馈

/*右前侧远驱驱动器扩展ID定义*/
#define YQFR_EXTID0				0x10F8E6F3	//下位机--->>>左后侧驱动器。下位机控制驱动器
#define YQFR_EXTID1				0x10F8169A	//右前侧驱动器--->>>下位机，左后侧驱动器消息反馈
#define YQFR_EXTID2				0x10F8168D	//右前侧驱动器--->>>下位机，左后侧驱动器消息反馈

/*下位机从机扩展ID定义*/
#define CLIENT_EXTID0			0x10F8E7F3	//下位机(主)--->>>下位机(从)，下位机(主)控制下位机(从)
#define CLIENT_EXTID1			0x00F8179A	//下位机(从)--->>>下位机(主)，下位机(从)消息反馈

/*居逸驱动器扩展ID定义*/
#define JY01_EXTID0				0x10F8E8F3	//下位机--->>> 居逸驱动器，下位机控制居逸驱动器
#define JY01_EXTID1				0x10F8189A	//居逸驱动器--->>> 下位机，居逸驱动器消息反馈

//CAN接收RX0中断使能
#define CAN_RX0_INT_ENABLE	1		//0,不使能;1,使能.

typedef struct{
	u8  Shift;
	u16 Throttle;
	u8  WorkState;
	s16 RPM;
	float speed; //m/s
	
	u8  MotorTemp;
	u8  CtrlTemp;
	
	u16 VBusVol;
	u16 MotorCur;
	u16 Lap;
	u16 Fault;
}YQ_info_t;

extern YQ_info_t YQFL_info,YQFR_info,YQRL_info,YQRR_info;
										 							 				    
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);	//CAN初始化
void Get_YQ_Data(YQ_info_t *YQ_info,u8 *buf);
u8 Can_SendStdMsg(u8* msg,u8 len,u32 stdId);	//通过标准帧发送数据
u8 Can_SendExtMsg(u8* msg,u8 len,u32 extID);	//通过扩展帧发送数据
u8 Can_Receive_Msg(u8 *buf,u32 *stdId);			  //接收数据

#endif


