#ifndef __CAN_H
#define __CAN_H	 

#include "sys.h"	
#include "common.h"


/*����Զ����������չID����*/
#define YQRL_EXTID0				0x10F8E3F3	//��λ��--->>>��������������λ������������
#define YQRL_EXTID1				0x10F8139A	//����������--->>>��λ����������������Ϣ����
#define YQRL_EXTID2				0x10F8138D	//����������--->>>��λ����������������Ϣ����

/*�Һ��Զ����������չID����*/
#define YQRR_EXTID0				0x10F8E4F3	//��λ��--->>>�Һ������������λ������������
#define YQRR_EXTID1				0x10F8149A	//�Һ��������--->>>��λ�����Һ����������Ϣ����
#define YQRR_EXTID2				0x10F8148D	//�Һ��������--->>>��λ�����Һ����������Ϣ����

/*��ǰ��Զ����������չID����*/
#define YQFL_EXTID0				0x10F8E5F3	//��λ��--->>>��ǰ������������λ������������
#define YQFL_EXTID1				0x10F8159A	//��ǰ��������--->>>��λ������ǰ����������Ϣ����
#define YQFL_EXTID2				0x10F8158D	//��ǰ��������--->>>��λ������ǰ����������Ϣ����

/*��ǰ��Զ����������չID����*/
#define YQFR_EXTID0				0x10F8E6F3	//��λ��--->>>��������������λ������������
#define YQFR_EXTID1				0x10F8169A	//��ǰ��������--->>>��λ����������������Ϣ����
#define YQFR_EXTID2				0x10F8168D	//��ǰ��������--->>>��λ����������������Ϣ����

/*��λ���ӻ���չID����*/
#define CLIENT_EXTID0			0x10F8E7F3	//��λ��(��)--->>>��λ��(��)����λ��(��)������λ��(��)
#define CLIENT_EXTID1			0x00F8179A	//��λ��(��)--->>>��λ��(��)����λ��(��)��Ϣ����

/*������������չID����*/
#define JY01_EXTID0				0x10F8E8F3	//��λ��--->>> ��������������λ�����ƾ���������
#define JY01_EXTID1				0x10F8189A	//����������--->>> ��λ����������������Ϣ����

//CAN����RX0�ж�ʹ��
#define CAN_RX0_INT_ENABLE	1		//0,��ʹ��;1,ʹ��.

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
										 							 				    
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);	//CAN��ʼ��
void Get_YQ_Data(YQ_info_t *YQ_info,u8 *buf);
u8 Can_SendStdMsg(u8* msg,u8 len,u32 stdId);	//ͨ����׼֡��������
u8 Can_SendExtMsg(u8* msg,u8 len,u32 extID);	//ͨ����չ֡��������
u8 Can_Receive_Msg(u8 *buf,u32 *stdId);			  //��������

#endif


