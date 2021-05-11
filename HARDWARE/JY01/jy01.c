#include "jy01.h"
#include "can.h"

/**
*********************************************************************
* @brief :JY01_Ctrl
* @func  :���ƾ���������
* @prama :DriveSta-��ʻ״̬[1/2-�ֶ�,3-�Զ�]
					RunSta-����״̬[1/2-פ��,3-����]
					SonicSta-������״̬[0-δ��⵽�ϰ���,1-��⵽�ϰ���]
					expectBrakeVal �������ƶ�����
					Speed-����ʵʱת��
					Alpha-ǰ��ת��
* @retval:��  
********************************************************************
* @attention :����ʵʱת��Ϊ������ƽ��ת��
*********************************************************************
  */
 u8 db_expectBrakeVal;
 u8 db_send_can_ok;
void JY01_Ctrl(u8 DriveSta,u8 RunSta,u8 SonicSta,u8 expectBrakeVal,int RealSpeed,int Alpha)
{
	u8 JY01ctrlMsg[8] = {0,0,0,0,0,0,0,0};
	u8 state;
	
	state = DriveSta<<6;
	state = state|(RunSta<<4);
	state = state|(SonicSta<<2);
	
	JY01ctrlMsg[0] = state;
	
	db_expectBrakeVal = expectBrakeVal;
	JY01ctrlMsg[1] = expectBrakeVal;
		
	JY01ctrlMsg[5] = (Alpha>>8)&0xFF;
	JY01ctrlMsg[6] = Alpha&0xFF;
	
	db_send_can_ok = Can_SendExtMsg(JY01ctrlMsg,8,JY01_EXTID0);
}
