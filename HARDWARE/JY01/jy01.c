#include "jy01.h"
#include "can.h"

/**
*********************************************************************
* @brief :JY01_Ctrl
* @func  :���ƾ���������
* @prama :DriveSta-��ʻ״̬[1/2-�ֶ�,3-�Զ�]
					RunSta-����״̬[1/2-פ��,3-����]
					SonicSta-������״̬[0-δ��⵽�ϰ���,1-��⵽�ϰ���]
					Speed-����ʵʱת��
					Alpha-ǰ��ת��
* @retval:��  
********************************************************************
* @attention :����ʵʱת��Ϊ������ƽ��ת��
*********************************************************************
  */ 
void JY01_Ctrl(u8 DriveSta,u8 RunSta,u8 SonicSta,int targetTorque,int RealSpeed,int Alpha)
{
	u8 JY01ctrlMsg[8] = {0,0,0,0,0,0,0,0};
	u8 state;
	
	state = DriveSta<<6;
	state = state|(RunSta<<4);
	state = state|(SonicSta<<2);
	
	JY01ctrlMsg[0] = state;
	
	JY01ctrlMsg[1] = (targetTorque>>8)&0xFF;
	JY01ctrlMsg[2] = targetTorque&0xFF;
	
	if(targetTorque>=0)
	{
		JY01ctrlMsg[3] = (RealSpeed>>8)&0xFF;
		JY01ctrlMsg[4] = RealSpeed&0xFF;
	}
	else
	{
		JY01ctrlMsg[3] = (-RealSpeed>>8)&0xFF;
		JY01ctrlMsg[4] = (-RealSpeed)&0xFF;
	}
		
	JY01ctrlMsg[5] = (Alpha>>8)&0xFF;
	JY01ctrlMsg[6] = Alpha&0xFF;
	
	Can_SendExtMsg(JY01ctrlMsg,8,JY01_EXTID0);
}
