#include "jy01.h"
#include "can.h"

/**
*********************************************************************
* @brief :JY01_Ctrl
* @func  :控制居逸驱动器
* @prama :DriveSta-驾驶状态[1/2-手动,3-自动]
					RunSta-运行状态[1/2-驻车,3-运行]
					SonicSta-超声波状态[0-未检测到障碍物,1-检测到障碍物]
					Speed-后轮实时转速
					Alpha-前轮转角
* @retval:无  
********************************************************************
* @attention :后轮实时转速为两后轮平均转速
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
