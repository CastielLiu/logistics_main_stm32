#include "jy01.h"
#include "can.h"

/**
*********************************************************************
* @brief :JY01_Ctrl
* @func  :控制居逸驱动器
* @prama :DriveSta-驾驶状态[1/2-手动,3-自动]
					RunSta-运行状态[1/2-驻车,3-运行]
					SonicSta-超声波状态[0-未检测到障碍物,1-检测到障碍物]
					expectBrakeVal 期望的制动力矩
					Speed-后轮实时转速
					Alpha-前轮转角
* @retval:无  
********************************************************************
* @attention :后轮实时转速为两后轮平均转速
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
