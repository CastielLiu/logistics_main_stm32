#include "client.h"
#include "can.h"

/**
*********************************************************************
* @brief :Client_Ctrl
* @func  :控制下位机（从）
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
u8 state;
void Client_Ctrl(u8 DriveSta,u8 RunSta,u8 SonicSta,int Speed,int Alpha)
{
    u8 ClientCtrlMsg[8] = {0,0,0,0,0,0,0,0};


    state = DriveSta<<6;
    state = state|(RunSta<<4);
    state = state|(SonicSta<<2);

    ClientCtrlMsg[0] = state;

    ClientCtrlMsg[1] = (Speed>>8)&0xFF;
    ClientCtrlMsg[2] = Speed&0xFF;

    ClientCtrlMsg[3] = (Alpha>>8)&0xFF;
    ClientCtrlMsg[4] = Alpha&0xFF;

    Can_SendExtMsg(ClientCtrlMsg,8,CLIENT_EXTID0);
}

