#include "client.h"
#include "can.h"

/**
*********************************************************************
* @brief :Client_Ctrl
* @func  :������λ�����ӣ�
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

