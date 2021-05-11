#include "yq.h"
#include "can.h"

/*
*********************************************************************
* @ name :YQFL_TRctrl
* @ func :左前电机油门控制
* @ prama: RunSta-运行状态[3-行驶,其他-刹车]
********** SonicSta-超声波状态[0-正常,1-报警]
********** TR-油门开度(-256~256)正值前进，负值后退
* @retval: 无
********************************************************************
* @attention:档位[0-空档,1-前进档,2-后退档,3-低速档]
*********************************************************************
*/
void YQFL_TRctrl(u8 RunSta,u8 SonicSta,s16 TR)
{
    u8 CtrlMsg[8] = {0,0,0,0,0,0,0,0};
		
		/*油门限幅*/
		if(TR>MAX_TR)
			TR = MAX_TR;
		if(TR<-MAX_TR)
			TR = -MAX_TR;

    /*运行状态*/
    if(RunSta==3)
    {
        /*超声波正常*/
        if(SonicSta==0)
        {
            if(TR>0)//电机前进
            {
                CtrlMsg[0] = 3;
                CtrlMsg[1] = TR&0xFF;
                CtrlMsg[2] = (TR>>8)&0xFF;
                CtrlMsg[3] = 0x20;
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
                CtrlMsg[6] = 0;
                CtrlMsg[7] = 0;
            }
            else if(TR<0)//电机后退
            {
                CtrlMsg[0] = 2;
                CtrlMsg[1] = (-TR)&0xFF;
                CtrlMsg[2] = (-TR>>8)&0xFF;
                CtrlMsg[3] = 0x20;
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
                CtrlMsg[6] = 0;
                CtrlMsg[7] = 0;
            }
            else//电机制动
            {
                CtrlMsg[0] = 3;
                CtrlMsg[1] = 0;
                CtrlMsg[2] = 0;
                CtrlMsg[3] = 0x60;
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
                CtrlMsg[6] = 0x10;
                CtrlMsg[7] = 0xFF;
            }
        }
        /*超声波报警*/
        else
        {
            CtrlMsg[0] = 3;
            CtrlMsg[1] = 0;
            CtrlMsg[2] = 0;
            CtrlMsg[3] = 0x60;
            CtrlMsg[4] = 0x00;
            CtrlMsg[5] = 0x00;
            CtrlMsg[6] = 0x10;
            CtrlMsg[7] = 0xFF;
        }
    }
    /*刹车状态*/
    else
    {
        CtrlMsg[0] = 3;
        CtrlMsg[1] = 0;
        CtrlMsg[2] = 0;
        CtrlMsg[3] = 0x60;
        CtrlMsg[4] = 0x00;
        CtrlMsg[5] = 0x00;
        CtrlMsg[6] = 0x10;
        CtrlMsg[7] = 0xFF;
    }
    Can_SendExtMsg(CtrlMsg,8,YQFL_EXTID0);
}

/*
*********************************************************************
* @ name :YQFR_TRctrl
* @ func :右前电机油门控制
* @ prama: RunSta-运行状态[3-行驶,其他-刹车]
********** SonicSta-超声波状态[0-正常,1-报警]
********** TR-油门开度(-256~256)正值前进，负值后退
* @retval: 无
********************************************************************
* @attention:档位[0-空档,1-前进档,2-后退档,3-低速档]
*********************************************************************
*/
void YQFR_TRctrl(u8 RunSta,u8 SonicSta,s16 TR)
{
    u8 CtrlMsg[8] = {0,0,0,0,0,0,0,0};
		
		/*油门限幅*/
		if(TR>MAX_TR)
			TR = MAX_TR;
		if(TR<-MAX_TR)
			TR = -MAX_TR;

    /*运行状态*/
    if(RunSta==3)
    {
        /*超声波正常*/
        if(SonicSta==0)
        {
            if(TR>0)//电机前进
            {
                CtrlMsg[0] = 3;
                CtrlMsg[1] = (TR)&0xFF;
                CtrlMsg[2] = (TR>>8)&0xFF;
                CtrlMsg[3] = 0x28;
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
                CtrlMsg[6] = 0;
                CtrlMsg[7] = 0;
            }
            else if(TR<0)//电机后退
            {
                CtrlMsg[0] = 2;
                CtrlMsg[1] = (-TR)&0xFF;
                CtrlMsg[2] = (-TR>>8)&0xFF;
                CtrlMsg[3] = 0x28;
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
                CtrlMsg[6] = 0;
                CtrlMsg[7] = 0;
            }
            else//电机制动
            {
                CtrlMsg[0] = 3;
                CtrlMsg[1] = 0;
                CtrlMsg[2] = 0;
                CtrlMsg[3] = 0x68;
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
                CtrlMsg[6] = 0x10;
                CtrlMsg[7] = 0xFF;
            }
        }
        /*超声波报警*/
        else
        {
            CtrlMsg[0] = 3;
            CtrlMsg[1] = 0;
            CtrlMsg[2] = 0;
            CtrlMsg[3] = 0x68;
            CtrlMsg[4] = 0x00;
            CtrlMsg[5] = 0x00;
            CtrlMsg[6] = 0x10;
            CtrlMsg[7] = 0xFF;
        }
    }
    /*刹车状态*/
    else
    {
        CtrlMsg[0] = 3;
        CtrlMsg[1] = 0;
        CtrlMsg[2] = 0;
        CtrlMsg[3] = 0x68;
        CtrlMsg[4] = 0x00;
        CtrlMsg[5] = 0x00;
        CtrlMsg[6] = 0x10;
        CtrlMsg[7] = 0xFF;
    }
    Can_SendExtMsg(CtrlMsg,8,YQFR_EXTID0);
}

/*
*********************************************************************
* @ name :YQRL_TRctrl
* @ func :左后电机油门控制
* @ prama: RunSta-运行状态[3-行驶,其他-刹车]
********** SonicSta-超声波状态[0-正常,1-报警]
********** TR-油门开度(-256~256)正值前进，负值后退
********** expectSpeedDir 当前期望车辆速度方向, 如果车辆正在向前行驶，但需要制动，此时期望的速度方向仍为正！
* @retval: 无
********************************************************************
* @attention:档位[0-空档,1-前进档,2-后退档,3-低速档]
*********************************************************************
*/

s8 g_expectSpeedDir;
s16 g_TR;
void YQRL_TRctrl(u8 RunSta,u8 SonicSta,s16 TR, s8 expectSpeedDir)
{
	u8 CtrlMsg[8] = {0,0,0,0,0,0,0,0};
	static u16 maxBrakeVal = 60 * 0.25; //60A * 0.25A/bit

	u16 u16_brakeVal = 0;
	s16 s16_brakeVal = 0;
	u8  brakeTorque = 0;
	
	/*油门限幅*/
	if(TR>MAX_TR)
		TR = MAX_TR;
	if(TR<-MAX_TR)
		TR = -MAX_TR;
	g_expectSpeedDir =  expectSpeedDir;
	g_TR = TR;
	if(expectSpeedDir * TR < 0 ||  // 期望速度方向与期望扭矩方向不一致时，表明需要进行制动操作
	   expectSpeedDir == 0)	       // 当期望速度为0时，无论如何都需要制动
	{
		brakeTorque = fabs(TR);
		u16_brakeVal = 1.0*maxBrakeVal*brakeTorque/255;
		s16_brakeVal = -u16_brakeVal;
		TR = 0.0; //将驱动扭矩置0
	}

    /*运行状态*/
    if(RunSta==3)
    {
        /*超声波正常*/
        if(SonicSta==0)
        {
            if(TR>0)//电机前进
            {							
                CtrlMsg[0] = 3;
                CtrlMsg[1] = (TR)&0xFF;
                CtrlMsg[2] = (TR>>8)&0xFF;
                CtrlMsg[3] = 0x20;   //0b0010 0000 油门扭矩控制
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
                CtrlMsg[6] = 0;
                CtrlMsg[7] = 0;
            }
            else if(TR<0)//电机后退
            {							
                CtrlMsg[0] = 2;
                CtrlMsg[1] = (-TR)&0xFF;
                CtrlMsg[2] = (-TR>>8)&0xFF;
                CtrlMsg[3] = 0x20;
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
                CtrlMsg[6] = 0;
                CtrlMsg[7] = 0;
            }
            else//电机制动
            {
				//此处应判断期望的速度方向，并据此设置对应的档位
				//因为速度反馈值为标量，只能通过档位值判断其方向
				//如果车辆正在向前行驶，但档位值置反，解析得到的速度将为负数，导致错误
				if(expectSpeedDir == 1)
					CtrlMsg[0] = 3; //前进挡
				else
					CtrlMsg[0] = 2; //

                CtrlMsg[1] = 0;
                CtrlMsg[2] = 0;
                CtrlMsg[3] = 0x60;   //0x40 刹车 0x20 油门扭矩控制
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
				
				//制动电流 低-高
				CtrlMsg[6] = s16_brakeVal;
                CtrlMsg[7] = s16_brakeVal >> 8;
            }
        }
        /*超声波报警*/
        else
        {
            CtrlMsg[0] = 3;
            CtrlMsg[1] = 0;
            CtrlMsg[2] = 0;
            CtrlMsg[3] = 0x60;
            CtrlMsg[4] = 0x00;
            CtrlMsg[5] = 0x00;
            CtrlMsg[6] = 0x10;
            CtrlMsg[7] = 0xFF;
        }
    }
    /*刹车状态*/
    else
    {
        CtrlMsg[0] = 3;
        CtrlMsg[1] = 0;
        CtrlMsg[2] = 0;
        CtrlMsg[3] = 0x60;
        CtrlMsg[4] = 0x00;
        CtrlMsg[5] = 0x00;
        CtrlMsg[6] = 0x10;
        CtrlMsg[7] = 0xFF;
    }
    Can_SendExtMsg(CtrlMsg,8,YQRL_EXTID0);
}

/*
*********************************************************************
* @ name :YQRR_TRctrl
* @ func :右后电机油门控制
* @ prama: RunSta-运行状态[3-行驶,其他-刹车]
********** SonicSta-超声波状态[0-正常,1-报警]
********** TR-油门开度(-256~256)正值前进，负值后退
* @retval: 无
********************************************************************
* @attention:档位[0-空档,1-前进档,2-后退档,3-低速档]
*********************************************************************
*/
void YQRR_TRctrl(u8 RunSta,u8 SonicSta,s16 TR, s8 expectSpeedDir)
{
    u8 CtrlMsg[8] = {0,0,0,0,0,0,0,0};
	
	static u16 maxBrakeVal = 60 * 0.25; //60A * 0.25A/bit

	u16 u16_brakeVal = 0;
	s16 s16_brakeVal = 0;
	u8  brakeTorque = 0;
	
	/*油门限幅*/
	if(TR>MAX_TR)
		TR = MAX_TR;
	if(TR<-MAX_TR)
		TR = -MAX_TR;
	
	if(expectSpeedDir * TR < 0 ||  // 期望速度方向与期望扭矩方向不一致时，表明需要进行制动操作
	   expectSpeedDir == 0)	       // 当期望速度为0时，无论如何都需要制动
	{
		brakeTorque = fabs(TR);
		u16_brakeVal = 1.0*maxBrakeVal*brakeTorque/255;
		s16_brakeVal = -u16_brakeVal;
		TR = 0; //将驱动扭矩置0
	}

    /*运行状态*/
    if(RunSta==3)
    {
        /*超声波正常*/
        if(SonicSta==0)
        {
            if(TR>0)//电机前进
            {							
                CtrlMsg[0] = 3;
                CtrlMsg[1] = (TR)&0xFF;
                CtrlMsg[2] = (TR>>8)&0xFF;
                CtrlMsg[3] = 0x28;
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
                CtrlMsg[6] = 0;
                CtrlMsg[7] = 0;
            }
            else if(TR<0)//电机后退
            {							
                CtrlMsg[0] = 2;
                CtrlMsg[1] = (-TR)&0xFF;
                CtrlMsg[2] = (-TR>>8)&0xFF;
                CtrlMsg[3] = 0x28;
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
                CtrlMsg[6] = 0;
                CtrlMsg[7] = 0;
            }
            else//电机制动
            {
				//此处应判断期望的速度方向，并据此设置对应的档位
				//因为速度反馈值为标量，只能通过档位值判断其方向
				//如果车辆正在向前行驶，但档位值置反，解析得到的速度将为负数，导致错误
				if(expectSpeedDir == 1)
					CtrlMsg[0] = 3; //前进挡
				else
					CtrlMsg[0] = 2; //后退档
				
                CtrlMsg[1] = 0;
                CtrlMsg[2] = 0;
                CtrlMsg[3] = 0x68;
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
				
                //制动电流 低-高
				CtrlMsg[6] = s16_brakeVal;
                CtrlMsg[7] = s16_brakeVal >> 8;
            }
        }
        /*超声波报警*/
        else
        {
            CtrlMsg[0] = 3;
            CtrlMsg[1] = 0;
            CtrlMsg[2] = 0;
            CtrlMsg[3] = 0x68;
            CtrlMsg[4] = 0x00;
            CtrlMsg[5] = 0x00;
            CtrlMsg[6] = 0x10;
            CtrlMsg[7] = 0xFF;
        }
    }
    /*刹车状态*/
    else
    {
        CtrlMsg[0] = 3;
        CtrlMsg[1] = 0;
        CtrlMsg[2] = 0;
        CtrlMsg[3] = 0x68;
        CtrlMsg[4] = 0x00;
        CtrlMsg[5] = 0x00;
        CtrlMsg[6] = 0x10;
        CtrlMsg[7] = 0xFF;
    }
    Can_SendExtMsg(CtrlMsg,8,YQRR_EXTID0);
}
