#ifndef __INDUSTRY_H
#define __INDUSTRY_H

#include "sys.h"
#include "pid.h"


typedef struct{
	u8  EN;
	float TargetSpeed;
	float TargetAngle;
	u8  BrakeSig;
	u8  SumCheck;
}Industry_info_t;

extern Industry_info_t	Industry_info;

#define Industry_Buf_Len	    11
#define Industry_Buf_MaxLen		50

void Industry_Init(u32 baudrate);
void Industry_SendData(u8 *s,u8 len);
void Get_Industry_Data(Industry_info_t	*Industry_info,u8 *buf, int len);
void Industry_Reply(s16 LrRealRPM,s16 RrRealRPM,s16 Angle,s16 targetTorque);
u8 generateCheckVal(u8* buf, int len);
#endif
