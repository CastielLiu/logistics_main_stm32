#ifndef	__YQ_H
#define __YQ_H

#include "sys.h"
#include "math.h"

#define MAX_TR		256

/*ÓÍÃÅ×ª¾Ø¿ØÖÆ*/
void YQFL_TRctrl(u8 RunSta,u8 SonicSta,s16 TR);
void YQFR_TRctrl(u8 RunSta,u8 SonicSta,s16 TR);


void YQRL_TRctrl(u8 RunSta,u8 SonicSta,s16 TR, s8 CurrentDir);
void YQRR_TRctrl(u8 RunSta,u8 SonicSta,s16 TR, s8 CurrentDir);


#endif
