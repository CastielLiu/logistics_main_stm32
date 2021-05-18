#ifndef _LED_H_
#define _LED_H_
#include "sys.h"
#include "common.h"

#if LED_PB5PE5
	#define MCU_Light PBout(5)
#else
	#define MCU_Light PAout(11)
#endif

void LED_Init(void);

#endif
