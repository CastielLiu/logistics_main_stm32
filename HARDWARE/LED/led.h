#ifndef _LED_H_
#define _LED_H_
#include "sys.h"

#define MCU_Light PAout(11)
#define MCU_Light2 PBout(5)
#define MCU_Light3 PEout(5)

void LED_Init(void);

#endif
