#ifndef	__BEEP_H
#define __BEEP_H

#include "sys.h"

#define BEEP_PORT		GPIOB
#define BEEP_PIN		GPIO_Pin_5

#define BEEP			PBout(5)

void BEEP_Init(void);
void Beep_Alarm(u8 times,u16 time);

#endif
