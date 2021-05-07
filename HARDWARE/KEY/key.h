#ifndef	__KEY_H
#define __KEY_H

#include "sys.h"

#define SW_PORT		GPIOC
#define SW_PIN		GPIO_Pin_5

#define EN_PORT		GPIOC
#define EN_PIN		GPIO_Pin_4

#define EN_STATE 	PCin(4)
#define SW_STATE	PCin(5)

extern u8 SW_Status;
extern u8 EN_Status;

void Switch_Init(void);
void EN_Init(void);

#endif
