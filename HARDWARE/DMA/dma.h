#ifndef _DMA_H_
#define _DMA_H_

#include "sys.h"
#include "timer.h"

#define PC1 PCout(1) // PC1

void MYDMA_Config(DMA_Channel_TypeDef*DMA_CHx,u32 cpar,u32 cmar,u16 cndtr);//≈‰÷√DMA1_CHx
void MYDMA_Enable(DMA_Channel_TypeDef*DMA_CHx);// πƒ‹DMA1_CHx

float getVoltagePc1();

#endif
