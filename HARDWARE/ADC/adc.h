#ifndef __PEDAL_H
#define __PEDAL_H

#include "sys.h"

//ADC数据寄存器地址
#define   ADC1_DR_Address    ((uint32_t)0x4001244C)

/* 48V电压测量 -> PC.0 -> ADC1_IN10 */
#define VOL_PORT			GPIOC
#define VOL_PIN				GPIO_Pin_0
#define VOL_CH				ADC_Channel_10

/* 备用1  -> PC.1 -> ADC1_IN11 */
#define RESERVE1_PORT		GPIOC
#define RESERVE1_PIN		GPIO_Pin_1
#define RESERVE1_CH			ADC_Channel_11

/* 备用2 -> PC.2 -> ADC1_IN12 */
#define RESERVE2_PORT		GPIOC
#define RESERVE2_PIN		GPIO_Pin_2
#define RESERVE2_CH			ADC_Channel_12

/* 备用3 -> PC.3 -> ADC1_IN13 */
#define RESERVE3_PORT		GPIOC
#define RESERVE3_PIN		GPIO_Pin_3
#define RESERVE3_CH			ADC_Channel_13

extern float h_Voltage;
extern u16 h_ADC_PC1;
extern u16 h_ADC_PC2;
extern u16 h_ADC_PC3;

u16 Get_Adc_Average(u8 ch,u8 times);
u16 Get_Adc(u8 ch);
void Adc_Init(void);

#endif

