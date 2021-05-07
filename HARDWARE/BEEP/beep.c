#include "beep.h"
#include "delay.h"

void BEEP_Init(void)
{
    GPIO_InitTypeDef		GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = BEEP_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(BEEP_PORT,&GPIO_InitStructure);
}

void Beep_Alarm(u8 times,u16 time)
{
    while(times--)
    {
        BEEP = !BEEP;
        delay_ms(time);
    }
}
