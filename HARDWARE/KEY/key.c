#include "key.h"
#include "remoter.h"

void Switch_Init(void)
{
    GPIO_InitTypeDef	GPIO_Inittructure;

    GPIO_Inittructure.GPIO_Pin = SW_PIN;
    GPIO_Inittructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Inittructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(SW_PORT,&GPIO_Inittructure);
}

void EN_Init(void)
{
    GPIO_InitTypeDef	GPIO_Inittructure;

    GPIO_Inittructure.GPIO_Pin = EN_PIN;
    GPIO_Inittructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Inittructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(EN_PORT,&GPIO_Inittructure);
}
