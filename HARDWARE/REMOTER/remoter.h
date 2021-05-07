#ifndef __REMOTER_H
#define __REMOTER_H

#include "sys.h"
#include "stdio.h"

typedef __packed struct
{
    /* rocker channel information */
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    /* left and right lever information */
    uint8_t sw1;
    uint8_t sw2;
    uint8_t leftsw;	//自定义左拨动开关键值
    /* mouse movement and button information */
    __packed struct
    {
        int16_t x;
        int16_t y;
        int16_t z;

        uint8_t l;
        uint8_t r;
    } mouse;
    /* keyboard key information */
    __packed union
    {
        uint16_t key_code;
        __packed struct
        {
            uint16_t W:1;
            uint16_t S:1;
            uint16_t A:1;
            uint16_t D:1;
            uint16_t SHIFT:1;
            uint16_t CTRL:1;
            uint16_t Q:1;
            uint16_t E:1;
            uint16_t R:1;
            uint16_t F:1;
            uint16_t G:1;
            uint16_t Z:1;
            uint16_t X:1;
            uint16_t C:1;
            uint16_t V:1;
            uint16_t B:1;
        } bit;
    } kb;
} rc_info_t;

#define DBUS_BUFLEN	18

extern rc_info_t rc;
extern uint8_t  dbus_buf[DBUS_BUFLEN];

void Remoter_Uart_Init(u32 baudrate);
void Remoter_Uart_SendData(u8 *s,u8 len);
static void Get_DR16_Data(rc_info_t *rc,u8 *buff);

#endif
