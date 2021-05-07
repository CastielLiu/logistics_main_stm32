#include "sys.h"
#include "usart.h"


//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
    int handle;
};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
_sys_exit(int x)
{
    x = x;
}
//重定义fputc函数
int fputc(int ch, FILE *f)
{
    while((UART5->SR&0X40)==0);//循环发送,直到发送完毕
    UART5->DR = (u8) ch;
    return ch;
}

#endif

void Debug_SendData(u8 *s,u8 len)
{
    while(len--)
    {
        while(USART_GetFlagStatus(UART5,USART_FLAG_TC )==RESET);
        USART_SendData(UART5,*s);
        s++;
    }
}

//注意,读取USARTx->SR能避免莫名其妙的错误

