#include "sys.h"
#include "usart.h"


//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//��׼����Ҫ��֧�ֺ���
struct __FILE
{
    int handle;
};

FILE __stdout;
//����_sys_exit()�Ա���ʹ�ð�����ģʽ
_sys_exit(int x)
{
    x = x;
}
//�ض���fputc����
int fputc(int ch, FILE *f)
{
    while((UART5->SR&0X40)==0);//ѭ������,ֱ���������
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

//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���

