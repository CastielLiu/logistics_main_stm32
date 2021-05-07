#include "remoter.h"
#include "stdlib.h"
#include "string.h"
#include "delay.h"
#include "led.h"

rc_info_t rc;
uint8_t   dbus_buf[DBUS_BUFLEN];

void Remoter_Uart_Init(u32 baudrate)
{
    //GPIO�˿�����
    GPIO_InitTypeDef 	GPIO_InitStructure;
    USART_InitTypeDef 	USART_InitStructure;
    NVIC_InitTypeDef 	NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);

    //Remote_TX -> UART5_TX -> PC.12
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //Remote_RX -> UART5_RX -> PD.2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    //USART ��ʼ������
    USART_InitStructure.USART_BaudRate = baudrate;//���ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_Even;//żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;	//����,����ģʽ
    USART_Init(UART5, &USART_InitStructure); //��ʼ������5

    USART_Cmd(UART5, ENABLE);                    //ʹ�ܴ���5

    //UART5 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;	//��ռ���ȼ�2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

    USART_ClearFlag(UART5,USART_FLAG_RXNE);
    USART_ITConfig(UART5,USART_IT_RXNE,ENABLE);	//��������5�����ж�
    USART_ClearFlag(UART5,USART_FLAG_IDLE);
    USART_ITConfig(UART5,USART_IT_IDLE,ENABLE);	//��������5�����ж�
}


void Remoter_Uart_SendData(u8 *s,u8 len)
{
    while(len--)
    {
        while(USART_GetFlagStatus(UART5,USART_FLAG_TC )==RESET);
        USART_SendData(UART5,*s);
        s++;
    }
}

static void Get_DR16_Data(rc_info_t *rc, uint8_t *buff)
{

    //satori��������ɵ������ݵķ����ƴ�ӣ���ȥ1024��Ϊ�������ݵ��м�ֵ��Ϊ0
    rc->ch1 = (buff[0]|buff[1]<<8)&0x07FF;
    rc->ch1 -= 1024;
    rc->ch2 = (buff[1]>>3|buff[2]<<5)&0x07FF;
    rc->ch2 -= 1024;
    rc->ch3 = (buff[2]>>6|buff[3]<<2|buff[4]<<10)&0x07FF;
    rc->ch3 -= 1024;
    rc->ch4 = (buff[4]>>1|buff[5]<<7)&0x07FF;
    rc->ch4 -= 1024;

    //satori:��ֹ������Ư����������5������
    /* prevent remote control zero deviation */
    if(rc->ch1 <= 5 && rc->ch1 >= -5)
        rc->ch1 = 0;
    if(rc->ch2 <= 5 && rc->ch2 >= -5)
        rc->ch2 = 0;
    if(rc->ch3 <= 5 && rc->ch3 >= -5)
        rc->ch3 = 0;
    if(rc->ch4 <= 5 && rc->ch4 >= -5)
        rc->ch4 = 0;

    rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->sw2 = (buff[5] >> 4) & 0x0003;

    /*�Զ��岦�����ؼ�ֵ*/
    switch(rc->sw1)
    {
    case 3:
    {
        rc->leftsw = 1;
        break;
    }
    case 1:
    case 2:
    {
        rc->leftsw = 0;
        break;
    }
    default:
        break;
    }

    //satori:��ֹ�������
    if ((abs(rc->ch1) > 660) || \
            (abs(rc->ch2) > 660) || \
            (abs(rc->ch3) > 660) || \
            (abs(rc->ch4) > 660))
    {
        memset(rc, 0, sizeof(rc_info_t));
        return ;
    }

    rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
    rc->mouse.y = buff[8] | (buff[9] << 8);
    rc->mouse.z = buff[10] | (buff[11] << 8);

    rc->mouse.l = buff[12];
    rc->mouse.r = buff[13];

    rc->kb.key_code = buff[14] | buff[15] << 8; // key borad code
}


u8 Remoter_Rx;
void UART5_IRQHandler(void)
{
    if(USART_GetITStatus(UART5,USART_IT_RXNE)!=RESET)		//����������5�����ж�
    {
        USART_ClearITPendingBit(UART5,USART_IT_RXNE);		//��������жϱ�־

        dbus_buf[Remoter_Rx] = USART_ReceiveData(UART5);	//���մ���5���ݵ�buff������
        Remoter_Rx ++;
    }
    if(USART_GetITStatus(UART5,USART_IT_IDLE)!=RESET)
    {
        (void)UART5->SR;
        (void)UART5->DR;

        if(Remoter_Rx==18)
            Get_DR16_Data(&rc,dbus_buf);
        Remoter_Rx = 0;
    }
}

