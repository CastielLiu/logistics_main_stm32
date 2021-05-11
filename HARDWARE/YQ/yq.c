#include "yq.h"
#include "can.h"

/*
*********************************************************************
* @ name :YQFL_TRctrl
* @ func :��ǰ������ſ���
* @ prama: RunSta-����״̬[3-��ʻ,����-ɲ��]
********** SonicSta-������״̬[0-����,1-����]
********** TR-���ſ���(-256~256)��ֵǰ������ֵ����
* @retval: ��
********************************************************************
* @attention:��λ[0-�յ�,1-ǰ����,2-���˵�,3-���ٵ�]
*********************************************************************
*/
void YQFL_TRctrl(u8 RunSta,u8 SonicSta,s16 TR)
{
    u8 CtrlMsg[8] = {0,0,0,0,0,0,0,0};
		
		/*�����޷�*/
		if(TR>MAX_TR)
			TR = MAX_TR;
		if(TR<-MAX_TR)
			TR = -MAX_TR;

    /*����״̬*/
    if(RunSta==3)
    {
        /*����������*/
        if(SonicSta==0)
        {
            if(TR>0)//���ǰ��
            {
                CtrlMsg[0] = 3;
                CtrlMsg[1] = TR&0xFF;
                CtrlMsg[2] = (TR>>8)&0xFF;
                CtrlMsg[3] = 0x20;
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
                CtrlMsg[6] = 0;
                CtrlMsg[7] = 0;
            }
            else if(TR<0)//�������
            {
                CtrlMsg[0] = 2;
                CtrlMsg[1] = (-TR)&0xFF;
                CtrlMsg[2] = (-TR>>8)&0xFF;
                CtrlMsg[3] = 0x20;
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
                CtrlMsg[6] = 0;
                CtrlMsg[7] = 0;
            }
            else//����ƶ�
            {
                CtrlMsg[0] = 3;
                CtrlMsg[1] = 0;
                CtrlMsg[2] = 0;
                CtrlMsg[3] = 0x60;
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
                CtrlMsg[6] = 0x10;
                CtrlMsg[7] = 0xFF;
            }
        }
        /*����������*/
        else
        {
            CtrlMsg[0] = 3;
            CtrlMsg[1] = 0;
            CtrlMsg[2] = 0;
            CtrlMsg[3] = 0x60;
            CtrlMsg[4] = 0x00;
            CtrlMsg[5] = 0x00;
            CtrlMsg[6] = 0x10;
            CtrlMsg[7] = 0xFF;
        }
    }
    /*ɲ��״̬*/
    else
    {
        CtrlMsg[0] = 3;
        CtrlMsg[1] = 0;
        CtrlMsg[2] = 0;
        CtrlMsg[3] = 0x60;
        CtrlMsg[4] = 0x00;
        CtrlMsg[5] = 0x00;
        CtrlMsg[6] = 0x10;
        CtrlMsg[7] = 0xFF;
    }
    Can_SendExtMsg(CtrlMsg,8,YQFL_EXTID0);
}

/*
*********************************************************************
* @ name :YQFR_TRctrl
* @ func :��ǰ������ſ���
* @ prama: RunSta-����״̬[3-��ʻ,����-ɲ��]
********** SonicSta-������״̬[0-����,1-����]
********** TR-���ſ���(-256~256)��ֵǰ������ֵ����
* @retval: ��
********************************************************************
* @attention:��λ[0-�յ�,1-ǰ����,2-���˵�,3-���ٵ�]
*********************************************************************
*/
void YQFR_TRctrl(u8 RunSta,u8 SonicSta,s16 TR)
{
    u8 CtrlMsg[8] = {0,0,0,0,0,0,0,0};
		
		/*�����޷�*/
		if(TR>MAX_TR)
			TR = MAX_TR;
		if(TR<-MAX_TR)
			TR = -MAX_TR;

    /*����״̬*/
    if(RunSta==3)
    {
        /*����������*/
        if(SonicSta==0)
        {
            if(TR>0)//���ǰ��
            {
                CtrlMsg[0] = 3;
                CtrlMsg[1] = (TR)&0xFF;
                CtrlMsg[2] = (TR>>8)&0xFF;
                CtrlMsg[3] = 0x28;
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
                CtrlMsg[6] = 0;
                CtrlMsg[7] = 0;
            }
            else if(TR<0)//�������
            {
                CtrlMsg[0] = 2;
                CtrlMsg[1] = (-TR)&0xFF;
                CtrlMsg[2] = (-TR>>8)&0xFF;
                CtrlMsg[3] = 0x28;
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
                CtrlMsg[6] = 0;
                CtrlMsg[7] = 0;
            }
            else//����ƶ�
            {
                CtrlMsg[0] = 3;
                CtrlMsg[1] = 0;
                CtrlMsg[2] = 0;
                CtrlMsg[3] = 0x68;
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
                CtrlMsg[6] = 0x10;
                CtrlMsg[7] = 0xFF;
            }
        }
        /*����������*/
        else
        {
            CtrlMsg[0] = 3;
            CtrlMsg[1] = 0;
            CtrlMsg[2] = 0;
            CtrlMsg[3] = 0x68;
            CtrlMsg[4] = 0x00;
            CtrlMsg[5] = 0x00;
            CtrlMsg[6] = 0x10;
            CtrlMsg[7] = 0xFF;
        }
    }
    /*ɲ��״̬*/
    else
    {
        CtrlMsg[0] = 3;
        CtrlMsg[1] = 0;
        CtrlMsg[2] = 0;
        CtrlMsg[3] = 0x68;
        CtrlMsg[4] = 0x00;
        CtrlMsg[5] = 0x00;
        CtrlMsg[6] = 0x10;
        CtrlMsg[7] = 0xFF;
    }
    Can_SendExtMsg(CtrlMsg,8,YQFR_EXTID0);
}

/*
*********************************************************************
* @ name :YQRL_TRctrl
* @ func :��������ſ���
* @ prama: RunSta-����״̬[3-��ʻ,����-ɲ��]
********** SonicSta-������״̬[0-����,1-����]
********** TR-���ſ���(-256~256)��ֵǰ������ֵ����
********** expectSpeedDir ��ǰ���������ٶȷ���, �������������ǰ��ʻ������Ҫ�ƶ�����ʱ�������ٶȷ�����Ϊ����
* @retval: ��
********************************************************************
* @attention:��λ[0-�յ�,1-ǰ����,2-���˵�,3-���ٵ�]
*********************************************************************
*/

s8 g_expectSpeedDir;
s16 g_TR;
void YQRL_TRctrl(u8 RunSta,u8 SonicSta,s16 TR, s8 expectSpeedDir)
{
	u8 CtrlMsg[8] = {0,0,0,0,0,0,0,0};
	static u16 maxBrakeVal = 60 * 0.25; //60A * 0.25A/bit

	u16 u16_brakeVal = 0;
	s16 s16_brakeVal = 0;
	u8  brakeTorque = 0;
	
	/*�����޷�*/
	if(TR>MAX_TR)
		TR = MAX_TR;
	if(TR<-MAX_TR)
		TR = -MAX_TR;
	g_expectSpeedDir =  expectSpeedDir;
	g_TR = TR;
	if(expectSpeedDir * TR < 0 ||  // �����ٶȷ���������Ť�ط���һ��ʱ��������Ҫ�����ƶ�����
	   expectSpeedDir == 0)	       // �������ٶ�Ϊ0ʱ��������ζ���Ҫ�ƶ�
	{
		brakeTorque = fabs(TR);
		u16_brakeVal = 1.0*maxBrakeVal*brakeTorque/255;
		s16_brakeVal = -u16_brakeVal;
		TR = 0.0; //������Ť����0
	}

    /*����״̬*/
    if(RunSta==3)
    {
        /*����������*/
        if(SonicSta==0)
        {
            if(TR>0)//���ǰ��
            {							
                CtrlMsg[0] = 3;
                CtrlMsg[1] = (TR)&0xFF;
                CtrlMsg[2] = (TR>>8)&0xFF;
                CtrlMsg[3] = 0x20;   //0b0010 0000 ����Ť�ؿ���
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
                CtrlMsg[6] = 0;
                CtrlMsg[7] = 0;
            }
            else if(TR<0)//�������
            {							
                CtrlMsg[0] = 2;
                CtrlMsg[1] = (-TR)&0xFF;
                CtrlMsg[2] = (-TR>>8)&0xFF;
                CtrlMsg[3] = 0x20;
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
                CtrlMsg[6] = 0;
                CtrlMsg[7] = 0;
            }
            else//����ƶ�
            {
				//�˴�Ӧ�ж��������ٶȷ��򣬲��ݴ����ö�Ӧ�ĵ�λ
				//��Ϊ�ٶȷ���ֵΪ������ֻ��ͨ����λֵ�ж��䷽��
				//�������������ǰ��ʻ������λֵ�÷��������õ����ٶȽ�Ϊ���������´���
				if(expectSpeedDir == 1)
					CtrlMsg[0] = 3; //ǰ����
				else
					CtrlMsg[0] = 2; //

                CtrlMsg[1] = 0;
                CtrlMsg[2] = 0;
                CtrlMsg[3] = 0x60;   //0x40 ɲ�� 0x20 ����Ť�ؿ���
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
				
				//�ƶ����� ��-��
				CtrlMsg[6] = s16_brakeVal;
                CtrlMsg[7] = s16_brakeVal >> 8;
            }
        }
        /*����������*/
        else
        {
            CtrlMsg[0] = 3;
            CtrlMsg[1] = 0;
            CtrlMsg[2] = 0;
            CtrlMsg[3] = 0x60;
            CtrlMsg[4] = 0x00;
            CtrlMsg[5] = 0x00;
            CtrlMsg[6] = 0x10;
            CtrlMsg[7] = 0xFF;
        }
    }
    /*ɲ��״̬*/
    else
    {
        CtrlMsg[0] = 3;
        CtrlMsg[1] = 0;
        CtrlMsg[2] = 0;
        CtrlMsg[3] = 0x60;
        CtrlMsg[4] = 0x00;
        CtrlMsg[5] = 0x00;
        CtrlMsg[6] = 0x10;
        CtrlMsg[7] = 0xFF;
    }
    Can_SendExtMsg(CtrlMsg,8,YQRL_EXTID0);
}

/*
*********************************************************************
* @ name :YQRR_TRctrl
* @ func :�Һ������ſ���
* @ prama: RunSta-����״̬[3-��ʻ,����-ɲ��]
********** SonicSta-������״̬[0-����,1-����]
********** TR-���ſ���(-256~256)��ֵǰ������ֵ����
* @retval: ��
********************************************************************
* @attention:��λ[0-�յ�,1-ǰ����,2-���˵�,3-���ٵ�]
*********************************************************************
*/
void YQRR_TRctrl(u8 RunSta,u8 SonicSta,s16 TR, s8 expectSpeedDir)
{
    u8 CtrlMsg[8] = {0,0,0,0,0,0,0,0};
	
	static u16 maxBrakeVal = 60 * 0.25; //60A * 0.25A/bit

	u16 u16_brakeVal = 0;
	s16 s16_brakeVal = 0;
	u8  brakeTorque = 0;
	
	/*�����޷�*/
	if(TR>MAX_TR)
		TR = MAX_TR;
	if(TR<-MAX_TR)
		TR = -MAX_TR;
	
	if(expectSpeedDir * TR < 0 ||  // �����ٶȷ���������Ť�ط���һ��ʱ��������Ҫ�����ƶ�����
	   expectSpeedDir == 0)	       // �������ٶ�Ϊ0ʱ��������ζ���Ҫ�ƶ�
	{
		brakeTorque = fabs(TR);
		u16_brakeVal = 1.0*maxBrakeVal*brakeTorque/255;
		s16_brakeVal = -u16_brakeVal;
		TR = 0; //������Ť����0
	}

    /*����״̬*/
    if(RunSta==3)
    {
        /*����������*/
        if(SonicSta==0)
        {
            if(TR>0)//���ǰ��
            {							
                CtrlMsg[0] = 3;
                CtrlMsg[1] = (TR)&0xFF;
                CtrlMsg[2] = (TR>>8)&0xFF;
                CtrlMsg[3] = 0x28;
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
                CtrlMsg[6] = 0;
                CtrlMsg[7] = 0;
            }
            else if(TR<0)//�������
            {							
                CtrlMsg[0] = 2;
                CtrlMsg[1] = (-TR)&0xFF;
                CtrlMsg[2] = (-TR>>8)&0xFF;
                CtrlMsg[3] = 0x28;
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
                CtrlMsg[6] = 0;
                CtrlMsg[7] = 0;
            }
            else//����ƶ�
            {
				//�˴�Ӧ�ж��������ٶȷ��򣬲��ݴ����ö�Ӧ�ĵ�λ
				//��Ϊ�ٶȷ���ֵΪ������ֻ��ͨ����λֵ�ж��䷽��
				//�������������ǰ��ʻ������λֵ�÷��������õ����ٶȽ�Ϊ���������´���
				if(expectSpeedDir == 1)
					CtrlMsg[0] = 3; //ǰ����
				else
					CtrlMsg[0] = 2; //���˵�
				
                CtrlMsg[1] = 0;
                CtrlMsg[2] = 0;
                CtrlMsg[3] = 0x68;
                CtrlMsg[4] = 0x00;
                CtrlMsg[5] = 0x00;
				
                //�ƶ����� ��-��
				CtrlMsg[6] = s16_brakeVal;
                CtrlMsg[7] = s16_brakeVal >> 8;
            }
        }
        /*����������*/
        else
        {
            CtrlMsg[0] = 3;
            CtrlMsg[1] = 0;
            CtrlMsg[2] = 0;
            CtrlMsg[3] = 0x68;
            CtrlMsg[4] = 0x00;
            CtrlMsg[5] = 0x00;
            CtrlMsg[6] = 0x10;
            CtrlMsg[7] = 0xFF;
        }
    }
    /*ɲ��״̬*/
    else
    {
        CtrlMsg[0] = 3;
        CtrlMsg[1] = 0;
        CtrlMsg[2] = 0;
        CtrlMsg[3] = 0x68;
        CtrlMsg[4] = 0x00;
        CtrlMsg[5] = 0x00;
        CtrlMsg[6] = 0x10;
        CtrlMsg[7] = 0xFF;
    }
    Can_SendExtMsg(CtrlMsg,8,YQRR_EXTID0);
}
