#ifndef _COMMON_H_
#define _COMMON_H_

#define BOAD_SU_YU_DONG_LI 1          //��������ư�
#define BOAD_ZHENG_DIAN_YUAN_ZI 2     //����ԭ�ӿ�����

//�޸Ĵ˺궨�������ò�ͬ�ĵ�Ƭ��
#define WHICH_BOAD BOAD_SU_YU_DONG_LI           //������
//#define WHICH_BOAD BOAD_ZHENG_DIAN_YUAN_ZI   //����ԭ�ӿ�����

#if WHICH_BOAD==BOAD_ZHENG_DIAN_YUAN_ZI
	#define CAN1_REMAP2_PB89 0
	#define AUTO_RXTX_485    0
	#define LED_PB5PE5       1

#elif WHICH_BOAD == BOAD_SU_YU_DONG_LI
	#define CAN1_REMAP2_PB89 1
	#define AUTO_RXTX_485    1
	#define LED_PB5PE5       0
	
#endif

#define ENABLE_SONIC 1   //������ʹ�ܿ���

#endif

