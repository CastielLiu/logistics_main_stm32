#ifndef _COMMON_H_
#define _COMMON_H_

#define BOAD_SU_YU_DONG_LI 1          //��������ư�
#define BOAD_ZHENG_DIAN_YUAN_ZI 2     //����ԭ�ӿ�����

//�޸Ĵ˺궨�������ò�ͬ�ĵ�Ƭ��
#define WHICH_BOAD BOAD_SU_YU_DONG_LI           //������
//#define WHICH_BOAD BOAD_ZHENG_DIAN_YUAN_ZI    //����ԭ�ӿ�����

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

#define STEER_CONTROL_INCREMENT 				1  //��������
#define STEER_CONTROL_POSITION  				2  //λ�ÿ���
#define STEER_CONTROL_POSITION_AND_INCREMENT	3  //λ�ÿ����¶�ȡ�����ǰλ�ã�ʹ�õ�ǰλ�ü���������Ϊ����ֵ

#define STEER_CONTROL_MODE  STEER_CONTROL_POSITION_AND_INCREMENT
//ǰ��ת���Լ�ת�Ǵ������궨
#define ROADWHEEL_ANGLE_MAX_DEG  12.3     //ǰ��ת�����ֵ
#define ANGLE_SENSOR_MID_VOLTAGE 1.72	  //ǰ��ת����λʱ��Ӧ�ĽǶȴ�������ѹֵ
#define ANGLE_SENSOR_MAX_VOLTAGE 2.77	  //ǰ��ת�����ֵ��Ӧ�ĽǶȴ�������ѹֵ

#define REDUCER_TRANSMISSION_RATIO 64.0   //�������ᵽת��������ᴫ����
										  //����1��  64.0
										  //����2��  36.0


#endif

