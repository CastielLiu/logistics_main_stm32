#ifndef _COMMON_H_
#define _COMMON_H_

#define BOAD_SU_YU_DONG_LI 1          //速羽动力设计板
#define BOAD_ZHENG_DIAN_YUAN_ZI 2     //正点原子开发板

//修改此宏定义以适用不同的单片机
#define WHICH_BOAD BOAD_SU_YU_DONG_LI           //速羽动力
//#define WHICH_BOAD BOAD_ZHENG_DIAN_YUAN_ZI   //正点原子开发板

#if WHICH_BOAD==BOAD_ZHENG_DIAN_YUAN_ZI
	#define CAN1_REMAP2_PB89 0
	#define AUTO_RXTX_485    0
	#define LED_PB5PE5       1

#elif WHICH_BOAD == BOAD_SU_YU_DONG_LI
	#define CAN1_REMAP2_PB89 1
	#define AUTO_RXTX_485    1
	#define LED_PB5PE5       0
	
#endif

#define ENABLE_SONIC 1   //超声波使能控制

#endif

