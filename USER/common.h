#ifndef _COMMON_H_
#define _COMMON_H_

#define BOAD_SU_YU_DONG_LI 1          //速羽动力设计板
#define BOAD_ZHENG_DIAN_YUAN_ZI 2     //正点原子开发板

//修改此宏定义以适用不同的单片机
#define WHICH_BOAD BOAD_SU_YU_DONG_LI           //速羽动力
//#define WHICH_BOAD BOAD_ZHENG_DIAN_YUAN_ZI    //正点原子开发板

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

#define STEER_CONTROL_INCREMENT 				1  //增量控制
#define STEER_CONTROL_POSITION  				2  //位置控制
#define STEER_CONTROL_POSITION_AND_INCREMENT	3  //位置控制下读取电机当前位置，使用当前位置加上增量作为控制值

#define STEER_CONTROL_MODE  STEER_CONTROL_POSITION_AND_INCREMENT
//前轮转角以及转角传感器标定
#define ROADWHEEL_ANGLE_MAX_DEG  12.3     //前轮转角最大值
#define ANGLE_SENSOR_MID_VOLTAGE 1.72	  //前轮转角中位时对应的角度传感器电压值
#define ANGLE_SENSOR_MAX_VOLTAGE 2.77	  //前轮转角最大值对应的角度传感器电压值

#define REDUCER_TRANSMISSION_RATIO 64.0   //电机输出轴到转向机输入轴传动比
										  //杰青1号  64.0
										  //杰青2号  36.0


#endif

