#ifndef __TIMER_H
#define __TIMER_H

#include "sys.h"
#include "stdlib.h"


void TIM7_Init(void);
void TIM8_Init(void);

extern double seconds;
extern float Alpha;
extern u8 Industry_TxMsg[11];

extern uint32_t g_timerCnt10ms;
extern double g_seconds;
extern double g_lastValidCmdTime;
extern float g_vehicleSpeed;
extern u8    g_angleSensorInited;
extern float g_firstRoadwheelAngle;
extern float g_roadwheelAngle;
extern float g_sensorVoltage;

#endif

