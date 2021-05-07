#ifndef __PID_H
#define __PID_H

#include "sys.h"

typedef struct 
{
	float kp,ki,kd;
}PID_param_t;

extern PID_param_t g_speedPID; //速度PID参数

void init_pid_params(PID_param_t* pid, float kp, float ki, float kd);

int PositionPID (int Target,int Real);

float IncrementPIDspeedCtrl(const PID_param_t* PID, float expectV, float nowV, double now_time, u8 reset);

#endif


