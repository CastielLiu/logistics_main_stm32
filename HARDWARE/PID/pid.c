#include "pid.h"
#include "sys.h"
#include "math.h"

PID_param_t g_speedPID;

void init_pid_params(PID_param_t* pid, float kp, float ki, float kd)
{
	pid->kd = kd;
	pid->ki = ki;
	pid->kp = kp;
}

/**
*********************************************************************
* @brief :PositionPID
* @func  :位置式PID
* @prama :Target-目标值
**********Real-实际值
* @retval:PID控制器输出的值   
********************************************************************
* @attention :
*********************************************************************
  */ 
int PositionPID (int Target,int Real)
{
    float Position_KP=0.97;
    float Position_KI=0.11;
    float Position_KD=0;

    static float Bias,Pwm,Integral_bias,Last_Bias;

    Bias = Real-Target;   //求出速度偏差，由测量值减去目标值
	/*误差容错范围*/
    if((Bias>-30)&&(Bias<30))
        Bias = 0;
	/*误差积分区间*/
    if((Integral_bias<200)&&(Integral_bias>-200))
    {
        Integral_bias += Bias;   //求出偏差的积分
		/*误差积分限幅*/
        if(Integral_bias>=Target*5)
            Integral_bias = Target*5;
        if(Integral_bias<=-Target*5)
            Integral_bias = -Target*5;
    }
	/*计算输出*/
    Pwm = Position_KP*Bias
          +Position_KI*Integral_bias
          +Position_KD*(Bias-Last_Bias);//位置式PID控制器

    Last_Bias=Bias;  //保存上一次偏差
	
    return Pwm;      //增量输出
}

int PositionPID1(float kp, float ki, float kd, float state)
{
	return 1;
}

float ERR;
float CMD;
float delT;
//增量型PID速度控制器
//输入PID参数、期望车速expectV, 当前车速nowV、当前时间now_time、是否重置PID reset
float IncrementPIDspeedCtrl(const PID_param_t* PID, float expectV, float nowV, double now_time, u8 reset)
{
	static float lastCmd=0.0;
	static float lastErr=0.0, lastLastErr=0.0;
	static double lastTime = 0.0;
	float deltaT, err;
	float cmd;
	
	if(lastTime == 0.0)
	{
		lastTime = now_time;
		return 0.0;
	}
	
	deltaT = now_time - lastTime;
	err = expectV - nowV;
	
	if(fabs(err) < 0.1)
		err = 0.0;
	
	ERR = err;
	CMD = cmd;
	delT = deltaT;
	
	if(deltaT > 0.3 || reset)
	{
		cmd = 0.0;
		lastCmd = 0.0;
		lastErr = lastLastErr = 0.0;
		lastTime = now_time;
		return cmd;
	}
	else
		cmd = lastCmd + PID->kp*(err-lastErr) + PID->ki*err + PID->kd*(err-2*lastErr+lastLastErr)/deltaT;
	lastCmd = cmd;
	lastLastErr = lastErr;
	lastErr = err;
	lastTime = now_time;
	
	return cmd;
}
