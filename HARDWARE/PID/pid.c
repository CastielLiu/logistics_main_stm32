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
* @func  :λ��ʽPID
* @prama :Target-Ŀ��ֵ
**********Real-ʵ��ֵ
* @retval:PID�����������ֵ   
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

    Bias = Real-Target;   //����ٶ�ƫ��ɲ���ֵ��ȥĿ��ֵ
	/*����ݴ�Χ*/
    if((Bias>-30)&&(Bias<30))
        Bias = 0;
	/*����������*/
    if((Integral_bias<200)&&(Integral_bias>-200))
    {
        Integral_bias += Bias;   //���ƫ��Ļ���
		/*�������޷�*/
        if(Integral_bias>=Target*5)
            Integral_bias = Target*5;
        if(Integral_bias<=-Target*5)
            Integral_bias = -Target*5;
    }
	/*�������*/
    Pwm = Position_KP*Bias
          +Position_KI*Integral_bias
          +Position_KD*(Bias-Last_Bias);//λ��ʽPID������

    Last_Bias=Bias;  //������һ��ƫ��
	
    return Pwm;      //�������
}

int PositionPID1(float kp, float ki, float kd, float state)
{
	return 1;
}

float ERR;
float CMD;
float delT;
//������PID�ٶȿ�����
//����PID��������������expectV, ��ǰ����nowV����ǰʱ��now_time���Ƿ�����PID reset
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
