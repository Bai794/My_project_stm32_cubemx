<<<<<<< HEAD
/**
  ******************************************************************************
  * @file    pid.c
  ******************************************************************************
	* @describe RoboMentor_Client
*/

#include "pid.h"

void Pid_Init(float p, float i, float d, int maxout, int imaxout, int mode) {
    Pid_Struct *pid;
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;
    pid->Last_error = 0;
    pid->Last_Last_error = 0;
    pid->now_error = 0;
    pid->sum_of_error = 0;
    pid->pid_mode = mode;
    pid->MaxOutput = maxout;
    pid->IntegralLimit = imaxout;
    pid->plus = 0;
    pid->plus_out = 0;
    pid->last_plus_out = 0;
}

void PID_Limit_Max(float *a, float PID_MAX) {
    if(*a > PID_MAX) {
        *a = PID_MAX;
    }
    if(*a < -PID_MAX) {
        *a = -PID_MAX;
    }
}

void PID_Limit_Min(float *a, float PID_MIN) {
    if(*a < PID_MIN && *a > 0) {
        *a = PID_MIN;
    }
    if(*a > -PID_MIN && *a < 0) {
        *a = -PID_MIN;
    }
}

float PID_Limit_Float(float a, float max) {
    if(a > 50000) {
        a = a - max;
    }
    if(a < 50000) {
        a = max - a;
    }
    return a;
}

void PID_AbsoluteValue(float *a) {
    if(*a > 0) {
        *a = *a;
    }
    if(*a < 0) {
        *a = -*a;
    }
}

void Pid_Change(float p, float i, float d, int maxout, int imaxout, int mode) {
    Pid_Struct *pid;
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->pid_mode = mode;
    pid->MaxOutput = maxout;
    pid->IntegralLimit = imaxout;
}

float Pid_Calc(float now, float set) {
    Pid_Struct *pid;
    int flag = 1;
    pid->now = now;
    pid->set = set;
    if(pid->now  > 50000) {
        pid->now  = pid->now - 65533;
    }
    pid->now_error = flag * pid->set - pid->now;
    if(pid->now_error < 10  && pid->now_error > -10) {
        pid->now_error = 0;
    }
    if(pid->pid_mode == 1) {
        pid->pout = pid->kp * pid->now_error;
        pid->iout = pid->ki * pid->sum_of_error;
        pid->dout = pid->kd * (pid->now_error - pid->Last_error );
        pid->sum_of_error += pid->now_error;
        PID_Limit_Max(&(pid->sum_of_error), 2000);
        PID_Limit_Max(&(pid->iout), pid->IntegralLimit);
        pid->out = pid->pout + pid->iout + pid->dout;
        PID_Limit_Max(&(pid->out), pid->MaxOutput);
    }
    if(pid->pid_mode == 2) {
        pid->pout = pid->kp * (pid->now_error - pid->Last_error);
        pid->iout = pid->ki * pid->now_error;
        pid->dout = pid->kd * (pid->now_error - 2 * pid->Last_error + pid->Last_Last_error);
        PID_Limit_Max(&(pid->iout), pid->IntegralLimit);
        pid->plus = pid->pout + pid->iout + pid->dout;
        pid->plus_out = pid->last_plus_out + pid->plus;
        pid->out = pid->plus_out;
        PID_Limit_Max(&(pid->out), pid->MaxOutput);
        pid->last_plus_out = pid->plus_out;
    }
    pid->Last_Last_error = pid->Last_error;
    pid->Last_error = pid->now_error;
    return pid->out * flag;
}

void Pid_Callback(char *type, float p, float i, float d, int maxout, int imaxout, int mode) {
    if(memcmp(type, "PID-INIT", 8) == 0) {
        Pid_Init(p, i, d, maxout, imaxout, mode);
    }
}


=======
#include "pid.h"

void pid (int target,uint8_t bainma)
{
	static float error,pwm,lasterror;
	error=bainma-target;
	pwm+=kp*(error-lasterror)+ki*error;
	lasterror=error;
	if(__fabs( pwm)>1000) pwm=1000;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,__fabs( pwm));
	
}
>>>>>>> 699fbad (pid电机)
