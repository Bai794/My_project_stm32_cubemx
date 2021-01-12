#ifndef API_PID_H
#define API_PID_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

typedef struct{
	float kp;
	float ki;
	float kd;
	double pout;
	float iout;
	float dout;
	float now_error;
	float Last_error;
	float Last_Last_error;
	float sum_of_error;
	float set;
	float now;
	float out;
	int pid_mode;
  float MaxOutput;
	float IntegralLimit;
	float plus;
  float plus_out;
  float last_plus_out;
}Pid_Struct;

void Pid_Callback(char *type,float p,float i,float d,int maxout,int imaxout,int mode);

#endif

