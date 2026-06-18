#include "stm32f10x.h"                  // Device header
#include "math.h"

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _constrain_I(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
float integral_vel_prev;

float Speed_PID(float Kp,float Ki,float Kd,float error)
{
	float output;
	float proportional,integral;

	
	proportional = Kp * (error);
	integral = integral_vel_prev + Ki*error;
	integral = _constrain_I(integral,-5,8);
	output = proportional + integral;
	output = _constrain(output, -9, 12);
	
	//存储上一次过程量
	integral_vel_prev += Ki*error;	
	return output;


}
