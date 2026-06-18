#include "stm32f10x.h"                  // Device header
#include "math.h"
#include "stm32f10x.h"
#include "bsp_pid.h"


float integral_vel_prev;	//速度积分的累加
float integral_agl_prev;  //角度积分的累加


float Speed_PID(float Kp,float Ki,float Kd,float error)
{
		float output;
	float proportional,integral;

	
	proportional = Kp * (error);
	integral = integral_vel_prev + Ki*error;
	integral = _constrain_I(integral,-0.1,0.1);
	output = proportional + integral;
	output = _constrain(output, -6, 6);
	
	//存储上一次过程量
	integral_vel_prev += Ki*error;	
	return output;


}


float Angle_PID(float Kp,float Ki,float Kd,float error)
{

	float output;
	float proportional,integral;
	proportional = Kp * (error);
	integral = integral_agl_prev + Ki*error;
	integral = _constrain_I(integral,-0.1,0.1);
	output = proportional + integral;
	output = _constrain(output, -5, 5);
	
	//存储上一次过程量
	integral_agl_prev += Ki*error;	
	return output;


}

