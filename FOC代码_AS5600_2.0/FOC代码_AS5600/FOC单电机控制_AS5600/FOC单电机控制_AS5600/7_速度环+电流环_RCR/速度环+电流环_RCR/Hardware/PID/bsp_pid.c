#include "./PID/bsp_pid.h"
#include "stm32f10x.h"
#include "Delay.h"
#include "Serial.h"

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _constrain_I(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define  limit            6.3
#define  Output_ramp      10000


static float integral_vel_prev_Speed = 0;


unsigned long Timestamp_Last = 0.0;
float Last_Error = 0.0;
float Last_intergration = 0.0;
float Last_Output = 0.0;


float PID_Controller(float Kp, float Ki, float Kd, float Error)
{
	float Ts = 0.0;
	uint32_t Timestamp = SysTick->VAL;
  if(Timestamp < Timestamp_Last) Ts = (float)(Timestamp_Last - Timestamp)/9*1e-6;
	else
		Ts = (0xFFFFFF - Timestamp + Timestamp_Last)/9*1e-6;
	
	if(Ts<=0 || Ts > 0.05f) Ts = 0.001;
	
	float proportion = Kp * Error;//P环
	
	float intergration = Last_intergration + Ki * 0.5f * Ts * Error;//I环
	intergration = _constrain(intergration, -limit, limit);
	
	float differential = Kd * (Error - Last_Error)/Ts;//D环
	
	float Output = proportion + intergration + differential;
	Output = _constrain(Output, -limit, limit);
	
	Last_Error = Error;
	Last_intergration = intergration;
	Last_Output = Output;
	Timestamp_Last = Timestamp;
	
	return Output;
}


float Speed_PID(float Kp,float Ki,float Kd,float error)
{
	float output;
	float proportional,integral;

	
	proportional = Kp * (error);
	integral = integral_vel_prev_Speed + Ki*error;
	integral = _constrain_I(integral,-1,1);
	output = proportional + integral;
	output = _constrain(output, -2, 2);
	
	//存储上一次过程量
	integral_vel_prev_Speed = integral;	
	return output;


}
