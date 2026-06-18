#ifndef __BSP_PID_H
#define __BSP_PID_H

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _constrain_I(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

float Speed_PID(float Kp, float Ki, float Kd, float error);
float Angle_PID(float Kp, float Ki, float Kd, float error);

#endif
