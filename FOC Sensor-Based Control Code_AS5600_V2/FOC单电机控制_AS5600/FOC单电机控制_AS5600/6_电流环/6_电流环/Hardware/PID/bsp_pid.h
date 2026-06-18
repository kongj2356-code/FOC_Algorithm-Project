#ifndef __BSP_PID_H
#define __BSP_PID_H

float PID_Ele_Id(float Kp,float Ki,float Kd,float error);
float PID_Ele_Iq(float Kp,float Ki,float Kd,float error);
float PID_Controller(float Kp, float Ki, float Kd, float Error);

#endif
