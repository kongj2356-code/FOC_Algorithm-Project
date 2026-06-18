#ifndef __BSP_PID_H
#define __BSP_PID_H

#include <stdint.h>

#define PID_D_AXIS   0u
#define PID_Q_AXIS   1u

float PID_Controller(uint8_t axis, float Kp, float Ki, float Kd, float Error);
float Speed_PID(float Kp, float Ki, float Kd, float error);
void PID_Clear_Integral(void);

#endif
