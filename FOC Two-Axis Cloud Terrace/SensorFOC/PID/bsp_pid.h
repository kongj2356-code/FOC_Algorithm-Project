#ifndef __BSP_PID_H
#define __BSP_PID_H

#include "stm32f10x.h"

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float last_error;
    float last_output;

    uint32_t timestamp_last;

    float integral_limit;
    float output_limit;

} PID_TypeDef;


void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float integral_limit,float output_limit);

void PID_Reset(PID_TypeDef *pid);

float PID_Controller_State(PID_TypeDef *pid, float Error);
float Speed_PID_State(PID_TypeDef *pid, float Error);
float Angle_PID(float Kp, float Ki, float Kd, float Error);

#endif
