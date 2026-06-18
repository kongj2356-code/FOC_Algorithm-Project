#ifndef __BSP_FOC_H
#define __BSP_FOC_H

void PWM_Init(void);
void SetSVPWM_FOC(float Ud, float Uq, float angle_e);

/* 新增：FOC 开环“音频驱动” */
void FOC_VelocityOpenLoop(float omega_e);

#endif