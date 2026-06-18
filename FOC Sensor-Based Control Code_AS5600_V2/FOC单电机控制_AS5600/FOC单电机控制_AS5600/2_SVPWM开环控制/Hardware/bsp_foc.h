#ifndef __BSP_FOC_H
#define __BSP_FOC_H

void PWM_Init(void);
void SetSVPWM_FOC(float Ud,float Uq ,float angle_e1);

#endif 