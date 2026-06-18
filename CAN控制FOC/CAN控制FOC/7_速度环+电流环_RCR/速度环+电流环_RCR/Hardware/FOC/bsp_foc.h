#ifndef __BSP_FOC_H
#define __BSP_FOC_H


void PWM_Init(void);
void setPhaseVoltage_FOC2(float Uq, float Ud, float angle_el);
void DriftOffsets(void);




#endif
