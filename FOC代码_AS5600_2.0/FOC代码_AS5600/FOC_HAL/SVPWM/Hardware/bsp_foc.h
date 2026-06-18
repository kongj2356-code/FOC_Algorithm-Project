#ifndef __BSP_FOC_H
#define __BSP_FOC_H

#include "stm32f1xx_hal.h" 

#ifdef __cplusplus
extern "C" {
#endif

// º¯ÊýÉùÃ÷
void FOC_PWM_Start(void);

void setPhaseVoltage_FOC2(float Uq, float Ud, float angle_el);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_FOC_H */
