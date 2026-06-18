#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "stm32f10x.h" 

typedef struct 
{
	float Ia;
	float Ib;
	float Ic;
	float I_alpha;
	float I_beta;
	float Iq;
	float Id;
}ElectricalQuantities;

void DriftOffsets(void);
void ADC_InjectedSync_Init(void);
void GPIO_Config(void);
void BSP_TIM(void);

void M1_Set_Velocity(float Target_Velocity);
void M1_Set_CurTorque(float Target_Current);

#endif
