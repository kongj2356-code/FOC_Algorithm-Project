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
void CurrentLoop_Enable(uint8_t enable);
#endif
