#include "stm32f10x.h"                  // Device header
#include "Serial.h"
#include "AS5600/AS5600.h"
#include "Delay.h"	
#include "./FOC/bsp_foc.h"
#include "./ADC/bsp_adc.h"

#define _3PI_2  4.71238898038
#define _2PI    6.28318530718

extern float Uq,Ud;
extern float Iq_hope , Id_hope;
extern uint16_t adc1_val,adc2_val;
extern ElectricalQuantities b;

float Zero_Angle;


void BSP_Init()
{
	USART_Config();
	AS5600_Init();
	PWM_Init();
}


void Zero_Get()
{
	float Angle;
	for(int i =0; i <200;i++)
	{
		Angle = _3PI_2 + _2PI * i / 200.0;
		setPhaseVoltage_FOC2(7, 0,  Angle);
		Delay_ms(1);
	}
	for(int i=200; i>=0; i--) //再转回去
	{
		Angle = _3PI_2 + _2PI * i / 200.0 ;
		setPhaseVoltage_FOC2(7, 0,  Angle);
		Delay_ms(5);
	}
	Zero_Angle = GetAngle() * 7;
  Delay_ms(20);
}



int main(void)
{
	BSP_Init();
	Zero_Get();
	ADC_InjectedSync_Init();
  DriftOffsets();
  CurrentLoop_Enable(1);
  
	while (1)
	{

    Iq_hope=Serial_RxData;
	 printf("%f,%f,%f,%f\r\n",Iq_hope,b.Iq,Uq,b.Id);
 
    // printf("%f,%f,%f\r\n",b.Ia,b.Ib,b.Ic);
   // printf("%d,%d\r\n",adc1_val,adc2_val);
  //  printf("%f,%f\r\n",b.Id,b.Iq);

	}
}
