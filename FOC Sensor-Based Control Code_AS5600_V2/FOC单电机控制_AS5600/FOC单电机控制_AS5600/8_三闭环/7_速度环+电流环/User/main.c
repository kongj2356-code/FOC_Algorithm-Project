#include "stm32f10x.h"                  // Device header
#include "Serial.h"
#include "AS5600/AS5600.h"
#include "Delay.h"	
#include "./FOC/bsp_foc.h"
#include "./ADC/bsp_adc.h"

#define _3PI_2  4.71238898038
#define _2PI    6.28318530718


/*
	0°= 0 rad
	90°= π/2 ≈ 1.5708 rad
	180°= π ≈ 3.1416 rad
	270°= 3π/2 ≈ 4.7124 rad
*/


// 声明外部总控制器结构体（关键！）
extern FOC_ControllerTypeDef foc;


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
	AD_Init();
  DriftOffsets();
  BSP_TIM();
  
	while (1)
	{  
      // 或者用串口控制
         M1_Set_Position(Serial_RxData);
       //  JustFloat_Send3(Serial_RxData,_normalizeAngle(GetAngle()), 0);
        // VOFA+ JustFloat传输协议
       
       //  JustFloat_Send5(foc.iq_hope, foc.ele.Iq, foc.uq, foc.speed, foc.speed_hope);

       // printf("%f,%f,%f,%f,%f,%f,%f\r\n",Iq_hope,b.Iq,Id_hope,b.Id,Ud,Uq,Speed);
			// printf("%f,%f,%f\r\n",b.Ia,b.Ib,b.Ic);
				// printf("%d,%d\r\n",adc1_val,adc2_val);
				// printf("%f,%f\r\n",b.I_alpha,b.I_beta);
	}
}
