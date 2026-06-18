#include "stm32f10x.h"                  // Device header
#include "Serial.h"
#include "AS5600/AS5600.h"
#include "Delay.h"	
#include "./FOC/bsp_foc.h"
#include "./ADC/bsp_adc.h"

#define _3PI_2  4.71238898038
#define _2PI    6.28318530718

// 声明外部总控制器结构体
extern FOC_ControllerTypeDef foc;
extern SMO_HandleTypeDef smo;
extern PLL_HandleTypeDef pll;
extern VF_HandleTypeDef vf;
float Zero_Angle;
extern volatile float Speed_Target_Cmd ;   // 串口给的最终目标速度


void BSP_Init()
{
	USART_Config();
	AS5600_Init();
	PWM_Init();
  BSP_TIM();
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
	AD_Init();
  DriftOffsets();
//	Zero_Get();
  BSP_TIM4();
  BSP_TIM3();

  // 启动默认正转VF开环
  VF_Start_Init(1);

	while (1)
	{    

     if(vf.VF_flag == 0)
     {
       M1_Set_Velocity(Serial_RxData);
     }
    
       
         // printf("%f,%f\n", smo.Ealpha_flt, smo.Ebeta_flt);
         // JustFloat_Send3(smo.Ealpha_flt, smo.Ebeta_flt,0);
         // JustFloat_Send3(pll.est_theta, _normalizeAngle(GetAngle()*7),_normalizeAngle(vf.shaft_angle*7));
          JustFloat_Send5(Speed_Target_Cmd, foc.speed, foc.speed_hope, pll.est_theta,_normalizeAngle(GetAngle()*7));
        // VOFA+ JustFloat传输协议
        // JustFloat_Send5(foc.iq_hope, foc.ele.Iq, foc.uq, foc.speed, foc.speed_hope);
        // printf("%f,%f,%f,%f,%f,%f,%f\r\n", foc.iq_hope, foc.ele.Iq, foc.id_hope, foc.ele.Id, foc.ud, foc.uq, foc.speed);
        // printf("%f,%f,%f\r\n", foc.ele.Ia, foc.ele.Ib, foc.ele.Ic);
        // printf("%d,%d\r\n", foc.samp_volts[0], foc.samp_volts[1]);
        // printf("%f,%f\r\n", foc.ele.I_alpha, foc.ele.I_beta);
	}
}

