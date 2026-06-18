#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "bsp_foc.h"
#include "AS5600.h"
#include "MyI2C.h"
#include "Serial.h"
#include "bsp_tim.h"

#define _2PI     6.28318530717958647f
#define _3PI_2   4.71238898038468985f

float Zero_Angle;
float Speed_hope = 0.0;
extern float Speed;
extern float Uq;
extern float Angle;

float Ta,Tb,Tc;


/* 零电角标定 */
void Zero_Get(void)
{
    float Elec_Angle;
    for (int i = 0; i <= 200; i++)
    {
        Elec_Angle = _3PI_2 + _2PI * i / 200.0f;
        setPhaseVoltage_FOC2(4.0f, 0.0f, Elec_Angle);
        Delay_ms(1);
    }

    for (int i = 200; i >= 0; i--)
    {
        Elec_Angle = _3PI_2 + _2PI * i / 200.0f;
        setPhaseVoltage_FOC2(4.0f, 0.0f, Elec_Angle);
        Delay_ms(5);
    }

    Zero_Angle = GetAngle() * 7;

}



void BSP_Init(void)
{    
    Serial_Init();
    OLED_Init();
    PWM_Init();
    MyI2C_Init();
    AS5600_Init();

}


int main(void)
{
    BSP_Init();
    Zero_Get();
    BSP_TIM(); 
    while (1)
    {
      Speed_hope = Serial_RxData;   // 串口调试
 
      printf("%f\r\n", Uq);
  
    }

}
