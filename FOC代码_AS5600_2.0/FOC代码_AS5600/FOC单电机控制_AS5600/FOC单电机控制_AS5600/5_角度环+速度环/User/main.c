#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "bsp_foc.h"
#include "AS5600.h"
#include "Serial.h"
#include "bsp_tim.h"

#define _2PI     6.28318530717958647f
#define _3PI_2   4.71238898038468985f
#define _PI      3.14159265358979323846f

float Zero_Angle;
extern float Speed;
extern float Uq;
extern float Ta,Tb,Tc;

/*
90°„ = ¶–/2 °÷ 1.5708
180°„ = ¶– °÷ 3.1416
270°„ = 3¶–/2 °÷ 4.7124
360°„ = 2¶– °÷ 6.2832
*/

float Angle_hope =  0;   

float Angle, Angle_ele;
uint16_t a;

/* ¡„µÁΩ«±Í∂® */
void Zero_Get(void)
{
    float Elec_Angle;
    for (int i = 0; i <= 500; i++)
    {
        Elec_Angle = _3PI_2 + _2PI * i / 500.0f;
        setPhaseVoltage_FOC2(3.0f, 0.0f, Elec_Angle);
        Delay_us(500);
    }

    for (int i = 500; i >= 0; i--)
    {
        Elec_Angle = _3PI_2 + _2PI * i / 500.0f;
        setPhaseVoltage_FOC2(3.0f, 0.0f, Elec_Angle);
        Delay_us(500);
    }

    Zero_Angle = GetAngle() * 7;
}

void BSP_Init(void)
{    
    Serial_Init();
    OLED_Init();
    PWM_Init();
    AS5600_Init();
}

int main(void)
{
    BSP_Init();
    Zero_Get();
    BSP_TIM(); 
    
    while (1)
    {
     printf("%f,%f,%f\r\n",Angle_hope,GetAngle(),Uq);
    }
}