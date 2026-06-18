#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "bsp_foc.h"
#include "AS5600.h"
#include "MyI2C.h"
#include "Serial.h"
 
#define _2PI     6.28318530717958647f
#define _3PI_2   4.71238898038468985f
#define _PI_3    1.04719755119659f    // π/3

void BSP_Init()
{
    OLED_Init();
    PWM_Init();
    MyI2C_Init();
    AS5600_Init();
    Serial_Init();
}


float Zero_Angle = 0;    // 零度电气角

// 零度电气角获取
void Zero_Get(void)
{
    float Elec_Angle;
   for(int i = 0; i <= 200; i++)  
    // 旋转一个完整的360°电角度
    {
        Elec_Angle = _3PI_2 + _2PI * i / 200.0f;          //3pai/2因为setPhaseVoltage_FOC2里会加pi/2，这样一来初始的电气角度归0
        setPhaseVoltage_FOC2(4.0f, 0.0f, Elec_Angle);  
        Delay_ms(1); 
    }
    for(int i = 200; i >= 0; i--)
     // 再转回去
    {
        Elec_Angle = _3PI_2 + _2PI * i / 200.0f;
        setPhaseVoltage_FOC2(4.0f, 0.0f, Elec_Angle);  
        Delay_ms(5);
    }
    
    Zero_Angle =  GetAngle() * 7;                         //并且读取当前时刻的机械角，后续在使用时减去，即完成了电气角和机械角的零位对齐
 
}





int main(void)
{

		float Angle; // 当前机械角度乘以极对数得到的原始电气角度
    
    BSP_Init();
    Zero_Get();  // 获取零位偏移

    while(1)
    {
      Angle =  GetAngle() * 7;
      setPhaseVoltage_FOC2(1.0f, 0.0f, Angle - Zero_Angle);
      printf("%f\r\n",Angle - Zero_Angle);

    }
}

