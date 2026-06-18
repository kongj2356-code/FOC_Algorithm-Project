#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "bsp_foc.h"
#include "AS5600.h"
#include "Serial.h"
#include "bsp_tim.h"

#define _3PI_2   4.71238898038468985f
#define _PI      3.14159265358979323846f

float Zero_Angle;
extern float Speed;
extern float Uq;
extern float Ta,Tb,Tc;

/*
90° = π/2 ≈ 1.5708
180° = π ≈ 3.1416
270° = 3π/2 ≈ 4.7124
360° = 2π ≈ 6.2832
*/

float Angle_hope =  0;   

float Angle, Angle_ele;

// 新增：当前锁定的档位
static uint8_t current_gear ;


/* 零电角标定 */
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


// 新增：将弧度转换为角度（方便显示）
float Rad2Deg(float rad)
{
    return rad * 180.0f / _PI;
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
    Knob_Init();      // 初始化X分度棘轮旋钮

     while (1)
    {
        //  获取当前角度和档位
        Angle = GetAngle();   // 获取磁编码器当前角度（弧度）
        // 计算当前锁定的档位（0-7）
        current_gear = (uint8_t)(Calculate_Target_Detent(Angle) / (_2PI/GEAR_NUM));
        printf("%d,%f\r\n",current_gear,Angle);

    }
}

