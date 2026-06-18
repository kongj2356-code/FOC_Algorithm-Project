#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "bsp_foc.h"
#include "AS5600.h"
#include "MyI2C.h"
#include "Serial.h"
#include "bsp_tim.h"
#include "Key.h"

extern float Angle_hope_M1 ;  // 电机1期望角度（替换原速度期望）
extern float Angle_hope_M2 ;  // 电机2期望角度（替换原速度期望）



uint16_t MODE;

void BSP_Init(void)
{    
    Serial_Init();
    OLED_Init();
    Key_Init();
    PWM_Init_Double();                              //
    MyI2C_Init();
    AS5600_Init(AS5600_CHANNEL_MOTOR1);             //
    AS5600_Init(AS5600_CHANNEL_MOTOR2);             //
} 



int main(void)
{
    BSP_Init();
    FOC_DualMotor_Init(); 
    // 控制单/双电机
    Set_Motor_Ctrl_Mode(3);
    while (1)
    {
      Angle_hope_M1  = Serial_RxData;   // 串口调试
      Angle_hope_M2 = Serial_RxData;

    //  printf("%f\r\n",Angle_FOC(GetAngle(1)));
      printf("%f,%f,%f\r\n",Angle_FOC(Motor1.Angle), Angle_FOC(Motor2.Angle), Serial_RxData);
    }

}
