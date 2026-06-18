#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "bsp_foc.h"
#include "Serial.h"


extern float Ta,Tb,Tc;

void BSP_Init()
{
    OLED_Init();
    PWM_Init();
    Serial_Init();
}

extern uint16_t Sector;

int main(void)
{
  BSP_Init();
  float a;
  while(1)
 {
    SetSVPWM_FOC(0,1,a);
    a += 0.1;

    //  printf("%f,%f,%f\r\n",Ta,Tb,Tc); // 三相占空比输出
   // printf("%d\r\n",Sector); // 扇区打印
  //  Delay_ms(1);
 }	
}

