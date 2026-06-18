#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"


int main(void)
{
	OLED_Init();
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                                  // 复用推挽输出，将引脚的控制权交给片上外设
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10  ;		      
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
   
  // 栅极芯片采样通用定时器：PWM（死区时间由芯片厂商设计了），只需要控制MOS管的一半，下半自动取反
  while(1)
	{
     // 1 0 0
			GPIO_SetBits(GPIOA, GPIO_Pin_8);
			GPIO_ResetBits(GPIOA, GPIO_Pin_9|GPIO_Pin_10);
			Delay_ms(5);

			// 1 1 0
			GPIO_SetBits(GPIOA, GPIO_Pin_8|GPIO_Pin_9);
			GPIO_ResetBits(GPIOA, GPIO_Pin_10);
			Delay_ms(5);

			// 0 1 0
			GPIO_SetBits(GPIOA, GPIO_Pin_9);
			GPIO_ResetBits(GPIOA, GPIO_Pin_8|GPIO_Pin_10);
			Delay_ms(5);

			// 0 1 1
			GPIO_SetBits(GPIOA, GPIO_Pin_9|GPIO_Pin_10);
			GPIO_ResetBits(GPIOA, GPIO_Pin_8);
			Delay_ms(5);

			// 0 0 1
			GPIO_SetBits(GPIOA, GPIO_Pin_10);
			GPIO_ResetBits(GPIOA, GPIO_Pin_9|GPIO_Pin_8);
			Delay_ms(5);

			// 1 0 1
			GPIO_SetBits(GPIOA, GPIO_Pin_10|GPIO_Pin_8);
			GPIO_ResetBits(GPIOA, GPIO_Pin_9);
			Delay_ms(5);

	}	
}

