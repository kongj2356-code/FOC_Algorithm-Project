#include "stm32f10x.h"                  // Device header


void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;   //定义结构体变量
    
    // 初始化 GPIOA Pin8
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);    // 开启时钟
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 初始化 GPIOD Pin12  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);     // 开启时钟
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure); 
    
    // 设置初始状态为高电平（LED熄灭）
    GPIO_SetBits(GPIOA, GPIO_Pin_8);
    GPIO_SetBits(GPIOD, GPIO_Pin_2);
}

 
void LED1_ON(void)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);
}


void LED1_OFF(void)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_8);
}

 

/**
  * 函    数：LED1状态翻转
  * 参    数：无
  * 返 回 值：无
  */
void LED1_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_8) == 0)		//获取输出寄存器的状态，如果当前引脚输出低电平
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_8);					//则设置PA8引脚为高电平
	}
	else													//否则，即当前引脚输出高电平
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_8);					//则设置PA8引脚为低电平
	}
}


void LED2_ON(void)
{
	GPIO_ResetBits(GPIOD, GPIO_Pin_2);
}


void LED2_OFF(void)
{
	GPIO_SetBits(GPIOD, GPIO_Pin_2);
}


void LED2_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_2) == 0)
	{
		GPIO_SetBits(GPIOD, GPIO_Pin_2);
	}
	else
	{
		GPIO_ResetBits(GPIOD, GPIO_Pin_2);
	}
}


