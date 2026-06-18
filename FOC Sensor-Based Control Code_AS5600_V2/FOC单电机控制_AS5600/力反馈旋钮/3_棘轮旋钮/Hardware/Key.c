#include "stm32f10x.h"                  // Device header
#include "Delay.h"

void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;      //上拉输入模式   //当外部无输入时，引脚保持高电平1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}


/**
  * 函    数：按键获取键码
  * 参    数：无
  * 返 回 值：按下按键的键码值，范围：0~2，返回0代表没有按键按下
  * 注意事项：此函数是阻塞式操作，当按键按住不放时，函数会卡住，直到按键松手
  */
uint8_t Key_GetNum(void)
{
	
	uint8_t KeyNum = 0;
	if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) == 0)   // 检测按键是否按下（低电平有效）
	{
			Delay_ms(20);                                    // 延时消抖
			while (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) == 0); // 等待按键释放（松手检测）
			Delay_ms(20);                                    // 再次消抖，确保稳定释放
			KeyNum = 1;                                      // 设置按键编号或标志位
	}

	
	if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13) ==0)
	{	
		Delay_ms(20);
		while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13) ==0);
		Delay_ms(20);
		KeyNum = 2;
		
	}
	
	return KeyNum;
	
	
}


