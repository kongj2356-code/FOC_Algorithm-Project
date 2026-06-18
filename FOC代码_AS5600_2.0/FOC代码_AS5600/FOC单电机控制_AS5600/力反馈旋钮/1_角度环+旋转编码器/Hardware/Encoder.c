#include "stm32f10x.h"                  // Device header


// PA6-TIM3_CH1    PA7-TIM3_CH2   （注意CH3 & CH4 不允许的）
void Encoder_Init(void)
{
	// 开启时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	// GPIO初始化
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     // 上拉输入（上下拉的选取原则：和外部模块保持默认一致，防止默认电平打架）
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// 无需内部时钟——编码器接口托管时钟，编码器接口就是一个带方向控制的外部时钟
		
	//配置时基单元 
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //无作用，计数方向也是被编码器接口托管的
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;		//ARR(满量程计数，计数范围最大，方便换算成负数)
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;		//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	// 输入捕获单元配置(只用了这两部分 滤波器 / 极性选择)
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure);   //默认赋初始值
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0xF;
	//TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    编码器部分会写，这里可去除，避免重复定义
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0xF;
	//TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;     编码器部分会写，这里可去除，避免重复定义
	TIM_ICInit(TIM3, &TIM_ICInitStructure);  
	
	// 配置编码器模式接口
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	                                                         //通道不反相             通道不反相                 
	TIM_Cmd(TIM3, ENABLE);
}



int16_t Encoder_Get_counter(void)   //返回值类型由 uint16_t 强转为 int16_t 这样65535--> -1
{
	return TIM_GetCounter(TIM3);
}

