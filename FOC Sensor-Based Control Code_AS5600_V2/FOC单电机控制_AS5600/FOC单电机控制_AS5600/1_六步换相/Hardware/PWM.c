#include "stm32f10x.h"                  // Device header



// PWM的初始化 (欲产生一个频率为1KHz ，占空比50% ， 分辨率为1% 的PWM波形)
void PWM_Init(void)
{
	// RCC时钟开启，要用的TIM外设和GPIO外设
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
// 以下是引脚重映射相关配置，当前被注释掉未使用
// RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);     // 开启AFIO复用功能时钟（使用重映射时需要）
// GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);    // 配置TIM2部分重映射1（改变TIM2的默认引脚映射）
// GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // 禁用JTAG功能，释放相关引脚用于其他用途
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   // 复用推挽输出，将引脚的控制权交给片上外设
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;		// 如果使用了上面的重映射功能，这里可能需要改为对应的重映射引脚（如注释中的GPIO_Pin_15）
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// 配置时钟源头选择，时基单元
	TIM_InternalClockConfig(TIM2);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;		      //ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 720 - 1;	  	//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
	// 配置输出比较单元
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;              // 选用PWM模式1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      // 极性不翻转
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  // 输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;	        	//CCR
	
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);  //TIM2 的 OC1 可输出波形，但是最后要借助GPIO口来输出(PA0 查表)
	
	// 启动定时器
	TIM_Cmd(TIM2, ENABLE);
}



void PWM_SetCompare1(uint16_t Compare)
{
	// 单独动态更改通道1的CCR的值，此时将PWM_Init(void)的CCR置0
	TIM_SetCompare1(TIM2, Compare);
}


