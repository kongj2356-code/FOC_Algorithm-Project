#include "./FOC/bsp_foc.h"
#include "stm32f10x.h"                  // Device header
#include "math.h"
#include "IQ_math.h"



#define _PI    3.14159265359
#define _PI_2  1.57079632679
#define _PI_3  1.0471975512
#define _2PI   6.28318530718
#define _3PI_2 4.71238898038
#define _PI_6  0.52359877559
#define _SQRT3 1.73205080757

// 角度归一化函数：将任意角度转换到 [0, 2π) 区间
float _normalizeAngle(float angle)
{
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}



void PWM_Init()
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef       TIM_OCInitStructure;

	//使能TIM1,GPIOA时钟 ,复用时钟AFIO不需要，只有重映射等3种情况才需要复用功能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA , ENABLE);
	
	//配置GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 |GPIO_Pin_9 |GPIO_Pin_10 ;	 //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 		 //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);			    		 //根据设定参数初始化GPIOA
	
	//定时器实际参数初始化
	TIM_TimeBaseInitStructure.TIM_Period = 1800 - 1;               
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;             
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	//TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim //这句话是设置定时器时钟(CK_INT)频率与数字滤波器(ETR，TIx)使用的采样频率之间的分频比例的（与输入捕获相关），0表示滤波器的频率和定时器的频率是一样的
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);
	
	TIM1->RCR = 1;   //  设置重复计数器RCR（可根据实际需求调整）
	
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
	//TIM1-CH1参数配置
	//！！！！！！！！！！！！！！！高级定时器的结构体必须全部配置，否则输出不了PWM波！！！！
  TIM_OCInitStructure.TIM_Pulse = 0;                         //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  //使能输出比较状态   //使能输出到端口
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//使能输出比较N状态
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//当 MOE=0 重置 TIM1 输出比较空闲状态
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;//当 MOE=0 重置 TIM1 输出比较 N 空闲状态
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//TIM1 输出比较N极性高
	
	
	TIM_OC1Init(TIM1,&TIM_OCInitStructure);
	//TIM1-CH4参数配置
	TIM_OC2Init(TIM1,&TIM_OCInitStructure);
	TIM_OC3Init(TIM1,&TIM_OCInitStructure);
	
	//!!!!!!!!!!!!!!!!!使能预装载寄存器!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); 
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable); 
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable); 

	//使能TIMx在ARR上的预装载寄存器
	TIM_ARRPreloadConfig(TIM1, ENABLE); 
		//MOE 主输出使能  高级定时器会用到   	
	TIM_CtrlPWMOutputs(TIM1,ENABLE); 
	//使能定时器
	TIM_Cmd(TIM1,ENABLE); 

}

/*
  Freq：72000000 / (0+1) / (1799+1) / 2 / (1+1) = 10000 HZ(T= 0.0001s = 100us) 触发ADC采样的时钟周期

  在bsp_adc.c中配置
  1.ADC时钟频率：APB2 / 6 = 72 MHz / 6 = 12 MHz(T = 1 / 12MHz ≈ 83.33ns)
  2.ADC采样时间配置：28.5个ADC周期，即采样时间 = 28.5 * 83.33ns = 2375ns = 2.375 us
  3.采样时间：28.5周期
    转换时间：12.5周期 
  4.总转换时间：28.5 + 12.5 = 41 个ADC周期，即41 * 83.33ns = 3.417 us
  5.那么矢量圆的最大半径减小为原来的 100 - 3.417 / 100 = 0.96，即从归一化后的√3/3→ √3/3*0.96 = 0.557
*/

void setPhaseVoltage_FOC2(float Uq, float Ud, float angle_el)
{
  float Uref;
	uint32_t sector;
	float T0,T1,T2;
	float Ta,Tb,Tc;
	float U_alpha,U_beta;
	
	angle_el =_normalizeAngle(angle_el);                    //电角度标准化在【0,2pi】

	U_alpha=Ud*_cos(angle_el)-Uq*_sin(angle_el);              //反park变换
	U_beta=Ud*_sin(angle_el)+Uq*_cos(angle_el);
	
	Uref=_sqrt(U_alpha*U_alpha + U_beta*U_beta) / 12;
//	Uref = Uq/24;
	if(Uref> 0.557)Uref= 0.557;                     			//六边形的内切圆(SVPWM最大不失真旋转电压矢量赋值)根号3/3
	if(Uref<-0.557)Uref=-0.557;  
	
   if(Ud <0.01 && Ud >-0.01)
    {
      if(Uq>0)
        angle_el =_normalizeAngle(angle_el + _PI_2);            //加90度后是参考电压矢量的位置
      else
		    angle_el =_normalizeAngle(angle_el - _PI_2);
    }
    else
    {
        angle_el = angle_el + atan2f(Uq,Ud);
        angle_el = _normalizeAngle(angle_el);
    }      
    
  sector = (angle_el / _PI_3) + 1;  
	T1 = _SQRT3*sin(sector*_PI_3 - angle_el) * Uref;           //计算两个相邻电压矢量作用时间
	T2 = _SQRT3*sin(angle_el - (sector-1.0)*_PI_3) * Uref;
	T0 = 1 - T1 - T2;                                          //零矢量作用时间
	
	// calculate the duty cycles(times)
	switch(sector) 
	{

		case 1:
			Ta = T0/2;
			Tb = T1+ T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 2:
			Ta = T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 3:
			Ta = T1 + T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T0/2;
			break;
    case 4:
			Ta = T1 + T2 + T0/2;
			Tb = T2 + T0/2;
			Tc = T0/2;
			break;
		case 5:
			Ta = T1 +  T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T0/2;
			break;
		case 6:
			Ta = T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T2 + T0/2;
			break;
		default:  // possible error state
			Ta = 0;
			Tb = 0;
			Tc = 0;
	}
	
  // PWM波 20kHz
	TIM_SetCompare1(TIM1,Ta*1799);
	TIM_SetCompare2(TIM1,Tb*1799);
	TIM_SetCompare3(TIM1,Tc*1799);
}
