#include "./FOC/bsp_foc.h"
#include "stm32f10x.h"                  // Device header
#include "math.h"
#include "IQ_math.h"

#define _PI 3.14159265359
#define _PI_2 1.57079632679
#define _PI_3 1.0471975512
#define _2PI 6.28318530718
#define _3PI_2 4.71238898038
#define _PI_6 0.52359877559
#define _SQRT3 1.73205080757

float _normalizeAngle(float angle){
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}

void PWM_Init()
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef       TIM_OCInitStructure;
	TIM_BDTRInitTypeDef     TIM_BDTRInitStructure;

	//使能TIM1,GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA , ENABLE);
	
	//配置GPIO PA8/9/10/11 (CH1/2/3/4)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					
	
	//定时器初始化：中心对齐
	TIM_TimeBaseInitStructure.TIM_Period = 3500;               
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0;             
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0; // 清空 RCR，不用它
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);
	
	// 必须配置 BDTR（高级定时器PWM输出必备）
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStructure.TIM_DeadTime = 0;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

	// CH1/2/3 三相PWM配置
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	
	TIM_OC1Init(TIM1,&TIM_OCInitStructure);
	TIM_OC2Init(TIM1,&TIM_OCInitStructure);
	TIM_OC3Init(TIM1,&TIM_OCInitStructure);

	// ===================== 【关键】CH4 配置：用来触发ADC =====================
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;          // 匹配触发
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 必须使能
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 350;                     
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	// =======================================================================

	// 预装载使能
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); 
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable); 
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable); 
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable); // CCR4预装载

	TIM_ARRPreloadConfig(TIM1, ENABLE); 
	
	// 主输出使能
	TIM_CtrlPWMOutputs(TIM1,ENABLE); 
	TIM_Cmd(TIM1,ENABLE); 
}


void setPhaseVoltage_FOC2(float Uq, float Ud, float angle_el)
{
    float Uref;
	uint32_t sector;
	float T0,T1,T2;
	float Ta,Tb,Tc;
	float U_alpha,U_beta;
	
	angle_el =_normalizeAngle(angle_el);                    //电角度标准化在【0,2pi】

	U_alpha=Ud*_cos(angle_el)-Uq*_sin(angle_el);            //反park变换
	U_beta=Ud*_sin(angle_el)+Uq*_cos(angle_el);
	
	Uref=_sqrt(U_alpha*U_alpha + U_beta*U_beta) / 24;
//	Uref = Uq/24;
	if(Uref> 0.40)Uref= 0.4;                     			//六边形的内切圆(SVPWM最大不失真旋转电压矢量赋值)根号3/3
	if(Uref<-0.40)Uref=-0.4; 
	
   if(Ud <0.01 && Ud >-0.01)
    {
      if(Uq>0)
        angle_el =_normalizeAngle(angle_el + _PI_2);            //加90度后是参考电压矢量的位置
      else
		angle_el =_normalizeAngle(angle_el-_PI_2);
    }
    else
    {
        angle_el = angle_el + atan2f(Uq,Ud);
        angle_el = _normalizeAngle(angle_el);
    }      
    
    sector = (angle_el / _PI_3) + 1;  
	T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uref;           //计算两个相邻电压矢量作用时间
	T2 = _SQRT3*_sin(angle_el - (sector-1.0)*_PI_3) * Uref;
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
	
	TIM_SetCompare1(TIM1,Ta*3500);
	TIM_SetCompare2(TIM1,Tb*3500);
	TIM_SetCompare3(TIM1,Tc*3500);
}
