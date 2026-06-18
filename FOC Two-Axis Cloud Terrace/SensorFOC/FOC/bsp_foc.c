#include "./FOC/bsp_foc.h"
#include "stm32f10x.h"                  // Device header
#include "math.h"
#include "IQ_math.h"
#include "Delay.h"
#include "AS5600/AS5600.h"

#define _PI_2      1.57079632679   // π/2
#define _PI_3      1.0471975512    // π/3
#define _2PI       6.28318530718   // 2π
#define _3PI_2     4.71238898038   // 3π/2
#define _PI_6      0.52359877559   // π/6
#define _SQRT3     1.73205080757   // √3

// 隔离两路电机的相电压变量
float Ta1,Tb1,Tc1;  // 电机1相电压占空比
float Ta2,Tb2,Tc2;  // 电机2相电压占空比



// 限制电角度在0 ~ 2PI区间内
float _normalizeAngle(float angle)
{
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 拆分定时器初始化：电机1-TIM1初始化
void PWM1_Init(void)
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


// 新增电机2-TIM3初始化
void PWM3_Init(void)
{
    GPIO_InitTypeDef          GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef   TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef         TIM_OCInitStructure;
  
    // 使能TIM3和GPIOA时钟（TIM3_CH1=PA6, CH2=PA7, CH3=PB0，根据硬件调整）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    // TIM3属于APB1总线
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
  
    // 配置GPIO - TIM3_CH1(PA6), CH2(PA7), CH3(PB0)
    // CH1/CH2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // CH3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 定时器时基初始化（通用定时器无重复计数器）
    TIM_TimeBaseInitStructure.TIM_Period = 1800 - 1;         // 和TIM1保持相同ARR，保证PWM频率一致
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;         // PSC预分频器
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1; // 中央对齐模式1
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    // 通用定时器无TIM_RepetitionCounter，无需配置
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    // 输出比较通道配置（通用定时器无互补输出/空闲状态）
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;        // PWM模式2（和TIM1一致）
    TIM_OCInitStructure.TIM_Pulse = 0;                       // CCR初值
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;// 输出极性高
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  // 主输出使能
    // 通用定时器无需配置：TIM_OCNPolarity/TIM_OutputNState/TIM_OCIdleState/TIM_OCNIdleState

    // 初始化三个通道
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    
    // 通用定时器无需TIM_CtrlPWMOutputs

    // 使能预装载寄存器
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    // 使能定时器
    TIM_Cmd(TIM3, ENABLE);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// 全局PWM初始化入口
void PWM_Init_Double(void)
{
    PWM1_Init();  // 初始化电机1-TIM1
    PWM3_Init();  // 初始化电机2-TIM3
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

// SVPWM函数新增电机编号参数，支持两路电机
void setPhaseVoltage_FOC2(Motor_NumTypeDef motor, float Uq, float Ud, float angle_el)
{
    float U_alpha, U_beta; 
    float Uref;
    uint16_t Sector;  
    float T1,T2,T0;  

    angle_el =_normalizeAngle(angle_el);                      //电角度标准化在【0,2pi】

   	U_alpha=Ud*_cos(angle_el)-Uq*_sin(angle_el);              //反park变换
	  U_beta=Ud*_sin(angle_el)+Uq*_cos(angle_el); 

    Uref = sqrt(U_alpha * U_alpha + U_beta * U_beta) / 12.0f;  // 12V电源电压

    // 线性调变区限制
    if (Uref > 0.577f) Uref = 0.577f;
    if (Uref < -0.577f) Uref = -0.577f; 
  
    if(Ud <0.01f && Ud >-0.01f)
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
    
    // SVPWM扇区判断
    Sector = (angle_el / _PI_3 ) + 1 ; 

    // 矢量控制时间计算
    T1 = _SQRT3 * _sin(Sector * _PI_3 - angle_el) * Uref;
    T2 = _SQRT3 * _sin(angle_el - (Sector - 1) * _PI_3) * Uref;
    T0 = 1 - T1 - T2;

    // 根据电机编号计算对应相电压占空比
    switch(Sector)
    {
        case 1:
            if(motor == MOTOR_1){Ta1=T0/2; Tb1=T1+T0/2; Tc1=T1+T2+T0/2;}
            if(motor == MOTOR_2){Ta2=T0/2; Tb2=T1+T0/2; Tc2=T1+T2+T0/2;}
            break;
        case 2:
            if(motor == MOTOR_1){Ta1=T2+T0/2; Tb1=T0/2; Tc1=T1+T2+T0/2;}
            if(motor == MOTOR_2){Ta2=T2+T0/2; Tb2=T0/2; Tc2=T1+T2+T0/2;}
            break;
        case 3:
            if(motor == MOTOR_1){Ta1=T1+T2+T0/2; Tb1=T0/2; Tc1=T1+T0/2;}
            if(motor == MOTOR_2){Ta2=T1+T2+T0/2; Tb2=T0/2; Tc2=T1+T0/2;}
            break;
        case 4:
            if(motor == MOTOR_1){Ta1=T1+T2+T0/2; Tb1=T2+T0/2; Tc1=T0/2;}
            if(motor == MOTOR_2){Ta2=T1+T2+T0/2; Tb2=T2+T0/2; Tc2=T0/2;}
            break;
        case 5:
            if(motor == MOTOR_1){Ta1=T1+T0/2; Tb1=T1+T2+T0/2; Tc1=T0/2;}
            if(motor == MOTOR_2){Ta2=T1+T0/2; Tb2=T1+T2+T0/2; Tc2=T0/2;}
            break;
        case 6:
            if(motor == MOTOR_1){Ta1=T0/2; Tb1=T1+T2+T0/2; Tc1=T2+T0/2;}
            if(motor == MOTOR_2){Ta2=T0/2; Tb2=T1+T2+T0/2; Tc2=T2+T0/2;}
            break;
        default:
            if(motor == MOTOR_1){Ta1=0; Tb1=0; Tc1=0;}
            if(motor == MOTOR_2){Ta2=0; Tb2=0; Tc2=0;}
            break;
    }

    // 根据电机编号更新对应定时器的CCR值
    if(motor == MOTOR_1)
    {
        TIM_SetCompare1(TIM1 , Ta1*1799);  
        TIM_SetCompare2(TIM1 , Tb1*1799);
        TIM_SetCompare3(TIM1 , Tc1*1799);
    }
    else if(motor == MOTOR_2)
    {
        TIM_SetCompare1(TIM3 , Ta2*1799);  
        TIM_SetCompare2(TIM3 , Tb2*1799);
        TIM_SetCompare3(TIM3 , Tc2*1799);
    }
}



/* 零点对齐
 * 用开环电压把电机转子拖到一个已知电角度位置，然后读取 AS5600 当前角度，
 * 把它作为电角度零点偏移量保存下来，以后编码器读出来的角度，都要减掉这个零偏，才能和 FOC 的电角度坐标对齐
 */
void Zero_Get(Motor_NumTypeDef motor, uint8_t AS5600_Channel, float *Zero_Angle)
{
    float Angle;

    for(int i = 0; i < 200; i++)
    {
        Angle = _3PI_2 + _2PI * i / 200.0f;
        setPhaseVoltage_FOC2(motor, 4, 0, Angle);
        Delay_ms(1);
    }

    for(int i = 200; i >= 0; i--)
    {
        Angle = _3PI_2 + _2PI * i / 200.0f;
        setPhaseVoltage_FOC2(motor, 4, 0, Angle);
        Delay_ms(6);
    }

    *Zero_Angle = GetAngle(AS5600_Channel) * 7;
     
    setPhaseVoltage_FOC2(motor, 0, 0, 0);

}

