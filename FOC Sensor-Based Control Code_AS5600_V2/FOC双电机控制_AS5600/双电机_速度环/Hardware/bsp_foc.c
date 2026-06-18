#include "stm32f10x.h"                  // Device header
#include <math.h> 
#include "Delay.h"  


#define _PI_2      1.57079632679   // π/2
#define _PI_3      1.0471975512    // π/3
#define _2PI       6.28318530718   // 2π
#define _3PI_2     4.71238898038   // 3π/2
#define _PI_6      0.52359877559   // π/6
#define _SQRT3     1.73205080757   // √3

// 隔离两路电机的相电压变量
float Ta1,Tb1,Tc1;  // 电机1相电压占空比
float Ta2,Tb2,Tc2;  // 电机2相电压占空比

// 电机编号枚举（方便后续扩展）
typedef enum
{
    MOTOR_1 = 1,  // TIM1
    MOTOR_2 = 2   // TIM3
} Motor_NumTypeDef;


// 限制电角度在0 ~ 2PI区间内
float Angle_FOC(float angle)
{
    float a = fmod(angle , _2PI);
    return a >= 0 ? a : (a + _2PI);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 拆分定时器初始化：电机1-TIM1初始化
void PWM1_Init(void)
{
    GPIO_InitTypeDef          GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef   TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef         TIM_OCInitStructure;
  
    // 使能TIM1和GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE);
  
    // 配置GPIO - PA8(TIM1_CH1), PA9(TIM1_CH2), PA10(TIM1_CH3)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    // I/O口输出速度
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 定时器时基初始化
    TIM_TimeBaseInitStructure.TIM_Period = 1440;         // ARR自动重装载值（和TIM3保持一致）
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;         // PSC预分频器值
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1; // 中央对齐模式1
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;  // 时钟分频
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0; // 重复计数器（高级定时器特有）
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    // 输出比较通道配置
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;        // PWM模式2
    TIM_OCInitStructure.TIM_Pulse = 0;                       // CCR初值
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;// 输出极性高
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;    // 互补输出极性
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  // 主输出使能
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;// 互补输出使能
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;   // 空闲状态低
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset; // 互补空闲低
  
    // 初始化三个通道
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    
    // 主输出使能（高级定时器必须）
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    // 使能预装载寄存器
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    // 使能定时器
    TIM_Cmd(TIM1, ENABLE);
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
    TIM_TimeBaseInitStructure.TIM_Period = 1440;         // 和TIM1保持相同ARR，保证PWM频率一致
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;         // PSC预分频器
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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// 全局PWM初始化入口
void PWM_Init_Double(void)
{
    PWM1_Init();  // 初始化电机1-TIM1
    PWM3_Init();  // 初始化电机2-TIM3
}




// SVPWM函数新增电机编号参数，支持两路电机
void setPhaseVoltage_FOC2(Motor_NumTypeDef motor, float Uq, float Ud, float angle_el)
{
    float U_alpha, U_beta; 
    float Uref;
    uint16_t Sector;  
    float T1,T2,T0;  

    angle_el = Angle_FOC(angle_el);  // 限制电角度

    // 反Park变换
    U_alpha = Ud * cos(angle_el) - Uq * sin(angle_el);
    U_beta  = Ud * sin(angle_el) + Uq * cos(angle_el);  

    Uref = sqrt(U_alpha * U_alpha + U_beta * U_beta) / 12.0;  // 12V电源电压

    // 线性调变区限制
    if (Uref > 0.577) Uref = 0.577;
    if (Uref < -0.577) Uref = -0.577; 
  
    // 电角度相位补偿
    if (Uq > 0)
        angle_el = Angle_FOC(angle_el + _PI_2);
    else
        angle_el = Angle_FOC(angle_el - _PI_2);
    
    // SVPWM扇区判断
    Sector = (angle_el / _PI_3 ) + 1 ; 
    if (Sector > 6) Sector = 6;

    // 矢量控制时间计算
    T1 = _SQRT3 * sin(Sector * _PI_3 - angle_el) * Uref;
    T2 = _SQRT3 * sin(angle_el - (Sector - 1.0) * _PI_3) * Uref;
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
        TIM_SetCompare1(TIM1 , Ta1*1440);  
        TIM_SetCompare2(TIM1 , Tb1*1440);
        TIM_SetCompare3(TIM1 , Tc1*1440);
    }
    else if(motor == MOTOR_2)
    {
        TIM_SetCompare1(TIM3 , Ta2*1440);  
        TIM_SetCompare2(TIM3 , Tb2*1440);
        TIM_SetCompare3(TIM3 , Tc2*1440);
    }
}



