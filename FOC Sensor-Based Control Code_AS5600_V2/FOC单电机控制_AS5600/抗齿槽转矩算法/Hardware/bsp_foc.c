#include "stm32f10x.h"                  // Device header
#include <math.h> 
#include <Delay.h>
#define _PI_2      1.57079632679   // π/2
#define _PI_3      1.0471975512    // π/3
#define _2PI       6.28318530718   // 2π
#define _3PI_2     4.71238898038   // 3π/2
#define _PI_6      0.52359877559   // π/6
#define _SQRT3     1.73205080757   // √3


extern float Ta,Tb,Tc;

// 限制电角度在0 ~ 2PAI区间内
float Angle_FOC(float angle)
{
	float a = fmod(angle , _2PI);
    return a >= 0 ? a : (a + _2PI);
}


// 使用高级定时器1初始化PWM--采用ARR中央对齐模式+PWM模式2
void PWM_Init()
{
    GPIO_InitTypeDef          GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef   TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef         TIM_OCInitStructure;
  
    // 使能TIM1和GPIOA时钟
    // 注意：AFIO时钟只有在重映射、外部中断配置、事件控制时才需要使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE);
  
    // 配置GPIO - PA8(TIM1_CH1), PA9(TIM1_CH2), PA10(TIM1_CH3)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    // I/O口输出速度
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 定时器时基初始化
    TIM_TimeBaseInitStructure.TIM_Period = 1440;         // ARR自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;         // PSC预分频器值
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1; // 中央对齐模式1  *
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;  // 时钟分频，TDTS = Tck_tim
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0; // 重复计数器，高级定时器特有
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    // 输出比较通道配置
    // 注意：高级定时器的输出比较结构体必须配置所有成员
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;        // PWM模式2  *
    TIM_OCInitStructure.TIM_Pulse = 0;                       // CCR捕获比较寄存器初值，决定占空比
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      // 输出极性：高电平有效  *
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;    // 互补输出极性：高电平有效
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  // 主输出使能
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;// 互补输出使能
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;   // 空闲状态：输出低电平
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset; // 互补空闲状态：输出低电平
  
    // 初始化三个通道
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);  // 通道1
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);  // 通道2
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);  // 通道3
    
    // 主输出使能 - 高级定时器必须调用此函数才能输出PWM
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    // 使能预装载寄存器 - 确保参数更新在下一个更新事件生效
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  // 通道1预装载
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  // 通道2预装载
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);  // 通道3预装载
    TIM_ARRPreloadConfig(TIM1, ENABLE);                // ARR预装载

    // 使能定时器
    TIM_Cmd(TIM1, ENABLE);
}



void SetSVPWM_FOC(float Uq,float Ud ,float angle_e1)
{
  float U_alpha, U_beta; 
  float Uref;
  uint16_t Sector;  
  float T1,T2,T0;  

  angle_e1 = Angle_FOC(angle_e1);  //限制电角度在(0,2PI)

  //反Park
  U_alpha = Ud * cos(angle_e1) - Uq * sin(angle_e1);
  U_beta  = Ud * sin(angle_e1) + Uq * cos(angle_e1);  

  Uref = sqrt(U_alpha * U_alpha + U_beta * U_beta) / 12.0;  // 12为当前使用电池的电压
		
  // 线性调变区(内切圆内)
   if (Uref > 0.577)       // 0.577 =  √3 / 3
     Uref = 0.577;
    if (Uref < -0.577)
     Uref = -0.577; 
  
  // 在理想情况下，当我们需要最大转矩时，我们会设置Ud=0，Uq=电压幅值
  // 但实际SVPWM算法期望的电压矢量角度与d-q坐标系中的角度有90度相位差
  // 通过外部磁编码器获取机械角度；再由机械角度换算回电气角度
  if (Uq > 0)
      angle_e1 = Angle_FOC(angle_e1 + _PI_2);
  else
      angle_e1 = Angle_FOC(angle_e1 - _PI_2);
    
  // SVPWM扇区判断
  Sector = (angle_e1 / _PI_3 ) + 1 ; 
  if (Sector > 6) Sector = 6;
  // 矢量控制时间
  T1 = _SQRT3 * sin(Sector * _PI_3 - angle_e1) * Uref;
  T2 = _SQRT3 * sin(angle_e1 - (Sector - 1.0) * _PI_3) * Uref;
  T0 = 1 - T1 - T2;  // 000 111 共同作用时间

  // 计算三相占空比
	switch(Sector)
		{
				case 1:
						Ta = T0 / 2;
						Tb = T1 + T0 / 2;
						Tc = T1 + T2 + T0 / 2;
						break;
						
				case 2:
						Ta = T2 + T0 / 2;
						Tb = T0 / 2;
						Tc = T1 + T2 + T0 / 2;
						break;
						
				case 3:
						Ta = T1 + T2 + T0 / 2;
						Tb = T0 / 2;
						Tc = T1 + T0 / 2;
						break;
						
				case 4:
						Ta = T1 + T2 + T0 / 2;
						Tb = T2 + T0 / 2;
						Tc = T0 / 2;
						break;
						
				case 5:
						Ta = T1 + T0 / 2;
						Tb = T1 + T2 + T0 / 2;
						Tc = T0 / 2;
						break;
						
				case 6:
						Ta = T0 / 2;
						Tb = T1 + T2 + T0 / 2;
						Tc = T2 + T0 / 2;
						break;
						
				default:
						// Possible error state
						Ta = 0;
						Tb = 0;
						Tc = 0; 
	          break;
						
		}

     // 修改CCR的值
     TIM_SetCompare1(TIM1 , Ta*1440);  
     TIM_SetCompare2(TIM1 , Tb*1440);
	   TIM_SetCompare3(TIM1 , Tc*1440);

}


/*若修改为PWM模式1  TIM_OCMode_PWM1; */


/**
 * @brief 设置相电压的FOC控制函数
 * @param Uq: 转矩分量（q轴电压），控制电机转矩
 * @param Ud: 磁通分量（d轴电压），控制电机磁通
 * @param angle_el: 电气角度（弧度），表示转子当前的电角度位置
 * @note 电气角度 = 机械角度 × 极对数
 */
void setPhaseVoltage_FOC2(float Uq, float Ud, float angle_el)
{
    // 调用SVPWM函数生成PWM信号
    SetSVPWM_FOC(Uq, Ud, angle_el);
}



