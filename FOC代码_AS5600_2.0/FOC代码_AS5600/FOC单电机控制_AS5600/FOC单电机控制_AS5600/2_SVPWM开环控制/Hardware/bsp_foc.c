 #include "stm32f10x.h"                  // Device header
#include <math.h> 

#define _PI_2      1.57079632679   // π/2
#define _PI_3      1.0471975512    // π/3
#define _2PI       6.28318530718   // 2π
#define _3PI_2     4.71238898038   // 3π/2
#define _PI_6      0.52359877559   // π/6
#define _SQRT3     1.73205080757   // √3

float Ta,Tb,Tc;

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


uint16_t Sector;



/**
  * @brief  FOC控制的SVPWM空间矢量脉宽调制核心函数
  * @param  Ud: d轴电压分量 (磁场定向轴，通常控制为0实现id=0控制)
  * @param  Uq: q轴电压分量 (转矩控制轴，大小决定电机转矩/转速)
  * @param  angle_e1: 电机转子电角度 (由编码器/霍尔/无感算法获取)
  * @retval 无
  * @note   基于三相全桥逆变电路，输出PWM驱动电机，适配TIM1高级定时器的PWM输出
  */
void SetSVPWM_FOC(float Ud,float Uq ,float angle_e1)
{
  float U_alpha, U_beta, Uref;  // αβ静止坐标系电压分量、电压参考矢量幅值
  float T1,T2,T0;               // 有效矢量T1/T2作用时间、零矢量T0作用时间

  // 电角度归一化处理：限制电角度范围在(0,2PI)，避免角度超界导致三角函数计算异常
  angle_e1 = Angle_FOC(angle_e1);  

  // 反Park变换：将dq旋转坐标系的电压分量转换为αβ静止坐标系
  // Park变换是αβ->dq，反Park则是dq->αβ，实现旋转坐标系到静止坐标系的解耦
  U_alpha = Ud * cos(angle_e1) - Uq * sin(angle_e1);
  U_beta  = Ud * sin(angle_e1) + Uq * cos(angle_e1);  

  // 计算电压参考矢量的标幺值（归一化）
  // 12.0为当前供电电池电压，除以母线电压将实际电压转换为0~1的标幺值，消除母线电压波动影响
  Uref = sqrt(U_alpha * U_alpha + U_beta * U_beta) / 12.0; 		
  
  // 限制电压参考矢量在SVPWM**线性调制区**（内切圆内）
  // 0.577 = √3/3 ≈0.57735，是SVPWM线性调制区的最大标幺值（三相全桥最大线性输出电压限制）
  if (Uref > 0.577)      
    Uref = 0.577;
  if (Uref < -0.577)
    Uref = -0.577; 
  
  // 电压矢量相位补偿：将电角度偏移±90°(_PI_2)，匹配SVPWM的矢量计算相位要求
  // Uq>0时电机正转，角度+90°；Uq<0时电机反转，角度-90°，保证矢量合成的正确性
  if (Uq > 0)
      angle_e1 = Angle_FOC(angle_e1 + _PI_2);  // 补偿后重新归一化角度
  else
      angle_e1 = Angle_FOC(angle_e1 - _PI_2);
    
  // SVPWM扇区判断：将αβ坐标系360°(2PI)均分为6个扇区，每个扇区60°(_PI_3)
  // 计算结果为1~6，对应SVPWM的6个有效扇区
  Sector = (angle_e1 / _PI_3 ) + 1 ; 
   
  // 计算当前扇区下两个基本有效电压矢量的作用时间T1、T2
  // _SQRT3为√3，通过三角函数计算矢量作用时间，满足SVPWM的矢量合成法则
  T1 = _SQRT3 * sin(Sector * _PI_3 - angle_e1) * Uref;
  T2 = _SQRT3 * sin(angle_e1 - (Sector - 1.0) * _PI_3) * Uref;
  T0 = 1 - T1 - T2;  // 计算零矢量（000/111）的作用时间，总矢量时间归一化为1（一个PWM周期）

  // 根据当前扇区，计算三相桥臂的PWM占空比（Ta/Tb/Tc：A/B/C相的占空比标幺值）
  // SVPWM的七段式调制算法：零矢量平均分配到PWM周期首尾，减小谐波
	switch(Sector)
	{
		case 1:  // 扇区1：有效矢量为U1(100)、U2(110)
				Ta = T0 / 2;                // A相占空比=零矢量前半段
				Tb = T1 + T0 / 2;           // B相占空比=零矢量前半段+T1
				Tc = T1 + T2 + T0 / 2;      // C相占空比=零矢量前半段+T1+T2
				break;
		case 2:  // 扇区2：有效矢量为U2(110)、U3(010)
				Ta = T2 + T0 / 2;
				Tb = T0 / 2;
				Tc = T1 + T2 + T0 / 2;
				break;
		case 3:  // 扇区3：有效矢量为U3(010)、U4(011)
				Ta = T1 + T2 + T0 / 2;
				Tb = T0 / 2;
				Tc = T1 + T0 / 2;
				break;
		case 4:  // 扇区4：有效矢量为U4(011)、U5(001)
				Ta = T1 + T2 + T0 / 2;
				Tb = T2 + T0 / 2;
				Tc = T0 / 2;
				break;
		case 5:  // 扇区5：有效矢量为U5(001)、U6(101)
				Ta = T1 + T0 / 2;
				Tb = T1 + T2 + T0 / 2;
				Tc = T0 / 2;
				break;
		case 6:  // 扇区6：有效矢量为U6(101)、U1(100)
				Ta = T0 / 2;
				Tb = T1 + T2 + T0 / 2;
				Tc = T2 + T0 / 2;
				break;
		default: // 异常扇区：保护机制，关闭所有PWM输出
				Ta = Tb = Tc = 0; 
				break;
	}

  // 将占空比标幺值转换为定时器比较值(CCR)，更新TIM1的PWM占空比
  // 1440为TIM1的自动重装值(ARR)，Ta*1440即A相PWM的比较值，决定实际占空比
  TIM_SetCompare1(TIM1 , Ta*1440);  // 配置TIM1_CH1(A相)的PWM比较值
  TIM_SetCompare2(TIM1 , Tb*1440);  // 配置TIM1_CH2(B相)的PWM比较值
  TIM_SetCompare3(TIM1 , Tc*1440);  // 配置TIM1_CH3(C相)的PWM比较值
}



/*若修改为PWM模式1  TIM_OCMode_PWM1; */
