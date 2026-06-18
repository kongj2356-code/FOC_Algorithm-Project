#include "bsp_foc.h"
#include "tim.h"          // 包含CubeMX生成的tim.h（必须）
#include "math.h"

// 数学常量定义
#define _PI_2      1.57079632679f   // π/2
#define _PI_3      1.0471975512f    // π/3
#define _2PI       6.28318530718f   // 2π
#define _3PI_2     4.71238898038f   // 3π/2
#define _PI_6      0.52359877559f   // π/6
#define _SQRT3     1.73205080757f   // √3



// 全局变量（按需使用，建议尽量改为局部变量或静态变量）
float Ta, Tb, Tc;  // 改为静态变量，减少全局变量污染

/**
 * @brief 限制电角度在0 ~ 2π区间内
 * @param angle 输入电角度（弧度）
 * @return 归一化后的电角度
 */
static float Angle_FOC(float angle)
{
    float a = fmodf(angle, _2PI);  // 使用fmodf适配浮点型，避免精度问题
    return a >= 0 ? a : (a + _2PI);
}



/**
 * @brief 启动TIM1的PWM输出（HAL库适配）
 * @note 基于CubeMX生成的TIM1初始化，仅需启动PWM输出
 */
void FOC_PWM_Start(void)
{
    // 直接开启TIM1和PWM输出（绕开HAL库）
    TIM1->CR1 |= TIM_CR1_CEN; // 使能TIM1计数器
    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E); // 使能CH1/2/3输出
    TIM1->BDTR |= TIM_BDTR_MOE; // 强制开启主输出（MOE位）
    // 开启预装载
    TIM1->CCMR1 |= (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM1->CR1 |= TIM_CR1_ARPE;
}


/**
 * @brief SVPWM算法实现（HAL库适配）
 * @param Uq q轴电压（转矩分量）
 * @param Ud d轴电压（磁通分量）
 * @param angle_el 电气角度（弧度）
 * @note 核心算法逻辑不变，仅替换PWM占空比设置接口
 */
void setPhaseVoltage_FOC2(float Uq, float Ud, float angle_el)
{
    float U_alpha, U_beta, Uref; 
    uint16_t Sector;  
    float T1, T2, T0; 
 

    // 1. 角度归一化（防止溢出）
    angle_el = Angle_FOC(angle_el);

    // 2. 反Park变换（算法逻辑不变）
    U_alpha = Ud * cos(angle_el) - Uq * sin(angle_el);
    U_beta  = Ud * sin(angle_el) + Uq * cos(angle_el);  

    // 3. 参考电压计算（不变）
    Uref = sqrtf(U_alpha * U_alpha + U_beta * U_beta) / 12.0f;

    if (Uref > 0.577)      
    Uref = 0.577;
    if (Uref < -0.577)
    Uref = -0.577; 
  
   // 电压矢量相位补偿：将电角度偏移±90°(_PI_2)，匹配SVPWM的矢量计算相位要求
  // Uq>0时电机正转，角度+90°；Uq<0时电机反转，角度-90°，保证矢量合成的正确性
  if (Uq > 0)
      angle_el = Angle_FOC(angle_el + _PI_2);  // 补偿后重新归一化角度
  else
      angle_el = Angle_FOC(angle_el - _PI_2);
    
  // SVPWM扇区判断：将αβ坐标系360°(2PI)均分为6个扇区，每个扇区60°(_PI_3)
  // 计算结果为1~6，对应SVPWM的6个有效扇区
  Sector = (angle_el / _PI_3 ) + 1 ; 
   
  // 计算当前扇区下两个基本有效电压矢量的作用时间T1、T2
  // _SQRT3为√3，通过三角函数计算矢量作用时间，满足SVPWM的矢量合成法则
  T1 = _SQRT3 * sin(Sector * _PI_3 - angle_el) * Uref;
  T2 = _SQRT3 * sin(angle_el - (Sector - 1.0) * _PI_3) * Uref;
  T0 = 1 - T1 - T2;  // 计算零矢量（000/111）的作用时间，总矢量时间归一化为1（一个PWM周期）


    // 7. 占空比计算（不变）
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
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(Ta * 1440));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)(Tb * 1440));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint16_t)(Tc * 1440));
 
}


