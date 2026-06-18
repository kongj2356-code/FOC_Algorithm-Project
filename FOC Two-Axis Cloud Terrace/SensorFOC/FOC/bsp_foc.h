#ifndef __BSP_FOC_H
#define __BSP_FOC_H

#include "stm32f10x.h"  // 引入STM32核心头文件，避免枚举/类型未定义

// 电机编号枚举（适配两路电机控制）
typedef enum
{
    MOTOR_1 = 1,  // 对应TIM1（PA8/9/10）
    MOTOR_2 = 2   // 对应TIM3（PA6/7、PB0）
} Motor_NumTypeDef;


// 原有函数声明
float _normalizeAngle(float angle);

// 新增/拓展函数声明（支持两路电机控制）
void PWM1_Init(void);          // 电机1-TIM1单独初始化
void PWM3_Init(void);          // 电机2-TIM3单独初始化
void PWM_Init_Double(void);       // 全局初始化（同时初始化两路电机）
void setPhaseVoltage_FOC2(Motor_NumTypeDef motor, float Uq, float Ud, float angle_el);

/*零点对齐*/
void Zero_Get(Motor_NumTypeDef motor, uint8_t AS5600_Channel, float *Zero_Angle);
#endif
