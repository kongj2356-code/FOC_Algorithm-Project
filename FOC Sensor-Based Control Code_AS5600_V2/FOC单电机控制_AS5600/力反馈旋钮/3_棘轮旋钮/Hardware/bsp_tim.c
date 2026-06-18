#include "stm32f10x.h"
#include "AS5600.h"
#include "bsp_pid.h"
#include "bsp_foc.h"
#include <math.h>
#include "bsp_tim.h"

#define M_PI     3.14159265358979f
#define PI       3.141592653589f
#define _2PI     6.28318530718f
#define abs(x) ((x)>0 ? (x) :(-(x)))


/* ================== 状态变量（接口保持不变） ================== */
static float Angle_pre = 0.0f;
static float Speed_pre = 0.0f;

float Speed;     // 角度环输出（期望速度）
float Uq;        // 速度环输出（电压）
extern float Zero_Angle;
extern float Angle, Angle_ele;

/* 目标档位角度 */
static float target_detent = 0.0f;

/* ================== 一阶低通滤波（速度） ================== */
float LPF_VELOCITY(float x)
{
    float y = 0.9f * Speed_pre + 0.1f * x;
    Speed_pre = y;
    return y;
}

/* ================== TIM2 初始化 ================== */
void BSP_TIM(void)  // 1MHz(1ms)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;     //ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;    //PSC
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief  计算角度差值（最短旋转方向）
 * @note   输出差值范围[-π, π)，表示从current到target的最短旋转角度
 * @param  target: 目标角度（rad）
 * @param  current: 当前角度（rad）
 * @retval 角度差值（rad），正负表示旋转方向
 */
float Angle_difference(float target, float current)
{
    float diff = target - current;  // 原始角度差值
    // 调整差值至[-π, π)，取最短旋转路径
    while (diff >  M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}


/**
 * @brief  计算目标齿轮卡位角度（将当前角度匹配到最近的齿轮卡位）
 * @note   核心功能：角度归一化 + 最近卡位匹配，适用于旋转机构的定位控制
 * @param  current_angle: 当前角度（rad）
 * @retval 匹配后的目标卡位角度（rad），范围：[0, 2π)
 */
float Calculate_Target_Detent(float current_angle)
{    
    // 角度归一化
    float norm_angle = current_angle;
    while (norm_angle < 0)        norm_angle += 2 * M_PI;
    while (norm_angle >= 2*M_PI)  norm_angle -= 2 * M_PI;
    // 四舍五入匹配最近的齿轮卡位
    int gear_idx = (int)round(norm_angle / GEAR_ANGLE_STEP) % GEAR_NUM;
    return gear_idx * GEAR_ANGLE_STEP;
}



/* ================== TIM2 中断 ================== */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        /* 1. 读取角度 */
        Angle = GetAngle();
        Angle_ele = Angle * 7.0f;

        /* 2. 目标档位 */
        target_detent = Calculate_Target_Detent(Angle);

        /* 3. 原始角度误差 */
        float angle_error = Angle_difference(target_detent, Angle);

        /* 4. 软死区：吃掉误差，而不是砍输出 */
        float effective_error;    // 有效错误
        if (abs(angle_error) < DEAD_ZONE)
        {
            effective_error = 0.0f;
        }
        else
        {
            if (angle_error > 0)
                effective_error = angle_error - DEAD_ZONE;
            else
                effective_error = angle_error + DEAD_ZONE;
        }

        /* 5. 角度环（P） -> 速度期望 */
        Speed = Angle_PID(30.0f, 0.0f, 0.0f, effective_error);

        /* 6. 低通滤波后的实际速度*/
        float actual_speed = Angle - Angle_pre;
        actual_speed = LPF_VELOCITY(actual_speed);

        /* 7. 速度环（PI） -> Uq */
        float speed_error = Speed - actual_speed;
        Uq = Speed_PID(0.3f, 100.0f, 0.0f, speed_error);

        /* 8. FOC 输出 */
        setPhaseVoltage_FOC2(Uq, 0.0f, (Angle_ele - Zero_Angle));

        /* 9. 保存历史量 */
        Angle_pre = Angle;
        Speed_pre = actual_speed;

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

/* ================== 初始化函数 ================== */
void Ratchet_Knob8_Init(void)
{
    BSP_TIM();
    Angle = GetAngle();
    Angle_pre = Angle;
    Speed_pre = 0.0f;
    target_detent = Calculate_Target_Detent(Angle);
}


