#include "stm32f10x.h"
#include "AS5600.h"
#include "bsp_pid.h"
#include "bsp_foc.h"
#include <math.h>
#include "bsp_tim.h"


/* ================== 状态变量 ================== */
static float Angle_pre = 0.0f;
static float Speed_pre = 0.0f;

float Speed;     // 角度环输出（期望速度）
float Uq;        // 速度环输出（电压）
extern float Zero_Angle;  // 零点角度（rad）
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
void BSP_TIM(void)  // 1ms中断
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;     // ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;    // PSC
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
 * @brief  将角度归一化到 0 ~ 2π 范围内
 * @param  angle: 原始角度（rad）
 * @retval 归一化后的角度（rad），范围 [0, 2π)
 */
float normalize_angle_0_2pi(float angle)
{
    // 先将角度对 2π 取模，先把超大/超小的角度“压缩”到-2π ~ 2π范围
    angle = fmodf(angle, _2PI);
    // 处理负数情况，确保最终范围在 0~2π
    if (angle < 0.0f)
    {
        angle += _2PI;
    }
    return angle;
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
    // 直接调用新增的归一化函数
    float norm_angle = normalize_angle_0_2pi(current_angle);
    
    // 四舍五入匹配最近的齿轮卡位
    int gear_idx = (int)round(norm_angle / GEAR_ANGLE_STEP) % GEAR_NUM;
    // 确保索引非负（避免负数取模问题）
    if (gear_idx < 0) gear_idx += GEAR_NUM;
    return gear_idx * GEAR_ANGLE_STEP;
}



/* ================== TIM2 中断 ================== */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        /* 1. 读取角度并归一化 */
        Angle = GetAngle();
        Angle = normalize_angle_0_2pi(Angle); // 限制Angle在[0,2π]
        Angle_ele = Angle * 7;

#if BOUNDARY_MODE_ENABLE
        /* ================== 边界模式 ================== */
        // 计算边界范围（将角度范围转换为弧度）
        float angle_range_half = ANGLE_RANGE / 2.0f;          // 半角度范围（deg）
        float limit_rad = angle_range_half / 180.0f * M_PI;   // 半角度范围（rad）
        
        // 基准角度（零点机械角度）
        float base_angle = normalize_angle_0_2pi(Zero_Angle / 7);
        // 计算上下边界并归一化（避免负数）
        float L1 = normalize_angle_0_2pi(base_angle + limit_rad);  // 上边界
        float L2 = normalize_angle_0_2pi(base_angle - limit_rad);  // 下边界

        // 标记是否在有效范围内
        uint8_t In_range = 0;
        
        /* 1.分两种情况判断：边界是否跨0点（2π）*/
        if (L1 > L2)            // 情况1：有效范围是 [L2, L1]（不跨0点）
        {
            if (Angle >= L2 && Angle <= L1)
            {
                In_range = 1;
            }
        }
        else
        {                      // 情况2：有效范围跨0点，分为 [L2, 2π) 和 [0, L1]
            if ((Angle >= L2 && Angle < _2PI) || (Angle >= 0 && Angle <= L1))
            {
                In_range = 1;
            }
        }

        /* 2.根据是否在范围内设置目标角度 */
        if (In_range)
        {
            // 在边界内，保持原有棘轮模式，自由转动
            target_detent = Calculate_Target_Detent(Angle);
        }
        else
        {
            // 越界，计算到两个边界的最短距离，拉回最近的边界
            float diff_L1 = fabs(Angle_difference(L1, Angle));
            float diff_L2 = fabs(Angle_difference(L2, Angle));
            
            if (diff_L1 < diff_L2)
            {
                // 离上边界更近，拉回上边界
                target_detent = L1;
            }
            else
            {
                // 离下边界更近，拉回下边界
                target_detent = L2;
            }
        }
#else
        /* ================== 棘轮模式 ================== */
        target_detent = Calculate_Target_Detent(Angle);
#endif

        /* 3. 原始角度误差 */
        float angle_error = Angle_difference(target_detent, Angle);

        /* 4. 软死区：吃掉误差，而不是砍输出 */
        float effective_error = 0.0f;    // 有效错误
        if (abs(angle_error) > DEAD_ZONE)
        {
            if (angle_error > 0)
                effective_error = angle_error - DEAD_ZONE;
            else
                effective_error = angle_error + DEAD_ZONE;
        }

        /* 5. 角度环（P） -> 速度期望 */
        Speed = Angle_PID(50, 0.0f, 0.0f, effective_error);

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
#if BOUNDARY_MODE_ENABLE
    // 边界模式下，初始目标角度设为零点（归一化）
    target_detent = normalize_angle_0_2pi(Zero_Angle/7);
#else
    target_detent = Calculate_Target_Detent(Angle);
#endif
}
