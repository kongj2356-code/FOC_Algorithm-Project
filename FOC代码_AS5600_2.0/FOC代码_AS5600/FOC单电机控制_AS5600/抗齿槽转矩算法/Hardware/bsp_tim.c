#include "stm32f10x.h"
#include "AS5600.h"
#include "bsp_pid.h"
#include "bsp_foc.h"
#include <math.h>

#define _PI_3    1.04719755119659f
#define PI       3.141592653589f
#define _2PI     6.28318530718f
#define abs(x) ((x)>0 ? (x) :(-x))

/* ================== 抗齿槽补偿配置 ================== */
#define COMPENSATE_POINTS      2091                         // 抗齿槽补偿的角度采样点数（一圈的离散点数）
#define ANGLE_STEP_RAD         (_2PI / COMPENSATE_POINTS)    // 每个补偿点对应的弧度步长

/* ================== 滑动平均滤波配置 ================== */
#define FILTER_WINDOW_SIZE  10                            // 滑动窗口大小（奇数更利于滤波），可根据噪声调整
static float filter_buffer[FILTER_WINDOW_SIZE] = {0};     // 滤波缓冲区（存储最近N个补偿值）
static uint8_t filter_index = 0;                          // 缓冲区写入索引（循环复用）
static float filter_sum = 0;                              // 缓冲区数据总和（优化平均值计算）

/* ================== 去均值配置 ================== */
static float compensate_mean = 0.0f;   // 补偿数组的均值
static uint8_t mean_calculated = 0;    // 均值是否已计算(=1已计算)

// Uq补偿数组
extern float Cogging_Compensate_Uq[2091];

/* ================== 状态变量 ================== */
static float Angle_pre = 0.0f;
static float Speed_pre = 0.0f;

float Speed;
float Uq;
float Uq_compensated;

extern float Speed_hope;
extern float Zero_Angle;

/* ================== 一阶低通滤波（速度） ================== */
float LPE_VELOCITY(float x)
{
    float y = 0.9f * Speed_pre + 0.1f * x; 
    Speed_pre = y;
    return y;
}

/* ================== 循环角度转换成无上限的连续角度================== */
float FOC_Angle_Continuous(float angle_raw)
{
    static float angle_foc = 0.0f;    
    static float angle_raw_last = 0.0f;

    float delta = angle_raw - angle_raw_last;

    if (delta > PI)      
        delta -= _2PI;
    else if (delta < -PI) 
        delta += _2PI;

    angle_foc += delta;
    angle_raw_last = angle_raw;

    return angle_foc;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* ==================均值计算================== */
/**
 * @brief  计算补偿数组的整体均值
 * @note   仅首次调用时计算，避免重复运算；去均值可消除补偿值的直流偏置
 */
void Calculate_Compensate_Mean(void)
{
    if (mean_calculated) 
          return;
    
    float sum = 0.0f;
    for (uint16_t i = 0; i < COMPENSATE_POINTS; i++)
    {
        sum += Cogging_Compensate_Uq[i];
    }
    compensate_mean = sum / COMPENSATE_POINTS;
    mean_calculated = 1;
}


/* ================== 滑动平均滤波函数 ================== */
/**
 * @brief  滑动平均滤波（移动平均滤波）函数
 * @note   维护一个固定长度的数值窗口，新数据加入时移除最旧数据，返回窗口内数据的平均值
 * @param  new_sample: 本次输入的新采样数据
 * @retval 滤波后的平均值
 * @attention 需提前定义全局/静态变量：
 */
float Moving_Average_Filter(float new_sample)
{
    // 减去移出窗口的旧值
    filter_sum -= filter_buffer[filter_index];
    // 添加新值到窗口
    filter_buffer[filter_index] = new_sample;
    filter_sum += new_sample;
    // 更新索引
    filter_index = (filter_index + 1) % FILTER_WINDOW_SIZE;
    // 返回平均值
    return filter_sum / FILTER_WINDOW_SIZE;
}


/* ================== 抗齿槽补偿函数================== */
float Cogging_Compensate(float current_mech_angle)
{
    // 首次调用时自动计算均值
    if (!mean_calculated)
    {
        Calculate_Compensate_Mean();
    }

    // 角度归一化至0~2π
    current_mech_angle =  Angle_FOC(current_mech_angle);

    // 根据当前角度计算对应的补偿数组索引
    uint16_t idx = (uint16_t)(current_mech_angle / ANGLE_STEP_RAD);
    idx = (idx >= COMPENSATE_POINTS) ? (COMPENSATE_POINTS - 1) : idx;
    
    // 去均值 + 滤波
    float raw_comp = Cogging_Compensate_Uq[idx] - compensate_mean; // 原始补偿值减去均值（消除直流偏置）
    float filtered_comp = Moving_Average_Filter(raw_comp);         // 滑动平均滤波（平滑补偿值，降低噪声）
    
    // 返回补偿值（可调整增益）
    return filtered_comp * 1.1f;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////




/* ================== TIM2 初始化 ================== */
void BSP_TIM()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;    // 1ms
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;   // 1MHz
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




 
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        float Angle,Angle_continuous, Angle_ele;
        float cogging_comp = 0.0f;

        Angle = GetAngle();
        Angle_ele = Angle * 7;

        Angle_continuous = FOC_Angle_Continuous(Angle);

        Speed = (Angle_continuous - Angle_pre) / 0.001f;  // rad/s
        Speed = LPE_VELOCITY(Speed);

        Uq = Speed_PID(0.1f, 0.002f, 0.0f, (Speed_hope - Speed));

        // 调用升级后的补偿函数
        cogging_comp = Cogging_Compensate(Angle);

        // 前馈补偿Uq，得到最终q轴电压
        Uq_compensated = Uq + cogging_comp;
        Uq_compensated = (Uq_compensated > 3.0f) ? 3.0f : ((Uq_compensated < -3.0f) ? -3.0f : Uq_compensated);

        setPhaseVoltage_FOC2(Uq_compensated, 0.0f, Angle_ele - Zero_Angle);

        Angle_pre = Angle_continuous;

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

