#include "stm32f10x.h"
#include "bsp_foc.h"
#include "AS5047P/bsp_as5047p.h"
#include "./PID/bsp_pid.h"


#define _PI_3    1.04719755119659f    // π/3
#define PI    3.141592653589f
#define _2PI  6.28318530718f
#define abs(x) ((x)>0 ? (x) :(-x))
/* ================== 状态变量 ================== */

static float Angle_pre = 0.0f;
static float Speed_pre = 0.0f;

float Speed;
float Uq;

extern float Speed_hope;
extern float Zero_Angle;

extern float measure;

/* ================== 一阶低通 ================== */
float LPE_VELOCITY(float x)
{
    float y = 0.9 * Speed_pre + 0.1 * x; 
    Speed_pre = y;
    return y;
}

/* ================== TIM2 初始化 ================== */
// 配置TIM2作为1ms定时中断（1000Hz）
void BSP_TIM()
{
    // 1. 使能TIM2时钟（TIM2挂载在APB1总线）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // 2. 配置TIM2使用内部时钟源
    TIM_InternalClockConfig(TIM2);
    
    // 3. 配置时基单元参数
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;   // 向上计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;       // 时钟分频因子（不分频）
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;                 // 自动重装载值ARR：1000个计数
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;                // 预分频器PSC：72分频
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;             // 重复计数器（高级定时器用）
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    
    // 4. 清除更新中断标志，防止首次进入中断
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    
    // 5. 使能TIM2更新中断（达到ARR值时触发）
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    
    // 6. 配置NVIC中断优先级分组（分组2：2位抢占优先级+2位响应优先级）
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    // 7. 配置TIM2中断通道
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;               // TIM2中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     // 抢占优先级2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // 子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               // 使能中断通道
    NVIC_Init(&NVIC_InitStructure);
    
    // 8. 使能TIM2计数器
    TIM_Cmd(TIM2, ENABLE);
}



void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        float Angle, Angle_data, Angle_ele;
        /* ===== 读取编码器角度 ===== */
        Angle =AS5047P_Read();
        Angle_ele = Angle * 7;

        /* ===== 连续角度处理 ===== */
        if ( Angle_pre- Angle > 0.8 * _2PI)
        {
            Angle_data = Angle + _2PI;
        }
        else if(Angle - Angle_pre > 0.8 * _2PI)
				{
            Angle_data = Angle - _2PI;
				}
        else
        {
            Angle_data = Angle;
        }

        /* ===== 速度计算 ===== */
        Speed = (Angle_data - Angle_pre) / 0.001f;  // 1ms采样
        Speed = LPE_VELOCITY(Speed);                // 一阶低通滤波

        /* ===== 速度闭环 PID ===== */
        Uq = Speed_PID(0.3f, 0.001f, 0.0f, (Speed_hope - Speed));

        /* ===== FOC 电角度 ===== */
        setPhaseVoltage_FOC2(Uq, 0.0f,Angle_ele - Zero_Angle );
        /* ===== 更新上一拍角度 ===== */
        Angle_pre = Angle;
        Speed_pre = Speed;

        /* ===== 清中断标志 ===== */
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

