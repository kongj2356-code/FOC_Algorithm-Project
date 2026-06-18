#include "stm32f10x.h"
#include "AS5600.h"
#include "bsp_pid.h"
#include "bsp_foc.h"

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



/* ================== 一阶低通 ================== */
float LPE_VELOCITY(float x)
{
    float y = 0.9 * Speed_pre + 0.1* x; 
    Speed_pre = y;
    return y;
}

/* ================== TIM2 初始化 ================== */
void BSP_TIM()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_InternalClockConfig(TIM2);

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



/**
 * @brief  将FOC电机的原始角度值转换为连续无跳变的角度值
 * @param  angle_raw: 原始角度值（范围通常为0~2π，会在2π处回零）
 * @retval 连续递增/递减的角度值（无2π跳变）
 */
float FOC_Angle_Continuous(float angle_raw)
{
    static float angle_foc = 0.0f;    
    static float angle_raw_last = 0.0f;

    float delta = angle_raw - angle_raw_last;  // 计算角度差值

    // 处理2π周期回绕：差值超π则修正
    if (delta >  _PI_3 * 3)      
        delta -= _2PI;           // 正向跳变，修正差值
    else if (delta < -_PI_3 * 3) 
        delta += _2PI;           // 反向跳变，修正差值

    angle_foc += delta;          // 累加得到连续角度
    angle_raw_last = angle_raw;  // 更新上一次角度

    return angle_foc;            // 返回连续FOC角度
}



float Angle;
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
       GPIO_WriteBit(GPIOA, GPIO_Pin_4, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4));
        float  Angle_data, Angle_ele;
        /* ===== 读取编码器角度 ===== */
        Angle = GetAngle();
        Angle_ele = Angle * 7;    //电气角获取

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
        Uq = Speed_PID(0.1f, 0.002f, 0.0f, (Speed_hope - Speed));

				float angle_cont = FOC_Angle_Continuous(Angle_ele - Zero_Angle);
        /* ===== FOC 电角度 ===== */
        setPhaseVoltage_FOC2(Uq, 0.0f,angle_cont );
        /* ===== 更新上一拍角度 ===== */
        Angle_pre = Angle;
        Speed_pre = Speed;

        /* ===== 清中断标志 ===== */
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

