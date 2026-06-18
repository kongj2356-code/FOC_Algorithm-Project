#include "stm32f10x.h"
#include "AS5600.h"
#include "bsp_pid.h"
#include "bsp_foc.h"

#define M_PI     3.14159265358979f
#define _PI_3    1.04719755119659f    // π/3
#define PI       3.141592653589f
#define _2PI     6.28318530718f
#define abs(x) ((x)>0 ? (x) :(-x))
/* ================== 状态变量 ================== */


static float Angle_pre = 0.0f;
static float Speed_pre = 0.0f;

float Speed;
float Uq;

extern float Zero_Angle;
extern  float Angle_hope;


extern float Angle, Angle_ele;

/* ================== 一阶低通 ================== */
float LPF_VELOCITY(float x)
{
    float y = 0.9 * Speed_pre + 0.1 * x; 
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
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;   // 1ms
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;  // 1MHz
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


// 机械角解卷绕
float Angle_difference(float target,float current)
{
	float diff = target - current;
  while(diff > M_PI)  diff -= 2 * M_PI;
  while(diff < -M_PI) diff += 2 * M_PI;
	return diff;
}


/* ================== TIM2中断服务函数 ================== */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {	
       
        Angle = GetAngle();
        Angle_ele = Angle * 7;
        

        float angle_error = Angle_difference(Angle_hope, Angle);
        Speed = Angle_PID(15.0f, 0.0f, 0.0f, angle_error);
        

        float actual_speed = (Angle - Angle_pre) ; 
        actual_speed = LPF_VELOCITY(actual_speed);
     
        float speed_error = Speed - actual_speed;
        Uq = Speed_PID(0.38f, 0.0001f, 0.0f, speed_error);
        
        // 7. FOC控制
        setPhaseVoltage_FOC2(Uq, 0.0f, (Angle_ele - Zero_Angle));
        
        // 8. 更新状态
        Speed_pre = actual_speed;  // 保存实际速度供下次使用
        Angle_pre = Angle;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        

    }
}


