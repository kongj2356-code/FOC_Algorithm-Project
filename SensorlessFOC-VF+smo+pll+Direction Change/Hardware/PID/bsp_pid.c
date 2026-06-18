#include "./PID/bsp_pid.h"
#include "stm32f10x.h"
#include "Delay.h"
#include "Serial.h"
#include <stdint.h>

#define _constrain(x, low, high)   ((x) < (low) ? (low) : ((x) > (high) ? (high) : (x)))

#define CUR_OUT_LIMIT    6.3f
#define CUR_I_LIMIT      6.3f

#define SPD_OUT_LIMIT    2.0f
#define SPD_I_LIMIT      1.0f
#define SPD_TS           0.001f   // 你的速度环在 TIM2 里 1ms 跑一次

typedef struct
{
    float last_error;
    float integral;
    float last_output;
    uint32_t ts_last;
} PID_State_t;

/* d轴、q轴电流环各自独立状态 */
static PID_State_t pid_d = {0};
static PID_State_t pid_q = {0};

/* 速度环独立积分状态 */
static float speed_integral = 0.0f;


/* 内部：单个PID计算 */
static float PID_Run(PID_State_t *pid, float Kp, float Ki, float Kd, float error)
{
    float Ts = 0.001f;   // 默认兜底 1ms
    uint32_t now = SysTick->VAL;

    /* 第一次运行时，直接用默认Ts */
    if (pid->ts_last != 0u)
    {
        /* 这里默认 SysTick 时钟 = 9MHz，即 72MHz / 8
           如果你的 SysTick 配置不是这个，需要把 9.0f 改掉 */
        if (now < pid->ts_last)
        {
            Ts = (float)(pid->ts_last - now) / 9.0f * 1e-6f;
        }
        else
        {
            Ts = (float)(0xFFFFFFu - now + pid->ts_last) / 9.0f * 1e-6f;
        }

        if (Ts <= 0.0f || Ts > 0.01f)
        {
            Ts = 0.001f;
        }
    }

    /* P */
    float proportional = Kp * error;

    /* I */
    pid->integral += Ki * error * Ts;
    pid->integral = _constrain(pid->integral, -CUR_I_LIMIT, CUR_I_LIMIT);

    /* D */
    float differential = 0.0f;
    if (Ts > 1e-6f)
    {
        differential = Kd * (error - pid->last_error) / Ts;
    }

    /* 输出 */
    float output = proportional + pid->integral + differential;
    output = _constrain(output, -CUR_OUT_LIMIT, CUR_OUT_LIMIT);

    /* 状态更新 */
    pid->last_error = error;
    pid->last_output = output;
    pid->ts_last = now;

    return output;
}


/* axis = 0: d轴, axis = 1: q轴 */
float PID_Controller(uint8_t axis, float Kp, float Ki, float Kd, float Error)
{
    if (axis == PID_D_AXIS)
    {
        return PID_Run(&pid_d, Kp, Ki, Kd, Error);
    }
    else
    {
        return PID_Run(&pid_q, Kp, Ki, Kd, Error);
    }
}


/* 速度环：你这里本来就在 TIM2 1ms 中断里跑，所以直接显式用1ms */
float Speed_PID(float Kp, float Ki, float Kd, float error)
{
    (void)Kd;   // 当前版本先不用D

    float proportional = Kp * error;

    speed_integral += Ki * error * SPD_TS;
    speed_integral = _constrain(speed_integral, -SPD_I_LIMIT, SPD_I_LIMIT);

    float output = proportional + speed_integral;
    output = _constrain(output, -SPD_OUT_LIMIT, SPD_OUT_LIMIT);

    return output;
}


/* 切换前/启动前统一清空 */
void PID_Clear_Integral(void)
{
    pid_d.last_error = 0.0f;
    pid_d.integral   = 0.0f;
    pid_d.last_output = 0.0f;
    pid_d.ts_last    = 0u;

    pid_q.last_error = 0.0f;
    pid_q.integral   = 0.0f;
    pid_q.last_output = 0.0f;
    pid_q.ts_last    = 0u;

    speed_integral   = 0.0f;
}
