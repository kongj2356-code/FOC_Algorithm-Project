#include "./PID/bsp_pid.h"
#include "stm32f10x.h"

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define limit 6.3f

/*PID参数初始化*/
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float integral_limit,float output_limit)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_output = 0.0f;

    pid->timestamp_last = SysTick->VAL;

    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
}

/* PID状态复位（清零历史数据）*/
void PID_Reset(PID_TypeDef *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_output = 0.0f;
    pid->timestamp_last = SysTick->VAL;
}


/* 电流环PID：独立状态版本 */
float PID_Controller_State(PID_TypeDef *pid, float Error)
{
    float Ts = 0.0f;                              // PID计算时间间隔（单位：秒）
    uint32_t Timestamp = SysTick->VAL;            // 当前系统定时器值

    float proportion;        // 比例项
    float differential;      // 微分项
    float Output;            // 最终输出

     // 根据SysTick递减特性，计算两次调用的时间差
    if(Timestamp < pid->timestamp_last)
    {
        Ts = (float)(pid->timestamp_last - Timestamp) / 9.0f * 1e-6f;  //秒
    }
    else
    {
        // 定时器溢出情况（0xFFFFFF是24位最大值）
        Ts = (float)(0xFFFFFF - Timestamp + pid->timestamp_last) / 9.0f * 1e-6f;
    }

    if(Ts <= 0.0f || Ts > 0.05f)
    {
        // 时间异常保护：防止时间间隔为0或过大，默认1ms
        Ts = 0.001f;
    }
    // 1. 比例项计算
    proportion = pid->Kp * Error;
    // 2. 积分项计算（梯形积分，更平滑）+ 积分限幅
    pid->integral = pid->integral + pid->Ki * 0.5f * Ts * Error;
    pid->integral = _constrain(pid->integral, -pid->integral_limit, pid->integral_limit);
    // 3. 微分项计算（误差微分）
    differential = pid->Kd * (Error - pid->last_error) / Ts;
    // 4. 三项叠加 + 输出限幅
    Output = proportion + pid->integral + differential;
    Output = _constrain(Output, -pid->output_limit, pid->output_limit);
    // 保存当前值
    pid->last_error = Error;
    pid->last_output = Output;
    pid->timestamp_last = Timestamp;

    return Output;
}


/* 速度环PI */
/**
  * @brief  速度环PI控制器（带时间戳 + 梯形积分，更精准）
  * @param  pid: PID结构体指针
  * @param  Error: 速度误差
  * @retval 速度环输出
  */
float Speed_PID_State(PID_TypeDef *pid, float Error)
{
    // 1. 时间戳计算（和电流环完全统一）
    float Ts = 0.0f;
    uint32_t Timestamp = SysTick->VAL;

    if(Timestamp < pid->timestamp_last)
    {
        Ts = (float)(pid->timestamp_last - Timestamp) / 9.0f * 1e-6f;
    }
    else
    {
        Ts = (float)(0xFFFFFF - Timestamp + pid->timestamp_last) / 9.0f * 1e-6f;
    }

    // 时间异常保护
    if(Ts <= 0.0f || Ts > 0.05f)
    {
        Ts = 0.001f;
    }

    // 2. 比例项
    float proportional = pid->Kp * Error;

    // 3. 梯形积分（核心改进：更平滑，无超调，精度更高）
    // 梯形积分公式：积分 = 原积分 + Ki * Ts * (当前误差 + 上一次误差) / 2
    pid->integral += pid->Ki * Ts * (Error + pid->last_error) * 0.5f;
    pid->integral = _constrain(pid->integral, -pid->integral_limit, pid->integral_limit);

    // 4. 输出计算 + 限幅
    float output = proportional + pid->integral;
    output = _constrain(output, -pid->output_limit, pid->output_limit);

    // 5. 保存历史值（必须保存，给下一次梯形积分使用）
    pid->last_error = Error;
    pid->last_output = output;
    pid->timestamp_last = Timestamp;  // 保存时间戳

    return output;
}


/* 角度环PID：纯P控制 */
float Angle_PID(float Kp, float Ki, float Kd, float Error)
{
    float output;
    float proportional;
    float integral;
    float derivative;

    proportional = Kp * Error;

    // 当前角度环按单路位置环写法：Ki/Kd传0，禁用积分和微分
    integral = 0.0f;
    derivative = 0.0f;

    output = proportional + integral + derivative;

    // 位置环输出作为速度期望，限幅
    output = _constrain(output, -50.0f, 50.0f);

    return output;
}

