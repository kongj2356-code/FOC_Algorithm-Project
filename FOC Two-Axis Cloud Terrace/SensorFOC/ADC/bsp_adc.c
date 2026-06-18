#include "./ADC/bsp_adc.h"
#include "stm32f10x.h"
#include "Serial.h"
#include "AS5600/AS5600.h"
#include "./FOC/bsp_foc.h"
#include "./PID/bsp_pid.h"
#include <math.h>
#include "Delay.h"
#include "IQ_math.h"

#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f
#define _2PI     6.28318530718f
#define M_PI     3.14159265358979f
#define ABS(x)   ((x) > 0 ? (x) : -(x))

#define POSITION_LOOP_TS        0.001f
#define POSITION_TARGET_RATE    11.0f
#define POSITION_KP             5.5f
#define POSITION_SPEED_LIMIT    14.0f
#define POSITION_DEADBAND       0.003f
#define IQ_SLEW_PER_TICK        0.05f

// 双路全局FOC控制器
FOC_ControllerTypeDef foc1 = {0};
FOC_ControllerTypeDef foc2 = {0};

// 外部零点角
extern float Zero_Angle1;
extern float Zero_Angle2;

// 双路位置环目标值
float Angle_hope1 = 0.0f;
float Angle_hope2 = 0.0f;
static float Angle_cmd1 = 0.0f;
static float Angle_cmd2 = 0.0f;
static uint8_t PositionLoop_Inited1 = 0;
static uint8_t PositionLoop_Inited2 = 0;


// PID初始化标志
static uint8_t FOC_PID_Inited = 0;


/* 机械角解卷绕 */
float Angle_difference(float target, float current)
{
    float diff = target - current;

    while(diff > M_PI)  diff -= _2PI;
    while(diff < -M_PI) diff += _2PI;

    return diff;
}



/* 双路PID初始化 */
static void FOC_PID_Init(void)
{
    if(FOC_PID_Inited) return;

    // 电机1参数
    PID_Init(&foc1.pid_id,    0.0f,   0.0f,   0.0f, 6.3f, 6.3f);
    PID_Init(&foc1.pid_iq,    0.85f,   12.0f, 0.0f, 6.3f, 6.3f);
    PID_Init(&foc1.pid_speed, 0.30f, 0.020f, 0.0f, 0.35f, 1.20f);

    // 电机2参数
    PID_Init(&foc2.pid_id,    0.0f,   0.0f,   0.0f, 6.3f, 6.3f);
    PID_Init(&foc2.pid_iq,    1.0f,   15.0f, 0.0f, 6.3f, 6.3f);
    PID_Init(&foc2.pid_speed, 0.45f, 0.0001f, 0.0f, 0.35f, 1.20f);

    FOC_PID_Inited = 1;
}

/*Filter************************************************************************************************************/
/* 低通滤波器：Q轴电流 */
static float LPF_current_q(FOC_ControllerTypeDef *foc, float x)
{
    float y = 0.95f * foc->lpf_q_prev + 0.05f * x;
    foc->lpf_q_prev = y;
    return y;
}


/* 低通滤波器：D轴电流 */
static float LPF_current_d(FOC_ControllerTypeDef *foc, float x)
{
    float y = 0.9f * foc->lpf_d_prev + 0.1f * x;
    foc->lpf_d_prev = y;
    return y;
}


/* 低通滤波器：速度 */
static float LPF_current_Speed(FOC_ControllerTypeDef *foc, float x)
{
    float y = 0.95f * foc->lpf_speed_prev + 0.05f * x;
    foc->lpf_speed_prev = y;
    return y;
}

/***************************************************************************************************************Filter*/


/*ADC*********************************************************************************************************************/
/* ADC初始化 —— 双ADC同步4通道采样 */
void AD_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    FOC_PID_Init();

    // 1. 开启ADC1、ADC2、GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_GPIOA, ENABLE);

    // 2. 配置PA0、PA1、PA2、PA3 为模拟输入模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. ADC时钟配置：ADCCLK = PCLK2 / 6 = 12MHz
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    // 4. 复位ADC1/ADC2
    ADC_DeInit(ADC1);
    ADC_DeInit(ADC2);

    // 5. ADC通用配置
    ADC_InitStructure.ADC_Mode = ADC_Mode_InjecSimult;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;

    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Init(ADC2, &ADC_InitStructure);

    // 6. 配置注入组触发源：TIM1_TRGO
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
    ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T1_TRGO);

    // 7. 注入组长度 = 2
    ADC_InjectedSequencerLengthConfig(ADC1, 2);
    ADC_InjectedSequencerLengthConfig(ADC2, 2);

    // ADC1：电机1电流采样 PA0、PA1
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_28Cycles5);

    // ADC2：电机2电流采样 PA2、PA3
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_2, 1, ADC_SampleTime_28Cycles5);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_3, 2, ADC_SampleTime_28Cycles5);

    // 8. 允许外部触发注入转换
    ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);
    ADC_ExternalTrigInjectedConvCmd(ADC2, ENABLE);

    // 9. 启用ADC
    ADC_Cmd(ADC1, ENABLE);
    ADC_Cmd(ADC2, ENABLE);

    // 10. 校准ADC
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

    ADC_ResetCalibration(ADC2);
    while(ADC_GetResetCalibrationStatus(ADC2));
    ADC_StartCalibration(ADC2);
    while(ADC_GetCalibrationStatus(ADC2));

    // 11. 中断配置
    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 测试中断频率 PA4
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}


/* ADC 中断 —— 4通道同步采样 */
void ADC1_2_IRQHandler(void)
{
    if (ADC_GetITStatus(ADC1, ADC_IT_JEOC) != RESET)
    {
        // 电机1：PA0、PA1
        foc1.samp_volts[0] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
        foc1.samp_volts[1] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);

        // 电机2：PA2、PA3
        foc2.samp_volts[0] = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);
        foc2.samp_volts[1] = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_2);

        ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
      //  GPIO_WriteBit(GPIOA, GPIO_Pin_4, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4));
    }
}
/*********************************************************************************************************************ADC*/

/* 电流零点校准 */
void DriftOffsets(void)
{
    uint16_t detect_rounds = 1000;

    float sum1_ia = 0.0f,sum1_ib = 0.0f;
    float sum2_ia = 0.0f,sum2_ib = 0.0f;

    setPhaseVoltage_FOC2(MOTOR_1, 0.0f, 0.0f, 0.0f);
    setPhaseVoltage_FOC2(MOTOR_2, 0.0f, 0.0f, 0.0f);

    Delay_ms(10);

    for (int i = 0; i < detect_rounds; i++)
    {
        sum1_ia += foc1.samp_volts[0];
        sum1_ib += foc1.samp_volts[1];

        sum2_ia += foc2.samp_volts[0];
        sum2_ib += foc2.samp_volts[1];
    }

    foc1.offset_ia = sum1_ia / detect_rounds;
    foc1.offset_ib = sum1_ib / detect_rounds;

    foc2.offset_ia = sum2_ia / detect_rounds;
    foc2.offset_ib = sum2_ib / detect_rounds;

    PID_Reset(&foc1.pid_id);
    PID_Reset(&foc1.pid_iq);
    PID_Reset(&foc1.pid_speed);

    PID_Reset(&foc2.pid_id);
    PID_Reset(&foc2.pid_iq);
    PID_Reset(&foc2.pid_speed);
}


/* 通用电流环 */
static void Set_CurTorque(FOC_ControllerTypeDef *foc, Motor_NumTypeDef motor_num, float Target_Current)
{
    float err_q;
    float err_d;

    // 电流采样
    foc->ele.Ia = -(foc->samp_volts[0] - foc->offset_ia) * 0.0016113f;
    foc->ele.Ib = -(foc->samp_volts[1] - foc->offset_ib) * 0.0016113f;
    foc->ele.Ic = -(foc->ele.Ia + foc->ele.Ib);

    // Clark
    foc->ele.I_alpha = foc->ele.Ia;
    foc->ele.I_beta = _1_SQRT3 * foc->ele.Ia + _2_SQRT3 * foc->ele.Ib;

    // Park
    foc->ele.Iq = foc->ele.I_beta * _cos(foc->angle_ele) - foc->ele.I_alpha * _sin(foc->angle_ele);
    foc->ele.Id = foc->ele.I_alpha * _cos(foc->angle_ele) + foc->ele.I_beta * _sin(foc->angle_ele);

    // 滤波
    foc->ele.Iq = LPF_current_q(foc, foc->ele.Iq);
    foc->ele.Id = LPF_current_d(foc, foc->ele.Id);

    // 电流环PID
    err_q = Target_Current - foc->ele.Iq;
    err_d = foc->id_hope - foc->ele.Id;

    foc->ud = PID_Controller_State(&foc->pid_id, err_d);
    foc->uq = PID_Controller_State(&foc->pid_iq, err_q);

    // 输出到对应电机
    setPhaseVoltage_FOC2(motor_num, foc->uq, foc->ud, foc->angle_ele);
}


/*Loop接口******************************************************************/

/* 电机1电流环 */
void M1_Set_CurTorque(float Target_Current)
{
    Set_CurTorque(&foc1, MOTOR_1, Target_Current);
}


/* 电机2电流环 */
void M2_Set_CurTorque(float Target_Current)
{
    Set_CurTorque(&foc2, MOTOR_2, Target_Current);
}


///* 电机1速度环接口：保留兼容 */
//void M1_Set_Velocity(float Target_Velocity)
//{
//    foc1.speed_hope = Target_Velocity;
//    M1_Set_CurTorque(foc1.iq_hope);
//}


///* 电机2速度环接口：保留兼容 */
//void M2_Set_Velocity(float Target_Velocity)
//{
//    foc2.speed_hope = Target_Velocity;
//    M2_Set_CurTorque(foc2.iq_hope);
//}


/* 电机1位置环接口 */
void M1_Set_Position(float target_angle)
{
    Angle_cmd1 = target_angle;
}


/* 电机2位置环接口 */
void M2_Set_Position(float target_angle)
{
    Angle_cmd2 = target_angle;
}

/*************************************************************************Loop接口*/


static float RampAngleTarget(float target, float current)
{
    float step_limit = POSITION_TARGET_RATE * POSITION_LOOP_TS;
    float diff = Angle_difference(target, current);

    if(diff > step_limit)  diff = step_limit;
    if(diff < -step_limit) diff = -step_limit;

    return current + diff;
}

/* 通用位置闭环更新：位置环 -> 速度环 -> 电流给定 */
static void PositionLoop_Update(FOC_ControllerTypeDef *foc, float Angle, float Zero_Angle, float Angle_cmd, float *Angle_hope)
{
    float angle_err;
    float speed_error;
    float iq_target;
    float iq_delta;

    foc->angle_ele = Angle * 7 - Zero_Angle;

    {
        uint8_t *position_loop_inited = (foc == &foc1) ? &PositionLoop_Inited1 : &PositionLoop_Inited2;
        if(!(*position_loop_inited))
        {
            *Angle_hope = Angle_cmd;
            foc->angle_pre = Angle;
            foc->speed = 0.0f;
            foc->speed_pre = 0.0f;
            foc->speed_hope = 0.0f;
            foc->iq_hope = 0.0f;
            *position_loop_inited = 1;
        }
    }

    // 1ms位置环下把角度差换算成rad/s，速度反馈才有阻尼意义。
    foc->speed = Angle_difference(Angle, foc->angle_pre) / POSITION_LOOP_TS;
    foc->speed = LPF_current_Speed(foc, foc->speed);

    *Angle_hope = RampAngleTarget(Angle_cmd, *Angle_hope);

    // 位置环P控制 -> 输出速度命令，小误差区清零，减少来回摆动。
    angle_err = Angle_difference(*Angle_hope, Angle);
    if(ABS(angle_err) < POSITION_DEADBAND)
    {
        angle_err = 0.0f;
        foc->speed_hope = 0.0f;
    }
    else
    {
        foc->speed_hope = Angle_PID(POSITION_KP, 0.0f, 0.0f, angle_err);
        if(foc->speed_hope > POSITION_SPEED_LIMIT)  foc->speed_hope = POSITION_SPEED_LIMIT;
        if(foc->speed_hope < -POSITION_SPEED_LIMIT) foc->speed_hope = -POSITION_SPEED_LIMIT;
    }

    // 速度环PI -> 电流命令，并限制电流给定变化率，抑制大幅超调。
    speed_error = foc->speed_hope - foc->speed;
    iq_target = Speed_PID_State(&foc->pid_speed, speed_error);
    iq_delta = iq_target - foc->iq_hope;
    if(iq_delta > IQ_SLEW_PER_TICK)  iq_delta = IQ_SLEW_PER_TICK;
    if(iq_delta < -IQ_SLEW_PER_TICK) iq_delta = -IQ_SLEW_PER_TICK;
    foc->iq_hope += iq_delta;

    // 保存历史值
    foc->angle_pre = Angle;
    foc->speed_pre = foc->speed;
}


/* TIM4 中断 —— 双路位置闭环 */
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
    {
        float Angle1 = GetAngle(AS5600_CHANNEL_MOTOR1);
        float Angle2 = GetAngle(AS5600_CHANNEL_MOTOR2);

        // 位置环 -> 速度环
        PositionLoop_Update(&foc1, Angle1, Zero_Angle1, Angle_cmd1, &Angle_hope1);
        PositionLoop_Update(&foc2, Angle2, Zero_Angle2, Angle_cmd2, &Angle_hope2);

        // 电流环
        M1_Set_CurTorque(foc1.iq_hope);
        M2_Set_CurTorque(foc2.iq_hope);

        //GPIO_WriteBit(GPIOA, GPIO_Pin_4, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4));

        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}


/*
位置指令 (Serial_RxData)
    |
    | 位置环
    v
位置控制器 (M1/M2_Set_Position) <--- 位置反馈 (AS5600角度传感器)
    |
    | 速度指令
    v
速度控制器 <--- 速度反馈 (从位置计算)
    |
    | 电流指令 (iq_hope)
    v
电流控制器 <--- 电流反馈 (ADC采样)
    |
    | 电压指令 (uq)
    v
FOC控制器 (foc1/foc2)
    |
    | PWM信号
    v
电机1/电机2
*/

