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
#define abs(x) ((x) > 0 ? (x) : -(x))
#define _2PI 6.28318530718f


// 全局实例化
FOC_ControllerTypeDef foc = {0};

// 外部零点角
extern float Zero_Angle;

/* 低通滤波器：Q轴电流 */
float LPF_current_q(float x)
{
    float y = 0.95f * foc.lpf_q_prev + 0.05f * x;
    foc.lpf_q_prev = y;
    return y;
}

/* 低通滤波器：D轴电流 */
float LPF_current_d(float x)
{
    float y = 0.9f * foc.lpf_d_prev + 0.1f * x;
    foc.lpf_d_prev = y;
    return y;
}

/* 低通滤波器：速度 */
float LPF_current_Speed(float x)
{
    float y = 0.95f * foc.lpf_speed_prev + 0.05f * x;
    foc.lpf_speed_prev = y;
    return y;
}

/* ADC初始化 */
void AD_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    ADC_DeInit(ADC1);
    ADC_DeInit(ADC2);

    ADC_InitStructure.ADC_Mode = ADC_Mode_InjecSimult;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Init(ADC2, &ADC_InitStructure);

    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
    ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T1_TRGO);

    ADC_InjectedSequencerLengthConfig(ADC1, 1);
    ADC_InjectedSequencerLengthConfig(ADC2, 1);

    ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_28Cycles5);

    ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);
    ADC_ExternalTrigInjectedConvCmd(ADC2, ENABLE);

    ADC_Cmd(ADC1, ENABLE);
    ADC_Cmd(ADC2, ENABLE);

    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

    ADC_ResetCalibration(ADC2);
    while (ADC_GetResetCalibrationStatus(ADC2));
    ADC_StartCalibration(ADC2);
    while (ADC_GetCalibrationStatus(ADC2));

    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 测试IO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/* TIM2 初始化 */
void BSP_TIM(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;
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

/* ADC 中断 */
void ADC1_2_IRQHandler(void)
{
    if (ADC_GetITStatus(ADC1, ADC_IT_JEOC) != RESET)
    {
        foc.samp_volts[0] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
        foc.samp_volts[1] = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);
        ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
    }
}

/* 电流零点校准 */
void DriftOffsets(void)
{
    uint16_t detect_rounds = 1000;
    float sum_ia = 0, sum_ib = 0;

    setPhaseVoltage_FOC2(0.0f, 0.0f, 0.0f);
    Delay_ms(10);

    for (int i = 0; i < detect_rounds; i++)
    {
        sum_ia += foc.samp_volts[0];
        sum_ib += foc.samp_volts[1];
    }

    foc.offset_ia = sum_ia / detect_rounds;
    foc.offset_ib = sum_ib / detect_rounds;
}

/* 电流环 */
void M1_Set_CurTorque(float Target_Current)
{
    // 电流采样
    foc.ele.Ia = -(foc.samp_volts[0] - foc.offset_ia) * 3.3f / 4096.0f / 50.0f / 0.01f;
    foc.ele.Ib = -(foc.samp_volts[1] - foc.offset_ib) * 3.3f / 4096.0f / 50.0f / 0.01f;
    foc.ele.Ic = -(foc.ele.Ia + foc.ele.Ib);

    // Clark
    foc.ele.I_alpha = foc.ele.Ia;
    foc.ele.I_beta = _1_SQRT3 * foc.ele.Ia + _2_SQRT3 * foc.ele.Ib;

    float Angle_el = foc.angle_ele;

    // Park
    float cos_t = _cos(Angle_el);
    float sin_t = _sin(Angle_el);
    foc.ele.Iq = foc.ele.I_beta * cos_t - foc.ele.I_alpha * sin_t;
    foc.ele.Id = foc.ele.I_alpha * cos_t + foc.ele.I_beta * sin_t;

    // 滤波
    foc.ele.Iq = LPF_current_q(foc.ele.Iq);
    foc.ele.Id = LPF_current_d(foc.ele.Id);

    // PID
    float err_q = Target_Current - foc.ele.Iq;
    float err_d = foc.id_hope - foc.ele.Id;
    foc.ud = PID_Controller(0, 0, 0, err_d);
    foc.uq = PID_Controller(10.5f, 50.0f, 0.0f, err_q);

    // 输出
    setPhaseVoltage_FOC2(foc.uq, 0, Angle_el);
}

/* 速度环 */
void M1_Set_Velocity(float Target_Velocity)
{
    foc.speed_hope = Target_Velocity;
    M1_Set_CurTorque(foc.iq_hope);
}

/* TIM2 中断 */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        float Angle = GetAngle();
        const float POLE_PAIRS = 7;

        foc.angle_ele = Angle * POLE_PAIRS - Zero_Angle;

        float Angle_data;
        if (abs(Angle - foc.angle_pre) > 0.75f * _2PI)
        {
            Angle_data = Angle + _2PI;
        }
        else
        {
            Angle_data = Angle;
        }

        foc.speed = (Angle_data - foc.angle_pre) / 0.001f;
        foc.speed = LPF_current_Speed(foc.speed);

        foc.iq_hope = Speed_PID(0.0150f, 0.001f, 0.0f, foc.speed_hope - foc.speed);

        GPIO_WriteBit(GPIOA, GPIO_Pin_4, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4));

        foc.angle_pre = Angle;
        foc.speed_pre = foc.speed;

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
