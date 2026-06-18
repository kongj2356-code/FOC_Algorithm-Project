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
#define M_PI 3.14159265358979f

// 全局实例化
FOC_ControllerTypeDef foc = {0};

// 外部零点角
extern float Zero_Angle;

// 位置环（角度环）目标值
float Angle_hope = 0.0f;


/* ================== 机械角解卷绕（必须加！） ================== */
float Angle_difference(float target, float current)
{
    float diff = target - current;
    while(diff > M_PI)  diff -= 2 * M_PI;
    while(diff < -M_PI) diff += 2 * M_PI;
    return diff;
}



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
     // 定义初始化结构体
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 开启外设时钟：ADC1、ADC2、GPIOA
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_GPIOA, ENABLE);

    // 配置GPIO引脚为模拟输入模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;  // PA0和PA1引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;           // 模拟输入模式
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置ADC时钟：PCLK2的6分频
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    // 复位ADC1和ADC2到默认状态
    ADC_DeInit(ADC1);
    ADC_DeInit(ADC2);

    // 配置ADC基本参数
    ADC_InitStructure.ADC_Mode = ADC_Mode_InjecSimult;        // 同步注入模式（ADC1和ADC2同步转换）
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;            // 扫描模式禁止（单通道）
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;      // 连续转换模式禁止（由外部触发）
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 规则通道不使用外部触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;   // 数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 1;                  // 规则通道数量（此处不使用规则通道）
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Init(ADC2, &ADC_InitStructure);

    // 配置注入通道的外部触发源：TIM1的TRGO事件
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
    ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T1_TRGO);

    // 配置注入序列长度：1个通道
    ADC_InjectedSequencerLengthConfig(ADC1, 1);
    ADC_InjectedSequencerLengthConfig(ADC2, 1);

    // 配置注入通道参数
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5); // ADC1使用通道0（PA0）
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_28Cycles5); // ADC2使用通道1（PA1）

    // 使能注入通道的外部触发
    ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);
    ADC_ExternalTrigInjectedConvCmd(ADC2, ENABLE);

    // 使能ADC1和ADC2
    ADC_Cmd(ADC1, ENABLE);
    ADC_Cmd(ADC2, ENABLE);

    // ADC1校准过程
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

    // ADC2校准过程
    ADC_ResetCalibration(ADC2);
    while(ADC_GetResetCalibrationStatus(ADC2));
    ADC_StartCalibration(ADC2);
    while(ADC_GetCalibrationStatus(ADC2));

    // 使能ADC1的注入转换结束中断
    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);

    // 配置ADC1_2中断（ADC1和ADC2共用同一个中断向量）
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;           // 中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   // 抢占优先级2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          // 子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             // 使能中断
    NVIC_Init(&NVIC_InitStructure);

    // 测试IO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/* TIM2初始化--0.001s*/
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
    foc.uq = PID_Controller(5.5f, 100.0f, 0.0f, err_q);

    // 输出
    setPhaseVoltage_FOC2(foc.uq, 0, Angle_el);
}


/* 速度环 */
void M1_Set_Velocity(float Target_Velocity)
{
    foc.speed_hope = Target_Velocity;
    M1_Set_CurTorque(foc.iq_hope);
}


/* 位置环（外部调用） */
void M1_Set_Position(float target_angle)
{
    Angle_hope = target_angle;
}


/* TIM2 中断 三闭环核心：位置环 → 速度环 → 电流环 */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        float Angle = GetAngle();
        foc.angle_ele = Angle * 7;                  // 将机械角度转换为电角度（极对数为7）

				foc.speed = (Angle - foc.angle_pre);
				foc.speed = LPF_current_Speed(foc.speed);

        // 位置环P控制→输出速度期望值
        float angle_err = Angle_difference(Angle_hope, Angle);
        foc.speed_hope =  Angle_PID(8.0f,0,0, angle_err);  

        // 速度环PID→输出电流期望值
        foc.iq_hope = Speed_PID(0.350f, 0.001f, 0.0f, foc.speed_hope - foc.speed);

        // 电流环在ADC中断里自动运行
        M1_Set_CurTorque(foc.iq_hope);

        // 测试IO翻转
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4));

        // 保存历史值
        foc.angle_pre = Angle;
        foc.speed_pre = foc.speed;

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
