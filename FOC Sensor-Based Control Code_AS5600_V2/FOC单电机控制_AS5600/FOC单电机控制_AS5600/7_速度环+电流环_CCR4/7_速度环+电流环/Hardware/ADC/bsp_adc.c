#include "./ADC/bsp_adc.h"
#include "stm32f10x.h"                  // Device header
#include "Serial.h"
#include "AS5600/AS5600.h"
#include "./FOC/bsp_foc.h"
#include "./PID/bsp_pid.h"
#include <math.h>
#include "Delay.h"
#include "IQ_math.h"

#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f
#define abs(x) ((x)>0?(x):-(x))
#define _2PI 6.28318530718f

ElectricalQuantities b = {0};

static float y_current_q_prev = 0, y_current_d_prev = 0,y_current_Speed_prev;
float Iq_hope= 0.00f;             // 期望Q轴电流
float Id_hope = 0.00f;      // 期望D轴电流（通常设为0）
float Uq, Ud;                // Q轴和D轴输出电压
uint16_t adc1_val, adc2_val;
extern float Zero_Angle;   // 零点偏移角
float offset_ia, offset_ib;


float Speed_hope = 0;
float Speed;
unsigned char Time_Speed = 0;
float Speed_pre = 0.00f ,Angle_pre = 0.00f;

// DMA 自动存放 ADC1 规则组 2 通道结果：Channel8(PB0), Channel9(PB1)
uint16_t Samp_volts[2];
// 电角度（全局变量，TIM2更新，电流环使用）
float Angle_ele = 0.0f;


/* 低通滤波器：Q轴电流 **********************************************/
float LPF_current_q(float x)
{
    float y = 0.9f * y_current_q_prev + 0.1f * x;
    y_current_q_prev = y;
    return y;
}


/* 低通滤波器：D轴电流 **********************************************/
float LPF_current_d(float x)
{
    float y = 0.9f * y_current_d_prev + 0.1f * x;
    y_current_d_prev = y;
    return y;
}

/* 低通滤波器：速度 **********************************************/
float LPF_current_Speed(float x)
{
	float y = 0.95*y_current_Speed_prev + 0.05*x;
	
  y_current_Speed_prev=y;
	
	return y;
}

/* ADC初始化函数*****************************************/
void ADC_InjectedSync_Init(void)
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

    ADC_InitStructure.ADC_Mode = ADC_Mode_InjecSimult;        // 同步注入
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Init(ADC2, &ADC_InitStructure);

    // ===================== 【关键】触发源改为 TIM1_CC4 =====================
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4);
    ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T1_CC4);
    // =======================================================================

    ADC_InjectedSequencerLengthConfig(ADC1, 1);
    ADC_InjectedSequencerLengthConfig(ADC2, 1);

    ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_28Cycles5);

    ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);
    ADC_ExternalTrigInjectedConvCmd(ADC2, ENABLE);

    ADC_Cmd(ADC1, ENABLE);
    ADC_Cmd(ADC2, ENABLE);

    // 校准
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

    ADC_ResetCalibration(ADC2);
    while(ADC_GetResetCalibrationStatus(ADC2));
    ADC_StartCalibration(ADC2);
    while(ADC_GetCalibrationStatus(ADC2));

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


 
/* ================== TIM2 初始化 ================== */
void BSP_TIM()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;   // 1ms 中断
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


// ADC1与ADC2共用中断
void ADC1_2_IRQHandler(void)
{
    // 判断注入组转换完成中断
    if(ADC_GetITStatus(ADC1, ADC_IT_JEOC) != RESET)
    {
        // 读取注入组转换结果
        Samp_volts[0] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
        Samp_volts[1] = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);
        
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4));
        // 清除中断标志位
       ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
    }
}

void DriftOffsets(void)
{
    uint16_t detect_rounds = 1000;
    float sum_ia = 0, sum_ib = 0;
    
    setPhaseVoltage_FOC2(0.0f, 0.0f, 0.0f);
    Delay_ms(10);
    
    for(int i = 0; i < detect_rounds; i++)
    {
        sum_ia += Samp_volts[0];
        sum_ib += Samp_volts[1];
    }
    
    offset_ia = sum_ia / detect_rounds;
    offset_ib = sum_ib / detect_rounds;
}

// ==============================
// [底层]电流环 
// ==============================
void M1_Set_CurTorque(float Target_Current)
{
    // 1. 直接读取
    b.Ia = -(Samp_volts[0] - offset_ia) * 3.3f / 4096.0f / 50.0f / 0.01f;
    b.Ib = -(Samp_volts[1] - offset_ia) * 3.3f / 4096.0f / 50.0f / 0.01f;
    b.Ic = -(b.Ia + b.Ib);

    // 2. Clark变换
    b.I_alpha = b.Ia;
    b.I_beta = _1_SQRT3 * b.Ia + _2_SQRT3 * b.Ib;

    // 3. 使用TIM2中断更新的电角度
    float Angle_el = Angle_ele;

    // 4. Park变换
    float cos_t = _cos(Angle_el);
    float sin_t = _sin(Angle_el);
    b.Iq = b.I_beta * cos_t - b.I_alpha * sin_t;
    b.Id = b.I_alpha * cos_t + b.I_beta * sin_t;

    // 5. 滤波
    b.Iq = LPF_current_q(b.Iq);
    b.Id = LPF_current_d(b.Id);

    // 6. 电流环PID
    float Error_Iq = Target_Current - b.Iq;
    float Error_Id = Id_hope - b.Id;
    Ud = PID_Controller(0, 0, 0, Error_Id);
    Uq = PID_Controller(10.5f, 50.0f, 0.0f, Error_Iq);

    // 7. SVPWM输出
    setPhaseVoltage_FOC2(Uq, 0, Angle_el);
}


// ==============================
// [顶层]速度环（现在只做调用，计算在TIM2）
// ==============================
void M1_Set_Velocity(float Target_Velocity)
{
    // 直接把目标速度赋值，速度计算在TIM2中断
    Speed_hope = Target_Velocity;

    // 电流环持续运行
    M1_Set_CurTorque(Iq_hope);
}


/* ================== TIM2中断服务函数：1ms 执行一次 ================== */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        // 1. 读取角度
        float Angle = GetAngle();
        const float POLE_PAIRS = 7;

        // 2. 更新电角度（给电流环用）
        Angle_ele = Angle * POLE_PAIRS - Zero_Angle;

        // 3. 速度计算（原速度环逻辑）
        float Angle_data;
        if(abs(Angle - Angle_pre) > 0.75f * _2PI)
        {
            Angle_data = Angle + _2PI;
        }
        else
        {
            Angle_data = Angle;
        }

        // 计算速度
        Speed = (Angle_data - Angle_pre) / 0.001f;
        Speed = LPF_current_Speed(Speed);

        // 4. 速度PID → 输出 Iq 期望
        Iq_hope = Speed_PID(0.025f, 0.001f, 0.0f, Speed_hope - Speed);

        // 5. 翻转测试引脚
     //   GPIO_WriteBit(GPIOA, GPIO_Pin_4, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4));

        // 6. 保存历史值
        Angle_pre = Angle;
        Speed_pre = Speed;

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

