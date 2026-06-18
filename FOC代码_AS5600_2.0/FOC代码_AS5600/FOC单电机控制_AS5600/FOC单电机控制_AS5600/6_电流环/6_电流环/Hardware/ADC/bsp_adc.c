#include "./ADC/bsp_adc.h"
#include "stm32f10x.h"                  // Device header
#include "Serial.h"
#include "AS5600/AS5600.h"
#include "./FOC/bsp_foc.h"
#include "./PID/bsp_pid.h"
#include <math.h>
#include "Delay.h"

#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f
#define abs(x) ((x)>0?(x):-(x))
#define _2PI 6.28318530718f

ElectricalQuantities b = {0};

static float y_current_q_prev = 0, y_current_d_prev = 0;
float Iq_hope = 0.00f;      // 期望Q轴电流
float Id_hope = 0.00f;      // 期望D轴电流（通常设为0）
float Uq, Ud;              // Q轴和D轴输出电压
uint16_t adc1_val, adc2_val;
extern float Zero_Angle;   // 零点偏移角
float offset_ia, offset_ib;


float Kp_d = 0 ,Ki_d = 0;



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

/* ADC初始化函数*****************************************/
void ADC_InjectedSync_Init(void)
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
}



// 零飘检测
void DriftOffsets(void)
{
    uint16_t detect_rounds = 1000;
    float sum_ia = 0, sum_ib = 0;  // 使用局部变量累加
    
    // 确保电机静止，PWM输出为0
    setPhaseVoltage_FOC2(0.0f, 0.0f, 0.0f);
    Delay_ms(10);                  // 短暂延时确保稳定
    
    for(int i = 0; i < detect_rounds; i++)
    {
        adc1_val = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
        adc2_val = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);
        
        sum_ia += adc1_val; 
        sum_ib += adc2_val;
    }
    
    offset_ia = sum_ia / detect_rounds; 
    offset_ib = sum_ib / detect_rounds; 


}



/*************************************************
 * 函数名称: ADC1_2_IRQHandler
 * 功能说明: ADC注入通道同步采样完成中断
 *************************************************/
void ADC1_2_IRQHandler(void)
{
    if (ADC_GetITStatus(ADC1, ADC_IT_JEOC))  // 判断注入采样完成
    {
        float Angle,Angle_el,Error_Iq,Error_Id,Angle_data,Angle_ele;
        Angle = GetAngle();
        Angle_ele = Angle * 7;
        Angle_el = Angle_ele - Zero_Angle;
        // 读取两个ADC注入组结果
        adc1_val = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
        adc2_val = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);
				//基尔霍夫定律
				b.Ia = -(adc1_val - offset_ia)*3.3/4096/50/0.01;
				b.Ib = -(adc2_val - offset_ib)*3.3/4096/50/0.01;
				b.Ic = -(b.Ia + b.Ib);
				// Clark变换
			  b.I_alpha = b.Ia;
				b.I_beta = _1_SQRT3 * b.Ia + _2_SQRT3 * b.Ib;
				// Park变换
				float cos_t = cos(Angle_el);
				float sin_t = sin(Angle_el);
				b.Iq = b.I_beta * cos_t - b.I_alpha * sin_t;
				b.Id = b.I_alpha * cos_t + b.I_beta * sin_t;

				b.Iq = LPF_current_q(b.Iq);
				b.Id = LPF_current_d(b.Id);

				Error_Iq = Iq_hope - b.Iq;
				Error_Id = Id_hope - b.Id;
				Ud = PID_Controller(Kp_d,Ki_d,0,Error_Id);
				Uq = PID_Controller(0.5,200,0,Error_Iq);

			setPhaseVoltage_FOC2(Uq,0,Angle_el);
      ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
    }
}



/* 启用/禁用电流环 **************************************************/
void CurrentLoop_Enable(uint8_t enable)
{
    if(enable)
    {
        // 使能ADC中断
        ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
    }
    else
    {
        // 禁用ADC中断
        ADC_ITConfig(ADC1, ADC_IT_JEOC, DISABLE);
        
        // 设置零输出电压
        setPhaseVoltage_FOC2(0.0f, 0.0f, 0.0f);
    }
}

