#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "stm32f10x.h"

typedef struct
{
    float Ia;
    float Ib;
    float Ic;
    float I_alpha;
    float I_beta;
    float Iq;
    float Id;
} ElectricalQuantities;

// 电机控制总结构体（全部变量封装在这里）
typedef struct
{
    // 电气量
    ElectricalQuantities ele;

    // 滤波历史值
    float lpf_q_prev;
    float lpf_d_prev;
    float lpf_speed_prev;

    // 电流环
    float iq_hope;
    float id_hope;
    float uq;
    float ud;

    // ADC采样
    uint16_t samp_volts[2];
    float offset_ia;
    float offset_ib;

    // 速度环
    float speed_hope;
    float speed;
    float speed_pre;

    // 角度
    float angle_pre;
    float angle_ele;

} FOC_ControllerTypeDef;


void DriftOffsets(void);
void AD_Init(void);
void BSP_TIM(void);

void M1_Set_Velocity(float Target_Velocity);
void M1_Set_CurTorque(float Target_Current);

#endif
