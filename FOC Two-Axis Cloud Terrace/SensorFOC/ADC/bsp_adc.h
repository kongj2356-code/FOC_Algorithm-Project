#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "stm32f10x.h"
#include "./PID/bsp_pid.h"

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


// 单个电机的FOC控制结构体
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

    // 电流环PID，双路独立
    PID_TypeDef pid_iq;
    PID_TypeDef pid_id;

    // 速度环PID，双路独立
    PID_TypeDef pid_speed;

    // ADC采样：单个电机两相电流
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


// 双路FOC控制器
extern FOC_ControllerTypeDef foc1;
extern FOC_ControllerTypeDef foc2;

void DriftOffsets(void);
void AD_Init(void);
float Angle_difference(float target, float current);

void M1_Set_Velocity(float Target_Velocity);
void M1_Set_CurTorque(float Target_Current);

void M2_Set_Velocity(float Target_Velocity);
void M2_Set_CurTorque(float Target_Current);

void M1_Set_Position(float target_angle);
void M2_Set_Position(float target_angle);
#endif
