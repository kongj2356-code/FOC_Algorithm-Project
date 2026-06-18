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



// SMO滑模观测器 结构体
typedef struct {
    float Ealpha;         // α轴反电动势观测值
    float Ebeta;          // β轴反电动势观测值
    float Est_Ialpha;     // α轴估计电流
    float Est_Ibeta;      // β轴估计电流
    float Ealpha_flt;     // α轴反电动势滤波值
    float Ebeta_flt;      // β轴反电动势滤波值
    float Est_theta;      // SMO观测电角度
    
    float Rs;            // 定子相电阻 Ω
    float Ls;            // 定子相电感 H
    float h;             // 滑模增益
    
    float Ualpha;        // α轴电压输入
    float Ubeta;         // β轴电压输入
    
    float angle_prev;    // 上一帧角度
    int32_t full_rot;    // 总圈数
    float absolute_theta;// 连续绝对角度
} SMO_HandleTypeDef;


// PLL锁相环 结构体
typedef struct {
    float theta_err;     // 角度误差
    float est_theta;     // 估算电角度
    float est_speed;     // 估算电角速度
    float est_speed_F;   // 滤波后角速度
    float i_err;         // PI积分项
    
    float kp;            // 比例系数
    float ki;            // 积分系数
} PLL_HandleTypeDef;


// VF启动
typedef struct
{
    uint8_t   VF_flag;             // 0=未启动 1=启动完成 2=启动中
    float     Target_Vel_openloop; // 开环目标速度（机械角速度 rad/s）
    float     VF_max_vel;          // 开环最大速度（机械 rad/s）
    float     VF_acc;              // 开环加速度（机械 rad/s^2）
    float     VF_uq_delta;         // 随速度增加的q轴电压增量
    float     shaft_angle;         // 开环机械角度
    uint32_t  open_loop_timestamp; // 这里先保留，不强依赖
    uint32_t  init_time;           // 启动计时（ms）
    int       _dir;                // 方向 1正转 -1反转
} VF_HandleTypeDef;



void DriftOffsets(void);
void AD_Init(void);
void BSP_TIM(void);
void BSP_TIM4(void);
void BSP_TIM3(void);

void M1_Set_Velocity(float Target_Velocity);
void M1_Set_CurTorque(float Target_Current);
void SMO_position_estimate(void) ;
void SMO_PLL_calc(float alpha, float beta);

void VF_Start_Init(int dir);
void Sensorless_CloseLoop_Start(float target_speed);
void VF_Run_Task(void);
#endif
