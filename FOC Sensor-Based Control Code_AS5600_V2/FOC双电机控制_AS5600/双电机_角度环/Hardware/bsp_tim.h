#ifndef __BSP_TIM_H
#define __BSP_TIM_H

#include "stm32f10x.h"
#include "AS5600.h"
#include "bsp_pid.h"
#include "bsp_foc.h"

/* ================== 数学常量定义 ================== */
#define M_PI     3.14159265358979f
#define _PI_3    1.04719755119659f    // π/3
#define PI       3.141592653589f
#define _2PI     6.28318530717958647f
#define _3PI_2   4.71238898038468985f

#define abs(x) ((x)>0 ? (x) :(-x))

/* ================== 电机控制模式定义（标志位取值） ================== */
#define MOTOR_CTRL_MODE_M1_ONLY    1   // 仅控制电机1
#define MOTOR_CTRL_MODE_M2_ONLY    2   // 仅控制电机2
#define MOTOR_CTRL_MODE_BOTH       3   // 同时控制两路电机


/* ================== FOC电机结构体定义（扩展角度环参数） ================== */
typedef struct
{
    // 状态变量
    float Angle_pre;        // 上一拍机械角度
    float Speed_pre;        // 上一拍速度（滤波前）
    float Speed;            // 当前速度（滤波后）
    float Uq;               // PID输出的q轴电压
    float Zero_Angle;       // 零电角度
    float Angle;            // 当前机械角度
    
    // 连续角度缓存
    float angle_raw_last;   // 上一次原始电气角
    float angle_foc;        // 连续无跳变电气角
    
    // 电机硬件配置
    uint8_t as5600_channel; // AS5600通道号
    Motor_NumTypeDef motor_id;       // 电机编号(MOTOR_1/MOTOR_2)
    
    // PID参数（分角度环和速度环）
    float Angle_Kp;         // 角度环Kp
    float Angle_Ki;         // 角度环Ki
    float Angle_Kd;         // 角度环Kd
    float Speed_Kp;         // 速度环Kp
    float Speed_Ki;         // 速度环Ki
    float Speed_Kd;         // 速度环Kd
} MotorFOC_Typedef;


/* ================== 全局变量声明（外部可访问） ================== */
extern uint8_t Motor_Ctrl_Mode;  // 电机控制模式标志位
extern float Angle_hope_M1;      // 电机1期望角度（替换原速度期望）
extern float Angle_hope_M2;      // 电机2期望角度（替换原速度期望）
extern MotorFOC_Typedef Motor1;  // 电机1实例（外部声明）
extern MotorFOC_Typedef Motor2;  // 电机2实例（外部声明）

/* ================== 函数声明 ================== */
// 设置电机控制模式（外部调用）
void Set_Motor_Ctrl_Mode(uint8_t mode);
// FOC双电机初始化
void FOC_DualMotor_Init(void);
// 零电角标定函数
void Zero_Get(MotorFOC_Typedef *motor);
// 一阶低通滤波函数
float LPE_VELOCITY(MotorFOC_Typedef *motor, float x);
// 连续角度转换函数
float FOC_Angle_Continuous(MotorFOC_Typedef *motor, float angle_raw);
// 机械角解卷绕函数
float Angle_difference(float target, float current);

#endif /* __BSP_TIM_H */

