#ifndef __BSP_TIM_H
#define __BSP_TIM_H

#include "stm32f10x.h"
#include <stdint.h>

#define M_PI     3.14159265358979f
#define _2PI     6.28318530718f
#define abs(x) ((x)>0 ? (x) :(-(x)))

/* ================== 模式定义 ================== */
#define MODE_RATCHET        0   // 普通棘轮模式
#define MODE_BOUNDARY       1   // 边界棘轮模式
#define MODE_DAMP           2   // 阻尼模式
#define MODE_SMOOTH         3   // 顺滑模式

/* ================== 边界模式配置 ================== */
#define ANGLE_RANGE         180.0f      // 允许转动的角度范围（deg）

/* ================== X分度棘轮旋钮配置 ================== */
#define GEAR_NUM            8                          // 修改这个可进行分度修改                                 
#define GEAR_ANGLE_STEP     (2 * M_PI / GEAR_NUM)      // 每个齿轮卡位的角度间隔（rad）
#define DEAD_ZONE           0.1f                      // 角度死区（单位：rad）
                                                       // 作用：过滤硬件角度检测的微小误差，避免电机在卡位处频繁抖动、异响
                                                       // 调整原则：
                                                       // 1. 死区值 < 齿轮步长的1/5（EX:9分度步长≈0.698rad，故死区<0.14rad），保证卡位定位精度
                                                       // 2. 若卡位处电机抖动，可小幅增大（如0.06f≈3.44°）；若定位不准，可小幅减小（如0.04f≈2.29°）

/* ================== 阻尼/顺滑模式参数 ================== */
#define KP_DAMP             0.15f    // 阻尼模式比例系数（越大阻尼感越强）
#define KP_SMOOTH           0.025f   // 顺滑模式比例系数（越大顺滑感越强）
#define VOLTAGE_LIMIT       3.3f     // FOC最大输出电压（适配3.3V供电）

// 函数声明
void Knob_Init(void);
float Calculate_Target_Detent(float current_angle);
float Angle_difference(float target, float current);
void Set_Knob_Mode(uint8_t mode);  // 模式切换函数
void BSP_TIM(void);                // 补充声明，避免编译警告

#endif /* __BSP_TIM_H */
