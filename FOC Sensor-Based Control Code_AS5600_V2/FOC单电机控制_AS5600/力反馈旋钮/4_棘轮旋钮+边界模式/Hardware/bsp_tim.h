#ifndef __BSP_TIM_H
#define __BSP_TIM_H

#include "stm32f10x.h"
#include <stdint.h>


#define M_PI     3.14159265358979f
#define _2PI     6.28318530718f
#define abs(x) ((x)>0 ? (x) :(-(x)))

/* ================== 边界模式配置 ================== */
#define BOUNDARY_MODE_ENABLE  0           // 1: 启用边界模式, 0: 保持原有棘轮模式
#define ANGLE_RANGE           180.0f      // 允许转动的角度范围（deg）


/* ================== X分度棘轮旋钮配置 ================== */
#define GEAR_NUM         9                          // ****** 修改这个可进行分度修改                                 
#define GEAR_ANGLE_STEP  (2 * M_PI / GEAR_NUM)      // 每个齿轮卡位的角度间隔（rad）
#define DEAD_ZONE        0.1f                  // 角度死区（单位：rad）
                                                 // 作用：过滤硬件角度检测的微小误差，避免电机在卡位处频繁抖动、异响
                                                 // 调整原则：
                                                 // 1. 死区值 < 齿轮步长的1/5（EX:9分度步长≈0.698rad，故死区<0.14rad），保证卡位定位精度
                                                 // 2. 若卡位处电机抖动，可小幅增大（如0.06f≈3.44°）；若定位不准，可小幅减小（如0.04f≈2.29°）
                               
void Ratchet_Knob8_Init(void);
float Calculate_Target_Detent(float current_angle);
float Angle_difference(float target, float current);


#endif /* __BSP_TIM_H */