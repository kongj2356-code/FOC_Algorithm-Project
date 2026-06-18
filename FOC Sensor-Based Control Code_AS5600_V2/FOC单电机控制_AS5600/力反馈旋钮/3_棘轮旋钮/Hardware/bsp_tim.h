#ifndef __BSP_TIM_H
#define __BSP_TIM_H

#include "stm32f10x.h"
#include <stdint.h>


/* ================== 八分度棘轮旋钮配置 ================== */
#define GEAR_NUM        8           // ****** 修改这个可进行分度修改                                 
#define GEAR_ANGLE_STEP (2 * PI / GEAR_NUM)      // 每个齿轮卡位的角度间隔（rad）
#define DEAD_ZONE       0.02f                   // 角度死区（rad）原 DEAD_ZONE = 0.02f rad（约 1.15°）对 8 分度（45° 步长）完全够用，
                                                 // 但9分度步长更小（40°），如果实际硬件的角度误差稍大，可适当调大死区（比如 0.03f rad，约 1.72°），避免电机在卡位处频繁抖动：

void Ratchet_Knob8_Init(void);
float Calculate_Target_Detent(float current_angle);
float Angle_difference(float target, float current);


#endif /* __BSP_TIM_H */
