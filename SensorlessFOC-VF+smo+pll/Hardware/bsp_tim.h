#ifndef __BSP_TIM_H
#define __BSP_TIM_H

#include "stm32f10x.h"

/* ================== TIM2 初始化 ================== */
/**
 * @brief  初始化 TIM2，产生 1ms 周期中断
 * @note   中断中只置位 TIM2_1ms_Flag，不执行控制算法
 */
void BSP_TIM(void);

/* ================== 1ms 调度标志 ================== */
/**
 * @brief  TIM2 1ms 调度标志
 * @note   在 TIM2_IRQHandler 中置 1，在 main 中清 0
 */
extern volatile uint8_t TIM2_1ms_Flag;

/* ================== FOC 主控制函数 ================== */
/**
 * @brief  1ms 执行一次的 FOC + 速度环
 * @note   由 main() 在检测到 TIM2_1ms_Flag 后调用
 */
void BSP_FOC_1ms(void);

#endif /* __BSP_TIM_H */
